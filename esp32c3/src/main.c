/*
 * NirogScan DEV firmware - ESP32-C3 BLE gateway (ESP-IDF + NimBLE)
 * ---------------------------------------------------------------------------
 * Same streaming gateway as main, PLUS a writable control characteristic. The
 * web UI writes short text commands (e.g. "ER 256") to it; the ESP32 forwards
 * them verbatim (newline-terminated) to the RP2040 over UART, which reconfigures
 * the MAX30001 / AFE4400 live.
 *
 *   Control service  cd5c0100-4448-7db8-ae4c-d1da8cba36d0
 *   Control char     cd5c0101-4448-7db8-ae4c-d1da8cba36d0   (write / write-no-rsp)
 *
 * All notification characteristics + batching are identical to main.
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/uart.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gatt.h"
#include "os/os_mbuf.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#define DEVICE_NAME   "HealthyPi5"
static const char *TAG = "hpi5";

#define UART_PORT     UART_NUM_1
#define UART_RX_PIN   7
#define UART_TX_PIN   6
#define UART_BAUD     230400
#define PKT_DATA_SIZE 25

// ---- 128-bit UUIDs (least-significant byte first) --------------------------
static const ble_uuid128_t UUID_ECG_SVC = BLE_UUID128_INIT(
    0xfb,0x34,0x9b,0x5f,0x80,0x00,0x00,0x80,0x00,0x10,0x00,0x00,0x22,0x11,0x00,0x00);
static const ble_uuid128_t UUID_ECG_CHR = BLE_UUID128_INIT(
    0xfb,0x34,0x9b,0x5f,0x80,0x00,0x00,0x80,0x00,0x10,0x00,0x00,0x24,0x14,0x00,0x00);
static const ble_uuid128_t UUID_RESP_CHR = BLE_UUID128_INIT(
    0x02,0x00,0x12,0xac,0x42,0x02,0xeb,0xa1,0xed,0x11,0x89,0x77,0x4c,0x4a,0xbe,0xba);
static const ble_uuid128_t UUID_HRV_SVC = BLE_UUID128_INIT(
    0xd0,0x36,0xba,0x8c,0xda,0xd1,0x4c,0xae,0xb8,0x7d,0x48,0x44,0x91,0x74,0x5c,0xcd);
static const ble_uuid128_t UUID_RR_CHR = BLE_UUID128_INIT(
    0xd0,0x36,0xba,0x8c,0xda,0xd1,0x4c,0xae,0xb8,0x7d,0x48,0x44,0x6f,0xa8,0x5c,0xcd);
static const ble_uuid128_t UUID_PPG_CHR = BLE_UUID128_INIT(
    0xd0,0x36,0xba,0x8c,0xda,0xd1,0x4c,0xae,0xb8,0x7d,0x48,0x44,0x25,0x15,0x5c,0xcd);
// control service / char (cd5c0100 / cd5c0101)
static const ble_uuid128_t UUID_CTRL_SVC = BLE_UUID128_INIT(
    0xd0,0x36,0xba,0x8c,0xda,0xd1,0x4c,0xae,0xb8,0x7d,0x48,0x44,0x00,0x01,0x5c,0xcd);
static const ble_uuid128_t UUID_CTRL_CHR = BLE_UUID128_INIT(
    0xd0,0x36,0xba,0x8c,0xda,0xd1,0x4c,0xae,0xb8,0x7d,0x48,0x44,0x01,0x01,0x5c,0xcd);

static uint16_t h_hr, h_spo2, h_temp, h_bat, h_ecg, h_resp, h_rr, h_ppg;
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint8_t  own_addr_type;

#define ECG_BATCH   4
#define BIOZ_BATCH  4
static int32_t ecg_batch[ECG_BATCH];
static int32_t bioz_batch[BIOZ_BATCH];
static uint8_t ecg_bn = 0, bioz_bn = 0, ppg_decim = 0;

static inline uint32_t millis(void){ return (uint32_t)(esp_timer_get_time()/1000); }

// ---- control characteristic: forward writes to RP2040 over UART ------------
static int ctrl_access(uint16_t c, uint16_t a, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)c; (void)a; (void)arg;
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        char buf[64];
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om), out = 0;
        if (len > sizeof(buf) - 2) len = sizeof(buf) - 2;
        if (ble_hs_mbuf_to_flat(ctxt->om, buf, len, &out) == 0 && out > 0) {
            buf[out] = '\n';
            uart_write_bytes(UART_PORT, buf, out + 1);
            ESP_LOGI(TAG, "cmd -> rp2040: %.*s", out, buf);
        }
    }
    return 0;
}

static int chr_access(uint16_t c, uint16_t a, struct ble_gatt_access_ctxt *ctxt, void *arg)
{ (void)c;(void)a;(void)ctxt;(void)arg; return 0; }

static const struct ble_gatt_svc_def gatt_svcs[] = {
    { .type=BLE_GATT_SVC_TYPE_PRIMARY, .uuid=BLE_UUID16_DECLARE(0x180D),
      .characteristics=(struct ble_gatt_chr_def[]){
        { .uuid=BLE_UUID16_DECLARE(0x2A37), .access_cb=chr_access, .flags=BLE_GATT_CHR_F_NOTIFY, .val_handle=&h_hr },{0}}},
    { .type=BLE_GATT_SVC_TYPE_PRIMARY, .uuid=BLE_UUID16_DECLARE(0x1822),
      .characteristics=(struct ble_gatt_chr_def[]){
        { .uuid=BLE_UUID16_DECLARE(0x2A5E), .access_cb=chr_access, .flags=BLE_GATT_CHR_F_NOTIFY, .val_handle=&h_spo2 },{0}}},
    { .type=BLE_GATT_SVC_TYPE_PRIMARY, .uuid=BLE_UUID16_DECLARE(0x1809),
      .characteristics=(struct ble_gatt_chr_def[]){
        { .uuid=BLE_UUID16_DECLARE(0x2A6E), .access_cb=chr_access, .flags=BLE_GATT_CHR_F_NOTIFY, .val_handle=&h_temp },{0}}},
    { .type=BLE_GATT_SVC_TYPE_PRIMARY, .uuid=BLE_UUID16_DECLARE(0x180F),
      .characteristics=(struct ble_gatt_chr_def[]){
        { .uuid=BLE_UUID16_DECLARE(0x2A19), .access_cb=chr_access, .flags=BLE_GATT_CHR_F_NOTIFY|BLE_GATT_CHR_F_READ, .val_handle=&h_bat },{0}}},
    { .type=BLE_GATT_SVC_TYPE_PRIMARY, .uuid=&UUID_ECG_SVC.u,
      .characteristics=(struct ble_gatt_chr_def[]){
        { .uuid=&UUID_ECG_CHR.u,  .access_cb=chr_access, .flags=BLE_GATT_CHR_F_NOTIFY, .val_handle=&h_ecg },
        { .uuid=&UUID_RESP_CHR.u, .access_cb=chr_access, .flags=BLE_GATT_CHR_F_NOTIFY, .val_handle=&h_resp },{0}}},
    { .type=BLE_GATT_SVC_TYPE_PRIMARY, .uuid=&UUID_HRV_SVC.u,
      .characteristics=(struct ble_gatt_chr_def[]){
        { .uuid=&UUID_RR_CHR.u,  .access_cb=chr_access, .flags=BLE_GATT_CHR_F_NOTIFY, .val_handle=&h_rr },
        { .uuid=&UUID_PPG_CHR.u, .access_cb=chr_access, .flags=BLE_GATT_CHR_F_NOTIFY, .val_handle=&h_ppg },{0}}},
    { .type=BLE_GATT_SVC_TYPE_PRIMARY, .uuid=&UUID_CTRL_SVC.u,
      .characteristics=(struct ble_gatt_chr_def[]){
        { .uuid=&UUID_CTRL_CHR.u, .access_cb=ctrl_access, .flags=BLE_GATT_CHR_F_WRITE|BLE_GATT_CHR_F_WRITE_NO_RSP },{0}}},
    {0},
};

static void notify_bytes(uint16_t handle, const void *data, uint16_t len)
{
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE) return;
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (om) ble_gatts_notify_custom(conn_handle, handle, om);
}

static int gap_event(struct ble_gap_event *event, void *arg);

static void start_advertising(void)
{
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    const char *name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name; fields.name_len = strlen(name); fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);
    struct ble_gap_adv_params advp;
    memset(&advp, 0, sizeof(advp));
    advp.conn_mode = BLE_GAP_CONN_MODE_UND; advp.disc_mode = BLE_GAP_DISC_MODE_GEN;
    int rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &advp, gap_event, NULL);
    ESP_LOGI(TAG, "adv_start rc=%d", rc);
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            conn_handle = event->connect.conn_handle;
            struct ble_gap_upd_params p; memset(&p, 0, sizeof(p));
            p.itvl_min = 12; p.itvl_max = 24; p.latency = 0; p.supervision_timeout = 400;
            ble_gap_update_params(conn_handle, &p);
            ecg_bn = 0; bioz_bn = 0; ppg_decim = 0;
            ESP_LOGI(TAG, "connected");
        } else start_advertising();
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        ecg_bn = 0; bioz_bn = 0; ppg_decim = 0;
        start_advertising();
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        start_advertising();
        break;
    default: break;
    }
    return 0;
}

static void on_sync(void){ ble_hs_id_infer_auto(0, &own_addr_type); start_advertising(); }
static void on_reset(int reason){ ESP_LOGW(TAG, "reset %d", reason); }
static void host_task(void *param){ nimble_port_run(); nimble_port_freertos_deinit(); }

// ---- packet parser ---------------------------------------------------------
typedef enum { WAIT_S1, WAIT_S2, WAIT_LEN_L, WAIT_LEN_H, WAIT_TYPE, READ_DATA, WAIT_ST1, WAIT_ST2 } pstate_t;
static pstate_t pstate = WAIT_S1;
static uint8_t pbuf[PKT_DATA_SIZE]; static uint8_t pidx = 0; static uint16_t plen = 0;
static uint32_t last_hr, last_spo2, last_temp, last_rr, last_bat;

static void process_packet(void)
{
    int32_t ecg, bioz, ppg_ir; uint8_t bioz_valid, spo2, bat; uint16_t temp_raw, hr, rr;
    memcpy(&ecg, &pbuf[0], 4); memcpy(&bioz, &pbuf[4], 4); bioz_valid = pbuf[8];
    memcpy(&ppg_ir, &pbuf[9], 4);
    temp_raw = pbuf[17] | ((uint16_t)pbuf[18]<<8); spo2 = pbuf[19]; bat = pbuf[20];
    hr = pbuf[21] | ((uint16_t)pbuf[22]<<8); rr = pbuf[23] | ((uint16_t)pbuf[24]<<8);
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE) return;
    uint32_t now = millis();

    ecg_batch[ecg_bn++] = ecg;
    if (ecg_bn >= ECG_BATCH) { notify_bytes(h_ecg, ecg_batch, ECG_BATCH*4); ecg_bn = 0; }
    if (bioz_valid == 0x00) {
        bioz_batch[bioz_bn++] = bioz;
        if (bioz_bn >= BIOZ_BATCH) { notify_bytes(h_resp, bioz_batch, BIOZ_BATCH*4); bioz_bn = 0; }
    }
    if (++ppg_decim >= 2) { ppg_decim = 0; int16_t ppg = (int16_t)(ppg_ir>>8); notify_bytes(h_ppg, &ppg, 2); }

    if (hr > 0 && (now-last_hr >= 1000)) {
        uint8_t d[3];
        if (hr > 255){ d[0]=0x01; d[1]=hr&0xFF; d[2]=hr>>8; notify_bytes(h_hr,d,3); }
        else         { d[0]=0x00; d[1]=(uint8_t)hr;         notify_bytes(h_hr,d,2); }
        last_hr = now;
    }
    if (rr >= 4 && rr <= 40 && (now-last_rr >= 1000)) {
        uint8_t d[2] = {(uint8_t)(rr&0xFF),(uint8_t)(rr>>8)}; notify_bytes(h_rr,d,2); last_rr = now;
    }
    if (temp_raw > 0 && (now-last_temp >= 1000)) {
        uint8_t d[2] = {(uint8_t)(temp_raw&0xFF),(uint8_t)(temp_raw>>8)}; notify_bytes(h_temp,d,2); last_temp = now;
    }
    if (spo2 > 0 && (now-last_spo2 >= 1000)) {
        uint8_t d[3] = {0x00,spo2,0x00}; notify_bytes(h_spo2,d,3); last_spo2 = now;
    }
    if (bat > 0 && (now-last_bat >= 5000)) { notify_bytes(h_bat,&bat,1); last_bat = now; }
}

static void parse_byte(uint8_t b)
{
    switch (pstate) {
    case WAIT_S1: if (b==0x0A) pstate=WAIT_S2; break;
    case WAIT_S2: if (b==0xFA) pstate=WAIT_LEN_L; else if (b!=0x0A) pstate=WAIT_S1; break;
    case WAIT_LEN_L: plen=b; pstate=WAIT_LEN_H; break;
    case WAIT_LEN_H: plen|=((uint16_t)b<<8); pstate=(plen>=20&&plen<=25)?WAIT_TYPE:WAIT_S1; break;
    case WAIT_TYPE: if (b==0x02){ pidx=0; memset(pbuf,0,PKT_DATA_SIZE); pstate=READ_DATA; } else pstate=WAIT_S1; break;
    case READ_DATA: if (pidx<PKT_DATA_SIZE) pbuf[pidx++]=b; if (pidx>=plen) pstate=WAIT_ST1; break;
    case WAIT_ST1: pstate=(b==0x00)?WAIT_ST2:WAIT_S1; break;
    case WAIT_ST2: if (b==0x0B) process_packet(); pstate=WAIT_S1; break;
    }
}

static void uart_task(void *arg)
{
    uint8_t buf[256];
    while (1) {
        int n = uart_read_bytes(UART_PORT, buf, sizeof(buf), pdMS_TO_TICKS(20));
        for (int i=0;i<n;i++) parse_byte(buf[i]);
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) { nvs_flash_erase(); nvs_flash_init(); }

    uart_config_t uc = { .baud_rate=UART_BAUD, .data_bits=UART_DATA_8_BITS, .parity=UART_PARITY_DISABLE,
                         .stop_bits=UART_STOP_BITS_1, .flow_ctrl=UART_HW_FLOWCTRL_DISABLE, .source_clk=UART_SCLK_DEFAULT };
    uart_driver_install(UART_PORT, 4096, 256, 0, NULL, 0);
    uart_param_config(UART_PORT, &uc);
    uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    nimble_port_init();
    ble_hs_cfg.sync_cb = on_sync; ble_hs_cfg.reset_cb = on_reset;
    ble_svc_gap_init(); ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs); ble_gatts_add_svcs(gatt_svcs);
    ble_svc_gap_device_name_set(DEVICE_NAME);
    nimble_port_freertos_init(host_task);
    xTaskCreate(uart_task, "uart", 4096, NULL, 10, NULL);
    ESP_LOGI(TAG, "NirogScan DEV gateway started");
}
