#include <Arduino.h>            // entry-point glue only (main -> setup/loop)
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

// ---- Pins / peripherals ----------------------------------------------------
#define PIN_SCK 2
#define PIN_MOSI 3
#define PIN_MISO 4
#define PIN_ECG_CS 5
#define PIN_PPG_CS 19
#define PIN_PPG_PWDN 18
#define PIN_I2C_SDA 6
#define PIN_I2C_SCL 7
#define PIN_UART_TX 24         // -> ESP32-C3 RX (GP7 there)
#define PIN_LED_BLUE 22        // active-low
#define PIN_LED_GREEN 21       // active-low

#define SPI_PORT spi0
#define I2C_PORT i2c1
#define UART_ESP uart1
#define SPI_HZ 1000000u        // original MAX30001 library SPI speed; shared bus, MODE0
#define I2C_HZ 400000u
#define UART_BAUD 230400u

static inline uint32_t now_ms(void) { return to_ms_since_boot(get_absolute_time()); }

// ---- MAX30001 registers ----------------------------------------------------
#define MX_RREG 0x01
#define MX_STATUS 0x01
#define MX_EN_INT 0x02
#define MX_MNGR_INT 0x04
#define MX_MNGR_DYN 0x05
#define MX_SW_RST 0x08
#define MX_SYNCH 0x09
#define MX_FIFO_RST 0x0A
#define MX_INFO 0x0F
#define MX_CNFG_GEN 0x10
#define MX_CNFG_CAL 0x12
#define MX_CNFG_EMUX 0x14
#define MX_CNFG_ECG 0x15
#define MX_CNFG_BMUX 0x17
#define MX_CNFG_BIOZ 0x18
#define MX_CNFG_BIOZ_LC 0x1A
#define MX_CNFG_RTOR1 0x1D
#define MX_CNFG_RTOR2 0x1E
#define MX_ECG_FIFO_BURST 0x20
#define MX_BIOZ_FIFO_BURST 0x22
#define MX_RTOR 0x25
#define MX_STAT_EINT 0x800000u
#define MX_STAT_EOVF 0x400000u
#define MX_STAT_BINT 0x080000u
#define MX_STAT_BOVF 0x040000u
#define MX_STAT_RRINT 0x000400u

// ---- AFE registers ---------------------------------------------------------
#define AFE_CONTROL0 0x00
#define AFE_LED2VAL 0x2A       // RED
#define AFE_LED1VAL 0x2C       // IR

// ---- I2C addresses ---------------------------------------------------------
#define MAX30205_A1 0x48
#define MAX30205_A2 0x49
#define MAX17048_ADDR 0x36

// ===========================================================================
//  MAX30001 (SPI)
// ===========================================================================
static inline void mx_cs(bool s) { gpio_put(PIN_ECG_CS, s ? 0 : 1); }

static void mx_write(uint8_t reg, uint32_t d)
{
    uint8_t tx[4] = { (uint8_t)((reg << 1) | 0), (uint8_t)(d >> 16), (uint8_t)(d >> 8), (uint8_t)d };
    mx_cs(true); spi_write_blocking(SPI_PORT, tx, 4); mx_cs(false);
}
static uint32_t mx_read(uint8_t reg)
{
    uint8_t tx[4] = { (uint8_t)((reg << 1) | MX_RREG), 0, 0, 0 };
    uint8_t rx[4] = {0};
    mx_cs(true); spi_write_read_blocking(SPI_PORT, tx, rx, 4); mx_cs(false);
    return ((uint32_t)rx[1] << 16) | ((uint32_t)rx[2] << 8) | rx[3];
}
static void mx_burst(uint8_t reg, uint8_t *buf, int n)
{
    uint8_t cmd = (reg << 1) | MX_RREG;
    mx_cs(true);
    spi_write_blocking(SPI_PORT, &cmd, 1);
    spi_read_blocking(SPI_PORT, 0x00, buf, n);
    mx_cs(false);
}
static void mx_fifo_reset(void) { mx_write(MX_FIFO_RST, 0); }
static void mx_synch(void)      { mx_write(MX_SYNCH, 0); }

static bool mx_init(void)
{
    mx_write(MX_SW_RST, 0); sleep_ms(100);
    uint32_t info = mx_read(MX_INFO);
    bool found = (((info >> 16) & 0xF0) == 0x50);
    // Register values matched to the original ProtoCentral Arduino library
    // (MAX30001::BeginECGBioZ + the sketch's enable_rtor_detection()).
    mx_write(MX_CNFG_GEN,     0x0C0004); sleep_ms(2);  // ECG+BioZ, no RBIAS/dcloff
    mx_write(MX_CNFG_CAL,     0x720000); sleep_ms(2);  // calibration sources off
    mx_write(MX_CNFG_ECG,     0x825000); sleep_ms(2);  // 128 SPS, gain, 0.5Hz HPF, 40Hz LPF
    mx_write(MX_CNFG_EMUX,    0x000000); sleep_ms(2);
    mx_write(MX_CNFG_BIOZ,    0x259421); sleep_ms(2);  // original BioZ config
    mx_write(MX_CNFG_BIOZ_LC, 0x800000); sleep_ms(2);  // low-current mode off
    mx_write(MX_CNFG_BMUX,    0x000040); sleep_ms(2);
    mx_write(MX_CNFG_RTOR1,   0x3FC600); sleep_ms(2);  // R-to-R detection enable
    mx_write(MX_CNFG_RTOR2,   0x202400); sleep_ms(2);
    mx_write(MX_EN_INT,       0x000400); sleep_ms(2);  // R-to-R interrupt (RRINT)
    // MNGR_INT=0 kept: this firmware reads the FIFO status-driven (EINT/BINT per
    // sample). It is a FIFO-management register, not a signal setting, so the
    // MAX30001 signal config still matches the original. MNGR_DYN is left at its
    // reset default, exactly as the original library did (it never wrote it).
    mx_write(MX_MNGR_INT,     0x000000); sleep_ms(2);
    mx_synch(); sleep_ms(100);
    return found;
}

// Read up to max_samples (<=8), ETAG/BTAG validated. Returns valid count.
static int mx_read_fifo(uint8_t reg, int32_t *out, int max_samples, bool is_ecg)
{
    uint8_t buf[8 * 3];
    if (max_samples > 8) max_samples = 8;
    mx_burst(reg, buf, max_samples * 3);
    int n = 0;
    for (int i = 0; i < max_samples; i++) {
        const uint8_t *p = &buf[i * 3];
        uint8_t tag = is_ecg ? ((p[2] & 0x38) >> 3) : (p[2] & 0x07);
        bool valid = is_ecg ? (tag == 0x00 || tag == 0x01 || tag == 0x02)
                            : (tag == 0x00 || tag == 0x02);
        if (valid) {
            if (is_ecg) {
                uint32_t raw = ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | (uint32_t)(p[2] & 0xC0);
                out[n++] = ((int32_t)(raw << 8)) >> 14;    // 18-bit
            } else {
                uint32_t raw = ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | (uint32_t)(p[2] & 0xF0);
                out[n++] = ((int32_t)(raw << 8)) >> 12;    // 20-bit
            }
        } else if (tag == 0x07) {
            mx_fifo_reset(); mx_synch(); break;
        } else {
            break;
        }
    }
    return n;
}

// ===========================================================================
//  AFE4400 (SPI)
// ===========================================================================
static inline void afe_cs(bool s) { gpio_put(PIN_PPG_CS, s ? 0 : 1); }

static void afe_write(uint8_t reg, uint32_t d)
{
    uint8_t tx[4] = { reg, (uint8_t)(d >> 16), (uint8_t)(d >> 8), (uint8_t)d };
    afe_cs(true); spi_write_blocking(SPI_PORT, tx, 4); afe_cs(false);
}
static void afe_read_enable(void) { afe_write(AFE_CONTROL0, 0x000001); }  // SPI read mode
static uint32_t afe_read(uint8_t reg)
{
    // Read mode is enabled once in afe_init(), so no per-read CONTROL0 write.
    uint8_t tx[4] = { reg, 0, 0, 0 };
    uint8_t rx[4] = {0};
    afe_cs(true); spi_write_read_blocking(SPI_PORT, tx, rx, 4); afe_cs(false);
    return ((uint32_t)rx[1] << 16) | ((uint32_t)rx[2] << 8) | rx[3];
}
static inline int32_t afe_sign22(uint32_t raw) { return ((int32_t)(raw << 10)) >> 10; }

static void afe_init(void)
{
    // AFE power-down pin (PDN): pulse LOW (reset/power-down) then HIGH so the
    // chip ends up POWERED/ACTIVE. Ending LOW leaves the AFE asleep -> no PPG.
    // Matches the ProtoCentral library and the Zephyr driver exactly.
    gpio_init(PIN_PPG_PWDN); gpio_set_dir(PIN_PPG_PWDN, GPIO_OUT);
    gpio_put(PIN_PPG_PWDN, 0); sleep_ms(100);
    gpio_put(PIN_PPG_PWDN, 1); sleep_ms(100);

    afe_write(0x00, 0x000000); afe_write(0x00, 0x000008);
    afe_write(0x20, 0x000000); afe_write(0x21, 0x000001);
    afe_write(0x22, 0x001414); afe_write(0x23, 0x000000);
    afe_write(0x1E, 0x010707); afe_write(0x1D, 0x001F3F);
    afe_write(0x01, 0x001770); afe_write(0x02, 0x001F3E);
    afe_write(0x03, 0x001770); afe_write(0x04, 0x001F3F);
    afe_write(0x05, 0x000000); afe_write(0x06, 0x0007CE);
    afe_write(0x0D, 0x000002); afe_write(0x0E, 0x0007CF);
    afe_write(0x0F, 0x0007D2); afe_write(0x10, 0x000F9F);
    afe_write(0x07, 0x0007D0); afe_write(0x08, 0x000F9E);
    afe_write(0x09, 0x0007D0); afe_write(0x0A, 0x000F9F);
    afe_write(0x0B, 0x000FA0); afe_write(0x0C, 0x00176E);
    afe_write(0x11, 0x000FA2); afe_write(0x12, 0x00176F);
    afe_write(0x13, 0x001772); afe_write(0x14, 0x001F3F);
    afe_write(0x15, 0x000000); afe_write(0x16, 0x000000);
    afe_write(0x17, 0x0007D0); afe_write(0x18, 0x0007D0);
    afe_write(0x19, 0x000FA0); afe_write(0x1A, 0x000FA0);
    afe_write(0x1B, 0x001770); afe_write(0x1C, 0x001770);
    sleep_ms(100);
    afe_read_enable();   // switch to SPI read mode once; reads are write-free
}

// ===========================================================================
//  I2C (temp + fuel gauge)
// ===========================================================================
static uint8_t temp_addr = 0;
static bool    fuel_ok = false;

static bool i2c_present(uint8_t addr)
{
    uint8_t d;
    return i2c_read_blocking(I2C_PORT, addr, &d, 1, false) >= 0;
}
static bool i2c_rd(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n)
{
    if (i2c_write_blocking(I2C_PORT, addr, &reg, 1, true) < 0) return false;
    return i2c_read_blocking(I2C_PORT, addr, buf, n, false) == (int)n;
}
static float temp_read_c(void)
{
    if (!temp_addr) return 0.0f;
    uint8_t b[2];
    if (!i2c_rd(temp_addr, 0x00, b, 2)) return 0.0f;
    int16_t raw = (int16_t)(((uint16_t)b[0] << 8) | b[1]);
    return raw * 0.00390625f;
}
static uint8_t batt_read_soc(void)
{
    uint8_t b[2];
    if (!i2c_rd(MAX17048_ADDR, 0x04, b, 2)) return 0;
    return b[0];
}

// ===========================================================================
//  SpO2 algorithm (ported from Zephyr app/src/spo2_process.c)
// ===========================================================================
#define SPO2_FreqS 125
#define SPO2_BUF (SPO2_FreqS * 2)
static uint32_t spo2_ir[SPO2_BUF], spo2_red[SPO2_BUF];
static int32_t  spo2_norm[SPO2_BUF];
static uint16_t spo2_fill = 0;

static const uint8_t spo2_tab[184] = {
    95,95,95,96,96,96,97,97,97,97,97,98,98,98,98,98,99,99,99,99,
    99,99,99,99,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,
    100,100,100,100,99,99,99,99,99,99,99,99,98,98,98,98,98,98,97,97,
    97,97,96,96,96,96,95,95,95,94,94,94,93,93,93,92,92,92,91,91,
    90,90,89,89,89,88,88,87,87,86,86,85,85,84,84,83,82,82,81,81,
    80,80,79,78,78,77,76,76,75,74,74,73,72,72,71,70,69,69,68,67,
    66,66,65,64,63,62,62,61,60,59,58,57,56,56,55,54,53,52,51,50,
    49,48,47,46,45,44,43,42,41,40,39,38,37,36,35,34,33,31,30,29,
    28,27,26,25,23,22,21,20,19,17,16,15,14,12,11,10,9,7,6,5,3,2,1 };

typedef struct { bool off_state; uint8_t coff, con, toff, ton; } probe_t;
static probe_t probe = { true, 0, 0, 4, 3 };

static void probe_update(bool off_raw)
{
    if (off_raw) { probe.coff++; probe.con = 0; if (!probe.off_state && probe.coff >= probe.toff) probe.off_state = true; }
    else         { probe.con++; probe.coff = 0; if (probe.off_state && probe.con >= probe.ton) probe.off_state = false; }
}
static void sortu(int32_t *x, int32_t n)
{
    for (int32_t i = 1; i < n; i++) { int32_t t = x[i], j = i; while (j > 0 && t < x[j-1]) { x[j] = x[j-1]; j--; } x[j] = t; }
}
static int32_t meanu(const uint32_t *b, int32_t n) { int64_t s = 0; for (int32_t i = 0; i < n; i++) s += b[i]; return (int32_t)(s / n); }

static void detect_peaks(int32_t *locs, int32_t *npk, const int32_t *sig, int32_t len, int32_t mind, int32_t maxp)
{
    *npk = 0; if (len < 10) return;
    int64_t sum = 0; for (int32_t i = 0; i < len; i++) sum += sig[i];
    int32_t mean = (int32_t)(sum / len);
    int64_t var = 0; for (int32_t i = 0; i < len; i++) { int32_t d = sig[i] - mean; var += (int64_t)d * d; }
    int32_t sd = 1;
    if (var > 0) { int64_t v = var / len, x = v, y = (v + 1) / 2; while (y < x) { x = y; y = (x + v / x) / 2; } sd = (int32_t)x; }
    int32_t thr = mean + (sd * 3) / 10; if (thr < 20) thr = 20; if (thr > 100) thr = 100;
    int32_t last = -mind;
    for (int32_t i = 1; i < len - 1 && *npk < maxp; i++)
        if (sig[i] > thr && sig[i] > sig[i-1] && sig[i] >= sig[i+1] && i - last >= mind) { locs[(*npk)++] = i; last = i; }
}
static void ac_dc(const uint32_t *buf, const int32_t *locs, int32_t npk, int32_t *ac, int32_t *dc, int32_t *nv)
{
    *nv = 0; if (npk < 2) return;
    for (int32_t p = 0; p < npk - 1 && *nv < 10; p++) {
        int32_t s = locs[p], e = locs[p+1]; if (e - s < 10 || e - s > 200) continue;
        uint32_t mn = buf[s], mx = buf[s];
        for (int32_t i = s + 1; i < e; i++) { if (buf[i] < mn) mn = buf[i]; if (buf[i] > mx) mx = buf[i]; }
        int32_t a = (int32_t)((mx - mn) / 2), d = (int32_t)((mx + mn) / 2);
        if (a > 0 && d > 100) { ac[*nv] = a; dc[*nv] = d; (*nv)++; }
    }
}
static uint16_t perf_idx(int32_t ac, int32_t dc)
{
    if (dc == 0 || ac <= 0) return 0;
    int64_t v = ((int64_t)ac * 10000) / dc; if (v > 2000) v = 2000; if (v < 0) v = 0; return (uint16_t)v;
}
static uint8_t spo2_compute(const uint32_t *ir, const uint32_t *red, int32_t len, bool *off_out)
{
    *off_out = true; if (len < 100) return 0;
    int32_t mir = meanu(ir, len), mrd = meanu(red, len);
    if (mir < 1000 || mrd < 1000) return 0;
    if (mir > 3800000 || mrd > 3800000) return 0;
    for (int32_t i = 0; i < len; i++) spo2_norm[i] = -((int32_t)ir[i] - mir);
    int32_t locs[15], npk = 0;
    detect_peaks(locs, &npk, spo2_norm, len, 40, 15);
    if (npk < 2) return 0;
    int32_t ac_i[10], dc_i[10], ni = 0, ac_r[10], dc_r[10], nr = 0;
    ac_dc(ir, locs, npk, ac_i, dc_i, &ni);
    ac_dc(red, locs, npk, ac_r, dc_r, &nr);
    if (ni < 1 || nr < 1) return 0;
    int32_t nval = (ni < nr) ? ni : nr, rr[10], nrat = 0;
    for (int32_t i = 0; i < nval && nrat < 10; i++) {
        if (dc_r[i] == 0 || dc_i[i] == 0 || ac_i[i] == 0) continue;
        int64_t num = (int64_t)ac_r[i] * dc_i[i], den = (int64_t)ac_i[i] * dc_r[i];
        if (den > 0) { int32_t r = (int32_t)((num * 100) / den); if (r >= 50 && r <= 230) rr[nrat++] = r; }
    }
    if (nrat == 0) return 0;
    sortu(rr, nrat);
    int32_t idx = rr[nrat / 2] - 50;
    uint8_t s = (idx >= 0 && idx < 184) ? spo2_tab[idx] : 0;
    if (s == 0) return 0;
    sortu(ac_i, ni); sortu(dc_i, ni);
    int32_t mac = ac_i[ni / 2], mdc = dc_i[ni / 2];
    uint16_t pi = perf_idx(mac, mdc);
    bool off = false;
    if (pi < 30) off = true;
    else if (npk < 2) off = true;
    else { int thr = probe.off_state ? 110 : 70; if (mac < thr) off = true; }
    probe_update(off);
    *off_out = probe.off_state;
    return s;
}

// SpO2 output state
static uint8_t  g_spo2 = 0;
static int32_t  spo2_hist[8]; static uint8_t spo2_hi = 0, spo2_hc = 0; static int32_t spo2_filt = 0;

static void spo2_feed(int32_t ir_s, int32_t red_s)
{
    if (spo2_fill < SPO2_FreqS) {
        int32_t idx = SPO2_BUF - SPO2_FreqS + spo2_fill;
        spo2_ir[idx]  = (ir_s  < 0) ? 0 : (uint32_t)ir_s;
        spo2_red[idx] = (red_s < 0) ? 0 : (uint32_t)red_s;
        spo2_fill++;
        return;
    }
    spo2_fill = 0;
    bool off = true;
    uint8_t s = spo2_compute(spo2_ir, spo2_red, SPO2_BUF, &off);
    if (off) { spo2_hc = 0; spo2_hi = 0; spo2_filt = 0; g_spo2 = 0; }
    else if (s > 0 && s <= 100) {
        bool outlier = (spo2_hc > 0 && spo2_filt > 0 && s < spo2_filt && (spo2_filt - s) > 5);
        if (!outlier || spo2_hc == 0) {
            spo2_hist[spo2_hi] = s; spo2_hi = (spo2_hi + 1) % 8; if (spo2_hc < 8) spo2_hc++;
            int32_t sum = 0; for (uint8_t i = 0; i < spo2_hc; i++) sum += spo2_hist[i];
            spo2_filt = sum / spo2_hc;
        }
        g_spo2 = (uint8_t)spo2_filt;
    }
    for (int32_t i = SPO2_FreqS; i < SPO2_BUF; i++) { spo2_ir[i - SPO2_FreqS] = spo2_ir[i]; spo2_red[i - SPO2_FreqS] = spo2_red[i]; }
}

// ===========================================================================
//  Respiration (ported from Zephyr app/src/resp_process.c)
// ===========================================================================
#define RESP_CAL 2980
static uint8_t g_resp_rate = 0;

static void resp_detect(int16_t wave)
{
    static int32_t baseline = 0;
    static uint16_t sinceX = 0, ivals[8] = {0};
    static uint8_t iidx = 0, nvalid = 0;
    static uint16_t startup = 312;
    static bool waitLow = false;
    static int16_t amin = 32767, amax = -32768;
    static uint16_t atimer = 0;

    if (startup > 0) { startup--; baseline = (int32_t)wave << 16; return; }
    baseline = baseline - (baseline >> 8) + ((int32_t)wave << 8);
    int16_t base = (int16_t)(baseline >> 16);
    if (wave < amin) amin = wave;
    if (wave > amax) amax = wave;
    if (++atimer >= 125) { if ((amax - amin) < 50) { g_resp_rate = 0; nvalid = 0; } amin = 32767; amax = -32768; atimer = 0; }
    sinceX++;
    int16_t hi = base + 20, lo = base - 20;
    if (!waitLow) { if (wave < lo) waitLow = true; }
    else {
        if (wave > hi) {
            waitLow = false;
            if (sinceX >= 30 && sinceX <= 400) {
                ivals[iidx] = sinceX; if (++iidx >= 8) iidx = 0; if (nvalid < 8) nvalid++;
                if (nvalid >= 3) {
                    uint32_t sum = 0; for (uint8_t i = 0; i < nvalid; i++) sum += ivals[i];
                    uint16_t avg = sum / nvalid;
                    if (avg > 0) { uint16_t rate = RESP_CAL / avg; if (rate >= 8 && rate <= 80) g_resp_rate = (uint8_t)rate; }
                }
            } else if (sinceX > 400) nvalid = 0;
            sinceX = 0;
        }
    }
    if (sinceX > 312) { g_resp_rate = 0; nvalid = 0; waitLow = false; sinceX = 0; }
}
static void resp_feed(int32_t bioz)
{
    static int16_t buf[4]; static uint8_t c = 0;
    buf[c++] = (int16_t)bioz;
    if (c >= 4) { long m = (long)buf[0] + buf[1] + buf[2] + buf[3]; resp_detect((int16_t)(m >> 2)); c = 0; }
}

// ===========================================================================
//  Packet (25 bytes) over UART1 to the ESP32-C3
// ===========================================================================
static uint8_t DataPacket[25];
static const uint8_t HDR[5] = { 0x0A, 0xFA, 25, 0x00, 0x02 };
static const uint8_t FTR[2] = { 0x00, 0x0B };

static int32_t  g_ppg_ir = 0, g_ppg_red = 0;
static uint16_t g_temp_raw = 0, g_hr = 0;
static uint8_t  g_batt = 0;
static uint32_t g_ecg_count = 0;

static inline void put_i32(uint8_t *p, int32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static void emit_sample(int32_t ecg, int32_t bioz_pkt, bool bioz_fresh)
{
    put_i32(&DataPacket[0], ecg);
    put_i32(&DataPacket[4], bioz_pkt);
    DataPacket[8] = bioz_fresh ? 0x00 : 0xFF;
    put_i32(&DataPacket[9], g_ppg_ir);
    put_i32(&DataPacket[13], g_ppg_red);
    DataPacket[17] = (uint8_t)(g_temp_raw & 0xFF);
    DataPacket[18] = (uint8_t)(g_temp_raw >> 8);
    DataPacket[19] = g_spo2;
    DataPacket[20] = g_batt;
    DataPacket[21] = (uint8_t)(g_hr & 0xFF);
    DataPacket[22] = (uint8_t)(g_hr >> 8);
    DataPacket[23] = (uint8_t)(g_resp_rate & 0xFF);
    DataPacket[24] = (uint8_t)(g_resp_rate >> 8);

    uint8_t frame[5 + 25 + 2];
    memcpy(frame, HDR, 5);
    memcpy(frame + 5, DataPacket, 25);
    memcpy(frame + 30, FTR, 2);
    uart_write_blocking(UART_ESP, frame, sizeof(frame));
}

// ===========================================================================
//  SETUP / LOOP
// ===========================================================================
static bool mx_found = false;
static int32_t g_bioz = 0;
static uint32_t last_temp = 0, last_batt = 0, last_ecg_time = 0;

void setup()
{
    gpio_init(PIN_LED_BLUE);  gpio_set_dir(PIN_LED_BLUE, GPIO_OUT);  gpio_put(PIN_LED_BLUE, 1);
    gpio_init(PIN_LED_GREEN); gpio_set_dir(PIN_LED_GREEN, GPIO_OUT); gpio_put(PIN_LED_GREEN, 1);

    spi_init(SPI_PORT, SPI_HZ);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_init(PIN_ECG_CS); gpio_set_dir(PIN_ECG_CS, GPIO_OUT); gpio_put(PIN_ECG_CS, 1);
    gpio_init(PIN_PPG_CS); gpio_set_dir(PIN_PPG_CS, GPIO_OUT); gpio_put(PIN_PPG_CS, 1);

    i2c_init(I2C_PORT, I2C_HZ);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA); gpio_pull_up(PIN_I2C_SCL);

    uart_init(UART_ESP, UART_BAUD);
    gpio_set_function(PIN_UART_TX, GPIO_FUNC_UART);
    uart_set_format(UART_ESP, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ESP, true);

    sleep_ms(50);

    if      (i2c_present(MAX30205_A1)) temp_addr = MAX30205_A1;
    else if (i2c_present(MAX30205_A2)) temp_addr = MAX30205_A2;
    fuel_ok = i2c_present(MAX17048_ADDR);

    afe_init();
    mx_found = mx_init();

    gpio_put(PIN_LED_GREEN, mx_found ? 0 : 1);

    uint32_t t = now_ms();
    last_temp = last_batt = last_ecg_time = t;
}

void loop()
{
    uint32_t now = now_ms();
    uint32_t status = mx_read(MX_STATUS);

    // Heart rate from R-to-R
    if (status & MX_STAT_RRINT) {
        uint32_t rtor = mx_read(MX_RTOR);
        uint32_t cnt = rtor >> 10;
        if (cnt > 0) {
            float rri = cnt * 7.8125f;
            uint16_t hr = (uint16_t)(60000.0f / rri);
            if (hr >= 30 && hr <= 220) g_hr = hr;
        }
    }

    // ECG + BioZ
    if (status & MX_STAT_EINT) {
        int32_t ecg[8], bioz[4];
        int ne = mx_read_fifo(MX_ECG_FIFO_BURST, ecg, 8, true);
        int nb = mx_read_fifo(MX_BIOZ_FIFO_BURST, bioz, 4, false);
        if (ne > 0) last_ecg_time = now;

        for (int i = 0; i < ne; i++) {
            if (nb > 0) { int bi = (i * nb) / ne; if (bi >= nb) bi = nb - 1; g_bioz = bioz[bi]; }

            // PPG at ~64 Hz (every 2nd ECG sample); held value fills the packet.
            if ((g_ecg_count & 1) == 0) {
                g_ppg_ir  = afe_sign22(afe_read(AFE_LED1VAL));
                g_ppg_red = afe_sign22(afe_read(AFE_LED2VAL));
            }

            spo2_feed(g_ppg_ir, g_ppg_red);
            resp_feed(g_bioz);

            bool fresh = ((g_ecg_count & 1) == 0);
            emit_sample(ecg[i], fresh ? g_bioz : 0, fresh);
            g_ecg_count++;
        }
    } else if (status & MX_STAT_BINT) {
        int32_t bioz[4];
        int nb = mx_read_fifo(MX_BIOZ_FIFO_BURST, bioz, 4, false);
        if (nb > 0) g_bioz = bioz[nb - 1];
    }

    if (status & (MX_STAT_EOVF | MX_STAT_BOVF)) { mx_fifo_reset(); mx_synch(); }
    if ((now - last_ecg_time) > 1500) { mx_fifo_reset(); mx_synch(); last_ecg_time = now; }

    if (temp_addr && (now - last_temp >= 1000)) {
        float c = temp_read_c();
        if (c >= 15.0f && c <= 45.0f) g_temp_raw = (uint16_t)(c * 100.0f);
        last_temp = now;
    }
    if (fuel_ok && (now - last_batt >= 5000)) {
        uint8_t p = batt_read_soc();
        if (p > 0 && p <= 100) g_batt = p;
        last_batt = now;
    }

    if (!(status & (MX_STAT_EINT | MX_STAT_BINT | MX_STAT_RRINT))) sleep_us(200);
}
