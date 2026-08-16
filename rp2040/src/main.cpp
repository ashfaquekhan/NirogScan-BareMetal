/*
 * NirogScan DEV firmware - RP2040 (PlatformIO / pico-sdk)
 * ---------------------------------------------------------------------------
 * Runtime-configurable exploration build. Every MAX30001 (ECG + BioZ) and
 * AFE4400 (PPG) feature that makes sense to toggle is driven by short text
 * commands received from the ESP32 over the UART back-channel (GP25 RX). The
 * ESP32 gets those commands from a writable BLE characteristic, so the web UI
 * controls the sensors live.
 *
 * Data path (25-byte packet over UART1 TX GP24) is unchanged, so the same web
 * UI plots ECG/BioZ/PPG and vitals. Disabled channels stream zeros. When ECG
 * is disabled the packet cadence falls back to a ~64 Hz timer so BioZ-only /
 * PPG-only modes still stream.
 *
 * Commands (newline-terminated), sent ESP32 -> RP2040:
 *   E <0|1>            ECG channel enable
 *   B <0|1>            BioZ channel enable
 *   R <0|1>            R-to-R (HR) engine
 *   D <0|1>            ECG DC lead-off
 *   X <0|1>            RBIAS input bias resistors (ECG baseline)
 *   P <0|1>            Pace (pacemaker edge) detect
 *   ER <128|256|512>   ECG sample rate
 *   EG <20|40|80|160>  ECG gain (V/V)
 *   EH <0|1>           ECG 0.5 Hz digital HPF
 *   EL <0|40|100|150>  ECG digital LPF (0 = bypass)
 *   BR <32|64>         BioZ sample rate
 *   BG <10|20|40|80>   BioZ gain (V/V)
 *   BC <0..7>          BioZ current-generator magnitude
 *   BH <0|1>           BioZ respiration tuning (1 = AHPF bypass + 0.05 Hz DHPF)
 *   L  <0..255>        AFE LED drive current (both LEDs)
 *   ?                  re-apply full config
 */

#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

// ---- Pins ------------------------------------------------------------------
#define PIN_SCK 2
#define PIN_MOSI 3
#define PIN_MISO 4
#define PIN_ECG_CS 5
#define PIN_PPG_CS 19
#define PIN_PPG_PWDN 18
#define PIN_I2C_SDA 6
#define PIN_I2C_SCL 7
#define PIN_UART_TX 24         // -> ESP32 RX
#define PIN_UART_RX 25         // <- ESP32 TX (commands)
#define PIN_LED_BLUE 22
#define PIN_LED_GREEN 21

#define SPI_PORT spi0
#define I2C_PORT i2c1
#define UART_ESP uart1
#define SPI_HZ 1000000u
#define I2C_HZ 400000u
#define UART_BAUD 230400u

static inline uint32_t now_ms(void){ return to_ms_since_boot(get_absolute_time()); }

// ---- MAX30001 registers ----------------------------------------------------
#define MX_RREG 0x01
#define MX_STATUS 0x01
#define MX_EN_INT 0x02
#define MX_MNGR_INT 0x04
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
#define AFE_LEDCNTRL 0x22
#define AFE_LED2VAL 0x2A       // RED
#define AFE_LED1VAL 0x2C       // IR

// ---- I2C addresses ---------------------------------------------------------
#define MAX30205_A1 0x48
#define MAX30205_A2 0x49
#define MAX17048_ADDR 0x36

// ===========================================================================
//  Runtime configuration (defaults = original working config)
// ===========================================================================
struct Config {
    uint8_t  ecg_en, bioz_en, rtor_en, dcloff_en, rbias_en, pace_en;
    uint16_t ecg_rate;   // 128/256/512
    uint8_t  ecg_gain;   // 20/40/80/160
    uint8_t  ecg_hpf;    // 0/1 (0.5 Hz)
    uint16_t ecg_lpf;    // 0(bypass)/40/100/150
    uint8_t  bioz_rate;  // 32/64
    uint8_t  bioz_gain;  // 10/20/40/80
    uint8_t  bioz_cgmag; // 0..7
    uint8_t  bioz_resp;  // 1 = AHPF bypass + 0.05 Hz DHPF
    uint8_t  afe_led;    // 0..255
};
static Config cfg = {
    /*ecg_en*/1, /*bioz_en*/1, /*rtor_en*/1, /*dcloff_en*/0, /*rbias_en*/0, /*pace_en*/0,
    /*ecg_rate*/128, /*ecg_gain*/80, /*ecg_hpf*/1, /*ecg_lpf*/40,
    /*bioz_rate*/64, /*bioz_gain*/20, /*bioz_cgmag*/2, /*bioz_resp*/0, /*afe_led*/0x14
};

// ===========================================================================
//  MAX30001 register unions (subset needed to compose configs)
// ===========================================================================
typedef union { uint32_t all; struct {
    uint32_t rbiasn:1; uint32_t rbiasp:1; uint32_t rbiasv:2; uint32_t en_rbias:2;
    uint32_t vth:2; uint32_t imag:3; uint32_t ipol:1; uint32_t en_dcloff:2;
    uint32_t en_bloff:2; uint32_t res1:1; uint32_t en_pace:1; uint32_t en_bioz:1;
    uint32_t en_ecg:1; uint32_t fmstr:2; uint32_t en_ulp_lon:2; uint32_t res:8;
} bit; } gen_t;

typedef union { uint32_t all; struct {
    uint32_t res1:12; uint32_t dlpf:2; uint32_t dhpf:1; uint32_t res2:1;
    uint32_t gain:2; uint32_t res3:4; uint32_t rate:2; uint32_t res:8;
} bit; } ecg_t;

typedef union { uint32_t all; struct {
    uint32_t phoff:4; uint32_t cgmag:3; uint32_t cgmon:1; uint32_t fcgen:4;
    uint32_t dlpf:2; uint32_t dhpf:2; uint32_t gain:2; uint32_t ln_bioz:1;
    uint32_t ext_rbias:1; uint32_t ahpf:3; uint32_t rate:1; uint32_t res:8;
} bit; } bioz_t;

// ===========================================================================
//  MAX30001 SPI
// ===========================================================================
static inline void mx_cs(bool s){ gpio_put(PIN_ECG_CS, s?0:1); }
static void mx_write(uint8_t reg, uint32_t d){
    uint8_t tx[4]={(uint8_t)((reg<<1)|0),(uint8_t)(d>>16),(uint8_t)(d>>8),(uint8_t)d};
    mx_cs(true); spi_write_blocking(SPI_PORT,tx,4); mx_cs(false);
}
static uint32_t mx_read(uint8_t reg){
    uint8_t tx[4]={(uint8_t)((reg<<1)|MX_RREG),0,0,0}, rx[4]={0};
    mx_cs(true); spi_write_read_blocking(SPI_PORT,tx,rx,4); mx_cs(false);
    return ((uint32_t)rx[1]<<16)|((uint32_t)rx[2]<<8)|rx[3];
}
static void mx_burst(uint8_t reg, uint8_t*buf, int n){
    uint8_t cmd=(reg<<1)|MX_RREG;
    mx_cs(true); spi_write_blocking(SPI_PORT,&cmd,1); spi_read_blocking(SPI_PORT,0x00,buf,n); mx_cs(false);
}
static void mx_fifo_reset(void){ mx_write(MX_FIFO_RST,0); }
static void mx_synch(void){ mx_write(MX_SYNCH,0); }

// ---- compose registers from cfg -------------------------------------------
static uint32_t build_gen(void){
    gen_t r; r.all=0;
    r.bit.en_ecg  = cfg.ecg_en ? 1:0;
    r.bit.en_bioz = cfg.bioz_en ? 1:0;
    r.bit.en_pace = cfg.pace_en ? 1:0;
    r.bit.rbiasv  = 0b01;                     // 100 Mohm
    if (cfg.rbias_en){ r.bit.en_rbias=0b01; r.bit.rbiasp=1; r.bit.rbiasn=1; }
    if (cfg.dcloff_en){ r.bit.en_dcloff=0b01; r.bit.imag=0b001; }
    r.bit.fmstr=0;
    return r.all;
}
static uint32_t build_ecg(void){
    ecg_t r; r.all=0;
    r.bit.rate = (cfg.ecg_rate==512)?0b00 : (cfg.ecg_rate==256)?0b01 : 0b10;   // 128 default
    r.bit.gain = (cfg.ecg_gain==20)?0b00 : (cfg.ecg_gain==40)?0b01 : (cfg.ecg_gain==160)?0b11 : 0b10;
    r.bit.dhpf = cfg.ecg_hpf ? 1:0;
    r.bit.dlpf = (cfg.ecg_lpf==0)?0b00 : (cfg.ecg_lpf==100)?0b10 : (cfg.ecg_lpf==150)?0b11 : 0b01;
    return r.all;
}
static uint32_t build_bioz(void){
    bioz_t r; r.all=0;
    r.bit.phoff = 0b0001;   // matches original 0x259421 default
    r.bit.cgmag = cfg.bioz_cgmag & 0x07;
    r.bit.fcgen = 0b0100;
    r.bit.gain  = (cfg.bioz_gain==10)?0b00 : (cfg.bioz_gain==40)?0b10 : (cfg.bioz_gain==80)?0b11 : 0b01; // 20 default
    r.bit.dlpf  = 0b01;
    if (cfg.bioz_resp){ r.bit.ahpf=0b000; r.bit.dhpf=0b01; r.bit.ln_bioz=0; }   // resp-tuned
    else             { r.bit.ahpf=0b010; r.bit.dhpf=0b10; r.bit.ln_bioz=1; }    // original
    r.bit.rate = (cfg.bioz_rate==32)?1:0;
    return r.all;
}

static bool mx_apply(void){
    mx_write(MX_SW_RST,0); sleep_ms(100);
    uint32_t info = mx_read(MX_INFO);
    bool found = (((info>>16)&0xF0)==0x50);
    mx_write(MX_CNFG_GEN,     build_gen());  sleep_ms(2);
    mx_write(MX_CNFG_CAL,     0x720000);     sleep_ms(2);
    mx_write(MX_CNFG_ECG,     build_ecg());  sleep_ms(2);
    mx_write(MX_CNFG_EMUX,    0x000000);     sleep_ms(2);
    mx_write(MX_CNFG_BIOZ,    build_bioz()); sleep_ms(2);
    mx_write(MX_CNFG_BIOZ_LC, 0x800000);     sleep_ms(2);
    mx_write(MX_CNFG_BMUX,    0x000040);     sleep_ms(2);
    if (cfg.rtor_en){
        mx_write(MX_CNFG_RTOR1, 0x3FC600); sleep_ms(2);   // en_rtor=1
        mx_write(MX_CNFG_RTOR2, 0x202400); sleep_ms(2);
        mx_write(MX_EN_INT,     0x000400); sleep_ms(2);
    } else {
        mx_write(MX_CNFG_RTOR1, 0x3F4600); sleep_ms(2);   // en_rtor=0
    }
    mx_write(MX_MNGR_INT, 0x000000); sleep_ms(2);
    mx_synch(); sleep_ms(80);
    return found;
}

static int mx_read_fifo(uint8_t reg, int32_t*out, int maxn, bool is_ecg){
    uint8_t buf[8*3]; if(maxn>8)maxn=8;
    mx_burst(reg, buf, maxn*3);
    int n=0;
    for(int i=0;i<maxn;i++){
        const uint8_t*p=&buf[i*3];
        uint8_t tag = is_ecg ? ((p[2]&0x38)>>3) : (p[2]&0x07);
        bool valid = is_ecg ? (tag==0||tag==1||tag==2) : (tag==0||tag==2);
        if(valid){
            if(is_ecg){ uint32_t raw=((uint32_t)p[0]<<16)|((uint32_t)p[1]<<8)|(uint32_t)(p[2]&0xC0); out[n++]=((int32_t)(raw<<8))>>14; }
            else       { uint32_t raw=((uint32_t)p[0]<<16)|((uint32_t)p[1]<<8)|(uint32_t)(p[2]&0xF0); out[n++]=((int32_t)(raw<<8))>>12; }
        } else if(tag==0x07){ mx_fifo_reset(); mx_synch(); break; }
        else break;
    }
    return n;
}

// ===========================================================================
//  AFE4400
// ===========================================================================
static inline void afe_cs(bool s){ gpio_put(PIN_PPG_CS, s?0:1); }
static void afe_write(uint8_t reg, uint32_t d){
    uint8_t tx[4]={reg,(uint8_t)(d>>16),(uint8_t)(d>>8),(uint8_t)d};
    afe_cs(true); spi_write_blocking(SPI_PORT,tx,4); afe_cs(false);
}
static void afe_read_enable(void){ afe_write(AFE_CONTROL0,0x000001); }
static uint32_t afe_read(uint8_t reg){
    uint8_t tx[4]={reg,0,0,0}, rx[4]={0};
    afe_cs(true); spi_write_read_blocking(SPI_PORT,tx,rx,4); afe_cs(false);
    return ((uint32_t)rx[1]<<16)|((uint32_t)rx[2]<<8)|rx[3];
}
static inline int32_t afe_sign22(uint32_t raw){ return ((int32_t)(raw<<10))>>10; }

static void afe_set_led(uint8_t v){ afe_write(AFE_CONTROL0,0x000000); afe_write(AFE_LEDCNTRL, ((uint32_t)v<<8)|v); afe_read_enable(); }

static void afe_init(void){
    gpio_init(PIN_PPG_PWDN); gpio_set_dir(PIN_PPG_PWDN,GPIO_OUT);
    gpio_put(PIN_PPG_PWDN,0); sleep_ms(100);
    gpio_put(PIN_PPG_PWDN,1); sleep_ms(100);
    afe_write(0x00,0x000000); afe_write(0x00,0x000008);
    afe_write(0x20,0x000000); afe_write(0x21,0x000001);
    afe_write(AFE_LEDCNTRL, ((uint32_t)cfg.afe_led<<8)|cfg.afe_led);
    afe_write(0x23,0x000000); afe_write(0x1E,0x010707); afe_write(0x1D,0x001F3F);
    afe_write(0x01,0x001770); afe_write(0x02,0x001F3E); afe_write(0x03,0x001770); afe_write(0x04,0x001F3F);
    afe_write(0x05,0x000000); afe_write(0x06,0x0007CE); afe_write(0x0D,0x000002); afe_write(0x0E,0x0007CF);
    afe_write(0x0F,0x0007D2); afe_write(0x10,0x000F9F); afe_write(0x07,0x0007D0); afe_write(0x08,0x000F9E);
    afe_write(0x09,0x0007D0); afe_write(0x0A,0x000F9F); afe_write(0x0B,0x000FA0); afe_write(0x0C,0x00176E);
    afe_write(0x11,0x000FA2); afe_write(0x12,0x00176F); afe_write(0x13,0x001772); afe_write(0x14,0x001F3F);
    afe_write(0x15,0x000000); afe_write(0x16,0x000000); afe_write(0x17,0x0007D0); afe_write(0x18,0x0007D0);
    afe_write(0x19,0x000FA0); afe_write(0x1A,0x000FA0); afe_write(0x1B,0x001770); afe_write(0x1C,0x001770);
    sleep_ms(100);
    afe_read_enable();
}

// ===========================================================================
//  I2C (temp + fuel gauge)
// ===========================================================================
static uint8_t temp_addr=0; static bool fuel_ok=false;
static bool i2c_present(uint8_t a){ uint8_t d; return i2c_read_blocking(I2C_PORT,a,&d,1,false)>=0; }
static bool i2c_rd(uint8_t a,uint8_t r,uint8_t*b,uint8_t n){
    if(i2c_write_blocking(I2C_PORT,a,&r,1,true)<0) return false;
    return i2c_read_blocking(I2C_PORT,a,b,n,false)==(int)n;
}
static float temp_read_c(void){ if(!temp_addr)return 0; uint8_t b[2]; if(!i2c_rd(temp_addr,0x00,b,2))return 0;
    int16_t raw=(int16_t)(((uint16_t)b[0]<<8)|b[1]); return raw*0.00390625f; }
static uint8_t batt_read_soc(void){ uint8_t b[2]; if(!i2c_rd(MAX17048_ADDR,0x04,b,2))return 0; return b[0]; }

// ===========================================================================
//  Packet (25 bytes) - unchanged layout
// ===========================================================================
static uint8_t DataPacket[25];
static const uint8_t HDR[5]={0x0A,0xFA,25,0x00,0x02};
static const uint8_t FTR[2]={0x00,0x0B};
static int32_t g_ppg_ir=0,g_ppg_red=0,g_bioz=0;
static uint16_t g_temp_raw=0,g_hr=0; static uint8_t g_batt=0,g_spo2=0,g_resp_rate=0;
static uint32_t g_count=0;

static inline void put_i32(uint8_t*p,int32_t v){ p[0]=v;p[1]=v>>8;p[2]=v>>16;p[3]=v>>24; }
static void emit_sample(int32_t ecg,int32_t bioz_pkt,bool fresh){
    put_i32(&DataPacket[0],ecg);
    put_i32(&DataPacket[4],bioz_pkt);
    DataPacket[8]=fresh?0x00:0xFF;
    put_i32(&DataPacket[9],g_ppg_ir);
    put_i32(&DataPacket[13],g_ppg_red);
    DataPacket[17]=g_temp_raw&0xFF; DataPacket[18]=g_temp_raw>>8;
    DataPacket[19]=g_spo2; DataPacket[20]=g_batt;
    DataPacket[21]=g_hr&0xFF; DataPacket[22]=g_hr>>8;
    DataPacket[23]=g_resp_rate&0xFF; DataPacket[24]=g_resp_rate>>8;
    uint8_t f[5+25+2]; memcpy(f,HDR,5); memcpy(f+5,DataPacket,25); memcpy(f+30,FTR,2);
    uart_write_blocking(UART_ESP,f,sizeof(f));
}

// ===========================================================================
//  Respiration (Zephyr-ported) - runs whenever BioZ enabled
// ===========================================================================
#define RESP_CAL 2980
static void resp_detect(int16_t wave){
    static int32_t baseline=0; static uint16_t sinceX=0, ivals[8]={0}; static uint8_t iidx=0,nv=0;
    static uint16_t startup=312; static bool waitLow=false; static int16_t amin=32767,amax=-32768; static uint16_t at=0;
    if(startup>0){ startup--; baseline=(int32_t)wave<<16; return; }
    baseline = baseline-(baseline>>8)+((int32_t)wave<<8);
    int16_t base=(int16_t)(baseline>>16);
    if(wave<amin)amin=wave; if(wave>amax)amax=wave;
    if(++at>=125){ if((amax-amin)<50){ g_resp_rate=0; nv=0; } amin=32767; amax=-32768; at=0; }
    sinceX++; int16_t hi=base+20, lo=base-20;
    if(!waitLow){ if(wave<lo)waitLow=true; }
    else if(wave>hi){ waitLow=false;
        if(sinceX>=30&&sinceX<=400){ ivals[iidx]=sinceX; if(++iidx>=8)iidx=0; if(nv<8)nv++;
            if(nv>=3){ uint32_t s=0; for(uint8_t i=0;i<nv;i++)s+=ivals[i]; uint16_t a=s/nv;
                if(a>0){ uint16_t rt=RESP_CAL/a; if(rt>=8&&rt<=80) g_resp_rate=(uint8_t)rt; } } }
        else if(sinceX>400) nv=0;
        sinceX=0;
    }
    if(sinceX>312){ g_resp_rate=0; nv=0; waitLow=false; sinceX=0; }
}
static void resp_feed(int32_t bioz){
    static int16_t buf[4]; static uint8_t c=0;
    buf[c++]=(int16_t)bioz; if(c>=4){ long m=(long)buf[0]+buf[1]+buf[2]+buf[3]; resp_detect((int16_t)(m>>2)); c=0; }
}

// ===========================================================================
//  Command parser (from ESP32 over UART RX)
// ===========================================================================
static void apply_config(void){ mx_apply(); afe_set_led(cfg.afe_led); }

static void handle_command(char*s){
    char key[8]={0}; int val=0;
    if(sscanf(s,"%7s %d",key,&val)<1) return;
    bool max_change=true, afe_change=false;
    if      (!strcmp(key,"E"))  cfg.ecg_en=val?1:0;
    else if (!strcmp(key,"B"))  cfg.bioz_en=val?1:0;
    else if (!strcmp(key,"R"))  cfg.rtor_en=val?1:0;
    else if (!strcmp(key,"D"))  cfg.dcloff_en=val?1:0;
    else if (!strcmp(key,"X"))  cfg.rbias_en=val?1:0;
    else if (!strcmp(key,"P"))  cfg.pace_en=val?1:0;
    else if (!strcmp(key,"ER")) cfg.ecg_rate=val;
    else if (!strcmp(key,"EG")) cfg.ecg_gain=val;
    else if (!strcmp(key,"EH")) cfg.ecg_hpf=val?1:0;
    else if (!strcmp(key,"EL")) cfg.ecg_lpf=val;
    else if (!strcmp(key,"BR")) cfg.bioz_rate=val;
    else if (!strcmp(key,"BG")) cfg.bioz_gain=val;
    else if (!strcmp(key,"BC")) cfg.bioz_cgmag=val&7;
    else if (!strcmp(key,"BH")) cfg.bioz_resp=val?1:0;
    else if (!strcmp(key,"L")){ cfg.afe_led=(uint8_t)val; max_change=false; afe_change=true; }
    else if (!strcmp(key,"?")){ /* full re-apply */ }
    else return;
    if (afe_change) afe_set_led(cfg.afe_led);
    if (max_change) mx_apply();
}

static void poll_commands(void){
    static char cb[48]; static uint8_t cl=0;
    while(uart_is_readable(UART_ESP)){
        char ch=uart_getc(UART_ESP);
        if(ch=='\n'||ch=='\r'){ if(cl){ cb[cl]=0; handle_command(cb); cl=0; } }
        else if(cl<sizeof(cb)-1) cb[cl++]=ch;
    }
}

// ===========================================================================
//  SETUP / LOOP
// ===========================================================================
static uint32_t last_temp=0,last_batt=0,last_ecg_time=0,last_emit=0;

void setup(){
    gpio_init(PIN_LED_BLUE);  gpio_set_dir(PIN_LED_BLUE,GPIO_OUT);  gpio_put(PIN_LED_BLUE,1);
    gpio_init(PIN_LED_GREEN); gpio_set_dir(PIN_LED_GREEN,GPIO_OUT); gpio_put(PIN_LED_GREEN,1);

    spi_init(SPI_PORT,SPI_HZ);
    spi_set_format(SPI_PORT,8,SPI_CPOL_0,SPI_CPHA_0,SPI_MSB_FIRST);
    gpio_set_function(PIN_SCK,GPIO_FUNC_SPI); gpio_set_function(PIN_MOSI,GPIO_FUNC_SPI); gpio_set_function(PIN_MISO,GPIO_FUNC_SPI);
    gpio_init(PIN_ECG_CS); gpio_set_dir(PIN_ECG_CS,GPIO_OUT); gpio_put(PIN_ECG_CS,1);
    gpio_init(PIN_PPG_CS); gpio_set_dir(PIN_PPG_CS,GPIO_OUT); gpio_put(PIN_PPG_CS,1);

    i2c_init(I2C_PORT,I2C_HZ);
    gpio_set_function(PIN_I2C_SDA,GPIO_FUNC_I2C); gpio_set_function(PIN_I2C_SCL,GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C_SDA); gpio_pull_up(PIN_I2C_SCL);

    uart_init(UART_ESP,UART_BAUD);
    gpio_set_function(PIN_UART_TX,GPIO_FUNC_UART);
    gpio_set_function(PIN_UART_RX,GPIO_FUNC_UART);   // command back-channel
    uart_set_format(UART_ESP,8,1,UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ESP,true);

    sleep_ms(50);
    if(i2c_present(MAX30205_A1)) temp_addr=MAX30205_A1; else if(i2c_present(MAX30205_A2)) temp_addr=MAX30205_A2;
    fuel_ok=i2c_present(MAX17048_ADDR);

    afe_init();
    bool found = mx_apply();
    gpio_put(PIN_LED_GREEN, found?0:1);

    uint32_t t=now_ms(); last_temp=last_batt=last_ecg_time=last_emit=t;
}

void loop(){
    uint32_t now=now_ms();
    poll_commands();

    uint32_t status=mx_read(MX_STATUS);

    if(cfg.rtor_en && (status&MX_STAT_RRINT)){
        uint32_t rtor=mx_read(MX_RTOR); uint32_t cnt=rtor>>10;
        if(cnt>0){ float rri=cnt*7.8125f; uint16_t hr=(uint16_t)(60000.0f/rri); if(hr>=30&&hr<=220) g_hr=hr; }
    }

    // Drain BioZ FIFO whenever BioZ is enabled (keep latest)
    if(cfg.bioz_en && (status&(MX_STAT_EINT|MX_STAT_BINT))){
        int32_t bz[4]; int nb=mx_read_fifo(MX_BIOZ_FIFO_BURST,bz,4,false);
        if(nb>0){ g_bioz=bz[nb-1]; resp_feed(g_bioz); }
    }

    if(cfg.ecg_en && (status&MX_STAT_EINT)){
        // ECG-paced streaming (primary)
        int32_t ecg[8]; int ne=mx_read_fifo(MX_ECG_FIFO_BURST,ecg,8,true);
        if(ne>0) last_ecg_time=now;
        for(int i=0;i<ne;i++){
            if((g_count&1)==0){ g_ppg_ir=afe_sign22(afe_read(AFE_LED1VAL)); g_ppg_red=afe_sign22(afe_read(AFE_LED2VAL)); }
            bool fresh=((g_count&1)==0) && cfg.bioz_en;
            emit_sample(ecg[i], fresh?g_bioz:0, fresh);
            g_count++;
        }
    } else if(!cfg.ecg_en){
        // ECG off: timer-paced streaming (~64 Hz) for BioZ-only / PPG-only modes
        if(now-last_emit>=15){
            last_emit=now;
            g_ppg_ir=afe_sign22(afe_read(AFE_LED1VAL)); g_ppg_red=afe_sign22(afe_read(AFE_LED2VAL));
            bool fresh=cfg.bioz_en;
            emit_sample(0, fresh?g_bioz:0, fresh);
        }
    }

    if(status&(MX_STAT_EOVF|MX_STAT_BOVF)){ mx_fifo_reset(); mx_synch(); }
    if(cfg.ecg_en && (now-last_ecg_time)>1500){ mx_fifo_reset(); mx_synch(); last_ecg_time=now; }

    if(temp_addr && (now-last_temp>=1000)){ float c=temp_read_c(); if(c>=15&&c<=45) g_temp_raw=(uint16_t)(c*100.0f); last_temp=now; }
    if(fuel_ok && (now-last_batt>=5000)){ uint8_t p=batt_read_soc(); if(p>0&&p<=100) g_batt=p; last_batt=now; }

    if(!(status&(MX_STAT_EINT|MX_STAT_BINT|MX_STAT_RRINT))) sleep_us(150);
}
