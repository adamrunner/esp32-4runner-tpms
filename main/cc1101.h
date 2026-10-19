#pragma once

#include <stdint.h>
#include "esp_err.h"

typedef struct {
    int pin_sck;
    int pin_mosi;
    int pin_miso;
    int pin_cs;
} cc1101_pins_t;

// Configuration registers
#define CC1101_IOCFG2   0x00
#define CC1101_IOCFG1   0x01
#define CC1101_IOCFG0   0x02
#define CC1101_FIFOTHR  0x03
#define CC1101_PKTCTRL1 0x07
#define CC1101_PKTCTRL0 0x08
#define CC1101_FSCTRL1  0x0B
#define CC1101_FSCTRL0  0x0C
#define CC1101_FREQ2    0x0D
#define CC1101_FREQ1    0x0E
#define CC1101_FREQ0    0x0F
#define CC1101_MDMCFG4  0x10
#define CC1101_MDMCFG3  0x11
#define CC1101_MDMCFG2  0x12
#define CC1101_MDMCFG1  0x13
#define CC1101_MDMCFG0  0x14
#define CC1101_DEVIATN  0x15
#define CC1101_MCSM1    0x17
#define CC1101_MCSM0    0x18
#define CC1101_FOCCFG   0x19
#define CC1101_BSCFG    0x1A
#define CC1101_AGCCTRL2 0x1B
#define CC1101_AGCCTRL1 0x1C
#define CC1101_AGCCTRL0 0x1D
#define CC1101_FREND1   0x21
#define CC1101_FREND0   0x22
#define CC1101_FSCAL3   0x23
#define CC1101_FSCAL2   0x24
#define CC1101_FSCAL1   0x25
#define CC1101_FSCAL0   0x26
#define CC1101_TEST2    0x2C
#define CC1101_TEST1    0x2D
#define CC1101_TEST0    0x2E

// Status registers (read with burst bit set)
#define CC1101_PARTNUM   0x30
#define CC1101_VERSION   0x31
#define CC1101_FREQEST   0x32
#define CC1101_RSSI      0x34
#define CC1101_MARCSTATE 0x35

// Command strobes
#define CC1101_SRES  0x30
#define CC1101_SCAL  0x33
#define CC1101_SRX   0x34
#define CC1101_SIDLE 0x36
#define CC1101_SFRX  0x3A

esp_err_t cc1101_init(const cc1101_pins_t *pins);
esp_err_t cc1101_reset(void);
uint8_t cc1101_read_reg(uint8_t addr);
uint8_t cc1101_read_status(uint8_t addr);
void cc1101_write_reg(uint8_t addr, uint8_t value);
void cc1101_strobe(uint8_t cmd);

// Configure for async (transparent) 2-FSK RX: demodulated data on GDO0,
// carrier sense on GDO2.
void cc1101_config_async_fsk_rx(uint32_t freq_hz);
void cc1101_set_freq(uint32_t freq_hz);
// Carrier-sense absolute threshold relative to MAGN_TARGET, -7..+7 dB.
void cc1101_set_cs_threshold(int db);
void cc1101_start_rx(void);
float cc1101_rssi_dbm(void);
