#include "cc1101.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define CC1101_XTAL_HZ 26000000ULL

#define READ_SINGLE 0x80
#define READ_BURST  0xC0

static const char *TAG = "cc1101";

static spi_device_handle_t s_spi;
static SemaphoreHandle_t s_lock;
static cc1101_pins_t s_pins;

// CS is driven by hand so we can wait for CHIP_RDYn (MISO low) after asserting it.
static bool select_chip(void)
{
    gpio_set_level(s_pins.pin_cs, 0);
    for (int i = 0; i < 1000; i++) {
        if (gpio_get_level(s_pins.pin_miso) == 0) {
            return true;
        }
        esp_rom_delay_us(1);
    }
    return false;
}

static void deselect_chip(void)
{
    gpio_set_level(s_pins.pin_cs, 1);
}

static uint8_t xfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (!select_chip()) {
        ESP_LOGW(TAG, "chip not ready (MISO stuck high)");
    }
    spi_device_polling_transmit(s_spi, &t);
    deselect_chip();
    xSemaphoreGive(s_lock);
    return rx[0];
}

uint8_t cc1101_read_reg(uint8_t addr)
{
    uint8_t tx[2] = {addr | READ_SINGLE, 0}, rx[2] = {0};
    xfer(tx, rx, 2);
    return rx[1];
}

uint8_t cc1101_read_status(uint8_t addr)
{
    uint8_t tx[2] = {addr | READ_BURST, 0}, rx[2] = {0};
    xfer(tx, rx, 2);
    return rx[1];
}

void cc1101_write_reg(uint8_t addr, uint8_t value)
{
    uint8_t tx[2] = {addr, value}, rx[2];
    xfer(tx, rx, 2);
}

void cc1101_strobe(uint8_t cmd)
{
    uint8_t tx[1] = {cmd}, rx[1];
    xfer(tx, rx, 1);
}

esp_err_t cc1101_init(const cc1101_pins_t *pins)
{
    s_pins = *pins;
    s_lock = xSemaphoreCreateMutex();

    gpio_config_t cs = {
        .pin_bit_mask = 1ULL << pins->pin_cs,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&cs));
    deselect_chip();

    spi_bus_config_t bus = {
        .sclk_io_num = pins->pin_sck,
        .mosi_io_num = pins->pin_mosi,
        .miso_io_num = pins->pin_miso,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED));

    spi_device_interface_config_t dev = {
        .clock_speed_hz = 4 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev, &s_spi));

    return cc1101_reset();
}

esp_err_t cc1101_reset(void)
{
    // Manual power-on reset sequence from the datasheet (section 19.1.2).
    deselect_chip();
    esp_rom_delay_us(5);
    gpio_set_level(s_pins.pin_cs, 0);
    esp_rom_delay_us(10);
    deselect_chip();
    esp_rom_delay_us(45);
    cc1101_strobe(CC1101_SRES);
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t part = cc1101_read_status(CC1101_PARTNUM);
    uint8_t ver = cc1101_read_status(CC1101_VERSION);
    ESP_LOGI(TAG, "PARTNUM=0x%02x VERSION=0x%02x", part, ver);
    // Genuine parts report VERSION 0x14; some clones report 0x04 or 0x17.
    if (ver == 0x00 || ver == 0xFF) {
        return ESP_ERR_NOT_FOUND;
    }
    return ESP_OK;
}

void cc1101_set_freq(uint32_t freq_hz)
{
    uint32_t word = (uint32_t)((((uint64_t)freq_hz << 16) + CC1101_XTAL_HZ / 2) / CC1101_XTAL_HZ);
    cc1101_write_reg(CC1101_FREQ2, (word >> 16) & 0xFF);
    cc1101_write_reg(CC1101_FREQ1, (word >> 8) & 0xFF);
    cc1101_write_reg(CC1101_FREQ0, word & 0xFF);
}

void cc1101_set_cs_threshold(int db)
{
    if (db < -7) db = -7;
    if (db > 7) db = 7;
    // AGC_LNA_PRIORITY=1, relative CS threshold disabled, absolute threshold = db.
    cc1101_write_reg(CC1101_AGCCTRL1, 0x40 | ((uint8_t)db & 0x0F));
}

void cc1101_config_async_fsk_rx(uint32_t freq_hz)
{
    static const uint8_t regs[][2] = {
        {CC1101_IOCFG2,   0x0E}, // GDO2: carrier sense
        {CC1101_IOCFG1,   0x2E}, // GDO1: hi-Z (shared with MISO)
        {CC1101_IOCFG0,   0x0D}, // GDO0: async serial data out
        {CC1101_FIFOTHR,  0x47},
        {CC1101_PKTCTRL1, 0x00},
        {CC1101_PKTCTRL0, 0x32}, // async serial mode, infinite length, no CRC
        {CC1101_FSCTRL1,  0x06}, // IF 152 kHz
        {CC1101_FSCTRL0,  0x00},
        {CC1101_MDMCFG4,  0x89}, // RX BW 203 kHz, DRATE_E=9
        {CC1101_MDMCFG3,  0x83}, // ~19.2 kBaud (Toyota chip rate; only loosely used in async)
        {CC1101_MDMCFG2,  0x00}, // 2-FSK, no sync word, DC filter on
        {CC1101_MDMCFG1,  0x22},
        {CC1101_MDMCFG0,  0xF8},
        {CC1101_DEVIATN,  0x44}, // ~38 kHz: sensors measured at ~314.953 / ~315.030 MHz
        {CC1101_MCSM1,    0x3C}, // stay in RX
        {CC1101_MCSM0,    0x18}, // autocal IDLE->RX
        {CC1101_FOCCFG,   0x16},
        {CC1101_BSCFG,    0x6C},
        {CC1101_AGCCTRL2, 0x43}, // all gain stages, MAGN_TARGET 33 dB
        {CC1101_AGCCTRL1, 0x40},
        {CC1101_AGCCTRL0, 0x91},
        {CC1101_FREND1,   0x56},
        {CC1101_FREND0,   0x10},
        {CC1101_FSCAL3,   0xE9},
        {CC1101_FSCAL2,   0x2A},
        {CC1101_FSCAL1,   0x00},
        {CC1101_FSCAL0,   0x1F},
        {CC1101_TEST2,    0x81},
        {CC1101_TEST1,    0x35},
        {CC1101_TEST0,    0x09},
    };

    cc1101_strobe(CC1101_SIDLE);
    for (size_t i = 0; i < sizeof(regs) / sizeof(regs[0]); i++) {
        cc1101_write_reg(regs[i][0], regs[i][1]);
    }
    cc1101_set_freq(freq_hz);
}

void cc1101_start_rx(void)
{
    cc1101_strobe(CC1101_SIDLE);
    cc1101_strobe(CC1101_SFRX);
    cc1101_strobe(CC1101_SCAL);
    vTaskDelay(pdMS_TO_TICKS(2));
    cc1101_strobe(CC1101_SRX);
}

float cc1101_rssi_dbm(void)
{
    int raw = cc1101_read_status(CC1101_RSSI);
    if (raw >= 128) {
        raw -= 256;
    }
    return raw / 2.0f - 74.0f;
}
