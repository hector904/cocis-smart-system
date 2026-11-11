#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>
#include <time.h>
#include <sys/time.h>
#include <sys/unistd.h>
#include <sys/stat.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_timer.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_sntp.h"

static const char *TAG = "CoCIS";

/* ===================== WIFI CONFIG ===================== */
#define WIFI_SSID "Wokwi-GUEST"
#define WIFI_PASS ""

/* ===================== PINS ===================== */
/* Entry door */
#define PIN_SERVO_IN ((gpio_num_t)13)
#define PIN_BUZZER_IN ((gpio_num_t)32)
#define PIN_RFID_IN_CS ((gpio_num_t)2)

/* Exit door */
#define PIN_SERVO_OUT ((gpio_num_t)19)
#define PIN_BUZZER_OUT ((gpio_num_t)21)
#define PIN_RFID_OUT_CS ((gpio_num_t)18)

/* Shared status LEDs */
#define PIN_LED_GREEN ((gpio_num_t)25)
#define PIN_LED_RED ((gpio_num_t)26)

/* Rooms */
#define PIN_LED_ROOM1 ((gpio_num_t)27)
#define PIN_LED_ROOM2 ((gpio_num_t)14)
#define PIN_PIR1 ((gpio_num_t)34)
#define PIN_PIR2 ((gpio_num_t)35)

/* microSD & OLED Shared SPI2 Host */
#define PIN_SD_CS ((gpio_num_t)5)
#define PIN_SD_SCK ((gpio_num_t)15)
#define PIN_SD_MOSI ((gpio_num_t)16)
#define PIN_SD_MISO ((gpio_num_t)17)
#define MOUNT_POINT "/sdcard"

/* OLED is I2C (wokwi-ssd1306 has no SPI mode) */
#define PIN_OLED_SDA ((gpio_num_t)22)
#define PIN_OLED_SCL ((gpio_num_t)23)
#define OLED_I2C_PORT I2C_NUM_0
#define OLED_ADDR 0x3C
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

/* Both RFID RC522 readers share the SPI3 bus (SCK/MOSI/MISO); each has its own CS */
#define PIN_RFID_SCK ((gpio_num_t)4)
#define PIN_RFID_MOSI ((gpio_num_t)12)
#define PIN_RFID_MISO ((gpio_num_t)33)

/* Buttons: the last 3 free GPIOs after the shared-bus layout above.
 * GPIO36/39 are input-only and have NO internal pull-up -> external 10k pull-ups to 3V3.
 * GPIO0 has an internal pull-up (it is also the BOOT pin: do not hold it at reset). */
#define PIN_BTN_ENTRY_OK ((gpio_num_t)36)
#define PIN_BTN_ENTRY_BAD ((gpio_num_t)39)
#define PIN_BTN_EXIT ((gpio_num_t)0)

/* ===================== CONSTANTS ===================== */
#define DEBOUNCE_MS 250
#define DOOR_OPEN_MS 2500
#define LIGHT_TIMEOUT_MS 8000
#define MENU_PASSWORD "cocis2026"
#define MAX_USERS 8
#define UID_STR_LEN 24

/* ===================== STRUCTURES ===================== */
typedef struct
{
    char uid[UID_STR_LEN];
    bool inside;
} user_t;

typedef struct
{
    uint8_t start_hour;
    uint8_t start_min;
    uint8_t end_hour;
    uint8_t end_min;
} schedule_t;

typedef struct
{
    int32_t occupancy;
    schedule_t schedule;
    bool room1_occupied;
    bool room1_light;
    bool room2_occupied;
    bool room2_light;
    bool authenticated;
} system_state_t;

typedef struct
{
    const char *name;
    ledc_channel_t channel;
    gpio_num_t buzzer;
    bool open;
} door_t;

typedef struct
{
    const char *name;
    gpio_num_t cs;
    spi_device_handle_t spi;
    bool ok;
} rfid_reader_t;

/* ===================== GLOBALS ===================== */
static system_state_t g_state;
static SemaphoreHandle_t g_state_mutex = NULL;
static user_t g_users[MAX_USERS];
static int g_user_count = 0;
static time_t g_time_manual_offset_sec = 0;
static bool g_sd_ok = false;
static bool g_oled_ok = false;

static door_t g_door_in = {"ENTRY", LEDC_CHANNEL_0, PIN_BUZZER_IN, false};
static door_t g_door_out = {"EXIT", LEDC_CHANNEL_1, PIN_BUZZER_OUT, false};

static rfid_reader_t g_rfid_in = {"ENTRY", PIN_RFID_IN_CS, NULL, false};
static rfid_reader_t g_rfid_out = {"EXIT", PIN_RFID_OUT_CS, NULL, false};

/* OLED framebuffer */
static uint8_t g_oled_fb[OLED_WIDTH * OLED_HEIGHT / 8];

/* ===================== 5x7 FONT ===================== */
static const uint8_t font5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, //
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // )
    {0x08, 0x2A, 0x1C, 0x2A, 0x08}, // *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ;
    {0x00, 0x08, 0x14, 0x22, 0x41}, // <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // =
    {0x41, 0x22, 0x14, 0x08, 0x00}, // >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // ?
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // @
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x09, 0x01, 0x01}, // F
    {0x3E, 0x41, 0x41, 0x51, 0x32}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7F, 0x02, 0x04, 0x02, 0x7F}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x7F, 0x20, 0x18, 0x20, 0x7F}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x03, 0x04, 0x78, 0x04, 0x03}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // Z
};

/* ===================== OLED SPI DRIVER ===================== */
static void oled_write(uint8_t control, const uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, control, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(OLED_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
}

static esp_err_t oled_cmd(uint8_t cmd)
{
    oled_write(0x00, &cmd, 1);
    return ESP_OK;
}

static void oled_clear(void)
{
    memset(g_oled_fb, 0, sizeof(g_oled_fb));
}

static void oled_set_pixel(int x, int y, bool on)
{
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT)
        return;
    int idx = x + (y / 8) * OLED_WIDTH;
    if (on)
        g_oled_fb[idx] |= (1 << (y & 7));
    else
        g_oled_fb[idx] &= ~(1 << (y & 7));
}

static void oled_draw_char(int x, int y, char c)
{
    if (c < 32 || c > 90)
        c = '?';
    const uint8_t *glyph = font5x7[c - 32];
    for (int col = 0; col < 5; col++)
    {
        uint8_t line = glyph[col];
        for (int row = 0; row < 7; row++)
        {
            if (line & (1 << row))
                oled_set_pixel(x + col, y + row, true);
        }
    }
}

static void oled_draw_string(int x, int y, const char *str)
{
    while (*str)
    {
        oled_draw_char(x, y, *str);
        x += 6;
        if (x > OLED_WIDTH - 6)
            break;
        str++;
    }
}

static void oled_update(void)
{
    if (!g_oled_ok)
        return;
    oled_cmd(0x21); oled_cmd(0); oled_cmd(127);
    oled_cmd(0x22); oled_cmd(0); oled_cmd(7);
    for (int page = 0; page < 8; page++)
        oled_write(0x40, &g_oled_fb[page * OLED_WIDTH], OLED_WIDTH);
}

static void oled_init(void)
{
    i2c_config_t conf;
    memset(&conf, 0, sizeof(conf));
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = PIN_OLED_SDA;
    conf.scl_io_num = PIN_OLED_SCL;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;

    if (i2c_param_config(OLED_I2C_PORT, &conf) != ESP_OK ||
        i2c_driver_install(OLED_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0) != ESP_OK)
    {
        ESP_LOGE(TAG, "OLED I2C init failed");
        return;
    }
    g_oled_ok = true;

    vTaskDelay(pdMS_TO_TICKS(100));

    static const uint8_t init_seq[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0x8D, 0x14,
        0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1,
        0xDB, 0x40, 0xA4, 0xA6, 0xAF};
    for (size_t i = 0; i < sizeof(init_seq); i++)
        oled_cmd(init_seq[i]);

    oled_clear();
    oled_update();
    ESP_LOGI(TAG, "OLED initialized via I2C (SDA=%d, SCL=%d, addr=0x%02X)",
             PIN_OLED_SDA, PIN_OLED_SCL, OLED_ADDR);
}

/* ===================== RFID RC522 DRIVER (SPI3) ===================== */
#define RC_CommandReg 0x01
#define RC_ComIrqReg 0x04
#define RC_DivIrqReg 0x05
#define RC_ErrorReg 0x06
#define RC_FIFODataReg 0x09
#define RC_FIFOLevelReg 0x0A
#define RC_ControlReg 0x0C
#define RC_BitFramingReg 0x0D
#define RC_CollReg 0x0E
#define RC_ModeReg 0x11
#define RC_TxControlReg 0x14
#define RC_TxASKReg 0x15
#define RC_CRCResultRegH 0x21
#define RC_CRCResultRegL 0x22
#define RC_TModeReg 0x2A
#define RC_TPrescalerReg 0x2B
#define RC_TReloadRegH 0x2C
#define RC_TReloadRegL 0x2D
#define RC_VersionReg 0x37

#define RC_CMD_IDLE 0x00
#define RC_CMD_CALCCRC 0x03
#define RC_CMD_TRANSCEIVE 0x0C
#define RC_CMD_SOFTRESET 0x0F

#define RC_OK 0
#define RC_TIMEOUT 1
#define RC_ERROR 2
#define RC_COLLISION 3

static uint8_t rc_read(rfid_reader_t *rd, uint8_t reg)
{
    uint8_t tx[2] = {(uint8_t)(((reg << 1) & 0x7E) | 0x80), 0x00};
    uint8_t rx[2] = {0, 0};
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 16;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    spi_device_polling_transmit(rd->spi, &t);
    return rx[1];
}

static void rc_write(rfid_reader_t *rd, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = {(uint8_t)((reg << 1) & 0x7E), val};
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 16;
    t.tx_buffer = tx;
    spi_device_polling_transmit(rd->spi, &t);
}

static void rc_set_bitmask(rfid_reader_t *rd, uint8_t reg, uint8_t mask)
{
    rc_write(rd, reg, rc_read(rd, reg) | mask);
}

static void rc_clear_bitmask(rfid_reader_t *rd, uint8_t reg, uint8_t mask)
{
    rc_write(rd, reg, rc_read(rd, reg) & (uint8_t)~mask);
}

static void rc_antenna_on(rfid_reader_t *rd)
{
    if ((rc_read(rd, RC_TxControlReg) & 0x03) != 0x03)
        rc_set_bitmask(rd, RC_TxControlReg, 0x03);
}

static bool rc_calc_crc(rfid_reader_t *rd, const uint8_t *data, uint8_t len, uint8_t *crc)
{
    rc_write(rd, RC_CommandReg, RC_CMD_IDLE);
    rc_write(rd, RC_DivIrqReg, 0x04);
    rc_write(rd, RC_FIFOLevelReg, 0x80);
    for (uint8_t i = 0; i < len; i++)
        rc_write(rd, RC_FIFODataReg, data[i]);
    rc_write(rd, RC_CommandReg, RC_CMD_CALCCRC);

    for (int i = 0; i < 250; i++)
    {
        if (rc_read(rd, RC_DivIrqReg) & 0x04)
        {
            rc_write(rd, RC_CommandReg, RC_CMD_IDLE);
            crc[0] = rc_read(rd, RC_CRCResultRegL);
            crc[1] = rc_read(rd, RC_CRCResultRegH);
            return true;
        }
        esp_rom_delay_us(200);
    }
    rc_write(rd, RC_CommandReg, RC_CMD_IDLE);
    return false;
}

static int rc_transceive(rfid_reader_t *rd, const uint8_t *send, uint8_t send_len,
                         uint8_t *recv, uint8_t *recv_len, uint8_t tx_last_bits)
{
    rc_write(rd, RC_CommandReg, RC_CMD_IDLE);
    rc_write(rd, RC_ComIrqReg, 0x7F);
    rc_write(rd, RC_FIFOLevelReg, 0x80);
    for (uint8_t i = 0; i < send_len; i++)
        rc_write(rd, RC_FIFODataReg, send[i]);
    rc_write(rd, RC_BitFramingReg, tx_last_bits & 0x07);
    rc_write(rd, RC_CommandReg, RC_CMD_TRANSCEIVE);
    rc_set_bitmask(rd, RC_BitFramingReg, 0x80);

    int i;
    bool done = false;
    for (i = 0; i < 150; i++)
    {
        uint8_t irq = rc_read(rd, RC_ComIrqReg);
        if (irq & 0x30)
        {
            done = true;
            break;
        }
        if (irq & 0x01)
            break;
        esp_rom_delay_us(200);
    }
    rc_clear_bitmask(rd, RC_BitFramingReg, 0x80);
    if (!done)
        return RC_TIMEOUT;

    uint8_t err = rc_read(rd, RC_ErrorReg);
    if (err & 0x13)
        return RC_ERROR;

    uint8_t n = rc_read(rd, RC_FIFOLevelReg);
    if (recv && recv_len)
    {
        if (n > *recv_len)
            return RC_ERROR;
        for (uint8_t k = 0; k < n; k++)
            recv[k] = rc_read(rd, RC_FIFODataReg);
        *recv_len = n;
    }
    if (err & 0x08)
        return RC_COLLISION;
    return RC_OK;
}

static bool rc_request(rfid_reader_t *rd)
{
    rc_clear_bitmask(rd, RC_CollReg, 0x80);
    uint8_t cmd = 0x26;
    uint8_t atqa[2];
    uint8_t len = sizeof(atqa);
    int s = rc_transceive(rd, &cmd, 1, atqa, &len, 7);
    return (s == RC_OK && len == 2);
}

static bool rc_read_uid(rfid_reader_t *rd, uint8_t *uid, uint8_t *uid_len)
{
    static const uint8_t sel_cmd[3] = {0x93, 0x95, 0x97};
    uint8_t n = 0;

    for (int level = 0; level < 3; level++)
    {
        uint8_t ac[2] = {sel_cmd[level], 0x20};
        uint8_t buf[5];
        uint8_t len = sizeof(buf);

        if (rc_transceive(rd, ac, 2, buf, &len, 0) != RC_OK || len != 5)
            return false;
        if ((uint8_t)(buf[0] ^ buf[1] ^ buf[2] ^ buf[3]) != buf[4])
            return false;

        uint8_t sel[9] = {sel_cmd[level], 0x70, buf[0], buf[1], buf[2], buf[3], buf[4], 0, 0};
        if (!rc_calc_crc(rd, sel, 7, &sel[7]))
            return false;

        uint8_t sak[3];
        uint8_t sak_len = sizeof(sak);
        if (rc_transceive(rd, sel, 9, sak, &sak_len, 0) != RC_OK || sak_len != 3)
            return false;

        if (sak[0] & 0x04)
        {
            memcpy(&uid[n], &buf[1], 3);
            n += 3;
            continue;
        }
        memcpy(&uid[n], buf, 4);
        n += 4;
        *uid_len = n;
        return true;
    }
    return false;
}

static void rc_halt(rfid_reader_t *rd)
{
    uint8_t hlt[4] = {0x50, 0x00, 0, 0};
    if (rc_calc_crc(rd, hlt, 2, &hlt[2]))
        rc_transceive(rd, hlt, 4, NULL, NULL, 0);
}

static bool rfid_bus_init(void)
{
    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.mosi_io_num = PIN_RFID_MOSI;
    buscfg.miso_io_num = PIN_RFID_MISO;
    buscfg.sclk_io_num = PIN_RFID_SCK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 64;

    esp_err_t ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "RFID SPI bus init failed: %s", esp_err_to_name(ret));
        return false;
    }
    return true;
}

static bool rfid_reader_init(rfid_reader_t *rd)
{
    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(devcfg));
    devcfg.clock_speed_hz = 4000000;
    devcfg.mode = 0;
    devcfg.spics_io_num = rd->cs;
    devcfg.queue_size = 3;

    esp_err_t ret = spi_bus_add_device(SPI3_HOST, &devcfg, &rd->spi);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "RFID %s: SPI add device failed: %s", rd->name, esp_err_to_name(ret));
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    rc_write(rd, RC_CommandReg, RC_CMD_SOFTRESET);
    for (int i = 0; i < 50; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(2));
        if (!(rc_read(rd, RC_CommandReg) & 0x10))
            break;
    }

    uint8_t ver = rc_read(rd, RC_VersionReg);
    if (ver == 0x00 || ver == 0xFF)
    {
        ESP_LOGW(TAG, "RFID %s: RC522 not detected (VersionReg=0x%02X)", rd->name, ver);
        return false;
    }

    rc_write(rd, RC_TModeReg, 0x80);
    rc_write(rd, RC_TPrescalerReg, 0xA9);
    rc_write(rd, RC_TReloadRegH, 0x03);
    rc_write(rd, RC_TReloadRegL, 0xE8);
    rc_write(rd, RC_TxASKReg, 0x40);
    rc_write(rd, RC_ModeReg, 0x3D);
    rc_antenna_on(rd);

    ESP_LOGI(TAG, "RFID %s: RC522 ready on SPI3, CS=%d (VersionReg=0x%02X)", rd->name, rd->cs, ver);
    return true;
}

static bool rfid_check_card(rfid_reader_t *rd, char *uid_str, size_t cap)
{
    if (!rc_request(rd))
        return false;

    uint8_t uid[10];
    uint8_t uid_len = 0;
    if (!rc_read_uid(rd, uid, &uid_len))
        return false;

    rc_halt(rd);

    if (cap < (size_t)(uid_len * 2 + 1))
        return false;
    for (uint8_t i = 0; i < uid_len; i++)
        snprintf(&uid_str[i * 2], 3, "%02X", uid[i]);
    return true;
}

/* ===================== SD CARD (SPI2 Host) ===================== */
static bool sd_init(void)
{
    esp_err_t ret;

    spi_bus_config_t bus_cfg;
    memset(&bus_cfg, 0, sizeof(bus_cfg));
    bus_cfg.mosi_io_num = PIN_SD_MOSI;
    bus_cfg.miso_io_num = PIN_SD_MISO;
    bus_cfg.sclk_io_num = PIN_SD_SCK;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = 4000;

    ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI2 bus init failed: %s", esp_err_to_name(ret));
        return false;
    }

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    host.max_freq_khz = 10000;

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_SD_CS;
    slot_config.host_id = SPI2_HOST;

    sdmmc_card_t *card;
    esp_vfs_fat_sdmmc_mount_config_t mount_config;
    memset(&mount_config, 0, sizeof(mount_config));
    mount_config.format_if_mount_failed = true;
    mount_config.max_files = 5;
    mount_config.allocation_unit_size = 16 * 1024;

    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SD mount failed (%s)", esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "SD card mounted at %s", MOUNT_POINT);
    sdmmc_card_print_info(stdout, card);
    return true;
}

/* ===================== WIFI & NTP ===================== */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Retrying Wi-Fi...");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
    }
}

static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config));
    strcpy((char *)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char *)wifi_config.sta.password, WIFI_PASS);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void obtain_time(void)
{
    wifi_init_sta();

    ESP_LOGI(TAG, "Initializing SNTP...");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();

    time_t now = 0;
    struct tm timeinfo;
    memset(&timeinfo, 0, sizeof(timeinfo));
    int retry = 0;
    const int retry_count = 10;

    while (timeinfo.tm_year < (2025 - 1900) && ++retry < retry_count)
    {
        ESP_LOGI(TAG, "Waiting for time... (%d/%d)", retry, retry_count);
        vTaskDelay(pdMS_TO_TICKS(1000));
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    // Fallback for offline simulation environment where NTP is blocked
    if (timeinfo.tm_year < (2025 - 1900))
    {
        ESP_LOGW(TAG, "NTP sync timed out (offline simulation). Initializing realistic start time.");
        struct tm default_time = {
            .tm_year = 2026 - 1900,
            .tm_mon = 10,  // November (0-indexed)
            .tm_mday = 11,
            .tm_hour = 20, // 8:00 PM realistic start
            .tm_min = 11,
            .tm_sec = 0
        };
        time_t t = mktime(&default_time);
        struct timeval tv = { .tv_sec = t, .tv_usec = 0 };
        settimeofday(&tv, NULL);
    }

    setenv("TZ", "UTC-3", 1);
    tzset();
    ESP_LOGI(TAG, "Time synchronized / initialized");
}

/* ===================== TIME & SCHEDULE ===================== */
static void get_current_time(int *hour, int *min, int *sec)
{
    time_t now;
    struct tm timeinfo;
    memset(&timeinfo, 0, sizeof(timeinfo));
    time(&now);
    time_t adjusted = now + g_time_manual_offset_sec;
    localtime_r(&adjusted, &timeinfo);
    *hour = timeinfo.tm_hour;
    *min = timeinfo.tm_min;
    *sec = timeinfo.tm_sec;
}

static bool is_within_schedule(void)
{
    int h, m, s;
    get_current_time(&h, &m, &s);
    int now_min = h * 60 + m;
    int start_min = g_state.schedule.start_hour * 60 + g_state.schedule.start_min;
    int end_min = g_state.schedule.end_hour * 60 + g_state.schedule.end_min;

    if (start_min <= end_min)
        return (now_min >= start_min && now_min < end_min);
    return (now_min >= start_min || now_min < end_min);
}

/* ===================== NVS ===================== */
static void load_schedule_from_nvs(void)
{
    nvs_handle_t handle;
    if (nvs_open("cocis", NVS_READONLY, &handle) == ESP_OK)
    {
        uint8_t v;
        if (nvs_get_u8(handle, "sh", &v) == ESP_OK)
            g_state.schedule.start_hour = v;
        if (nvs_get_u8(handle, "sm", &v) == ESP_OK)
            g_state.schedule.start_min = v;
        if (nvs_get_u8(handle, "eh", &v) == ESP_OK)
            g_state.schedule.end_hour = v;
        if (nvs_get_u8(handle, "em", &v) == ESP_OK)
            g_state.schedule.end_min = v;
        nvs_close(handle);
    }
}

static void save_schedule_to_nvs(void)
{
    nvs_handle_t handle;
    if (nvs_open("cocis", NVS_READWRITE, &handle) == ESP_OK)
    {
        nvs_set_u8(handle, "sh", g_state.schedule.start_hour);
        nvs_set_u8(handle, "sm", g_state.schedule.start_min);
        nvs_set_u8(handle, "eh", g_state.schedule.end_hour);
        nvs_set_u8(handle, "em", g_state.schedule.end_min);
        nvs_commit(handle);
        nvs_close(handle);
    }
}

/* ===================== HARDWARE HELPERS ===================== */
static void servo_set_angle(ledc_channel_t channel, int angle)
{
    uint32_t duty = 1638 + (angle * (8191 - 1638)) / 180;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

static void door_open(door_t *d)
{
    servo_set_angle(d->channel, 90);
    gpio_set_level(PIN_LED_GREEN, 1);
    gpio_set_level(PIN_LED_RED, 0);
    gpio_set_level(d->buzzer, 1);
    vTaskDelay(pdMS_TO_TICKS(80));
    gpio_set_level(d->buzzer, 0);

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    d->open = true;
    xSemaphoreGive(g_state_mutex);

    vTaskDelay(pdMS_TO_TICKS(DOOR_OPEN_MS));

    servo_set_angle(d->channel, 0);
    gpio_set_level(PIN_LED_GREEN, 0);

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    d->open = false;
    xSemaphoreGive(g_state_mutex);
}

static void access_denied(door_t *d)
{
    gpio_set_level(PIN_LED_RED, 1);
    for (int i = 0; i < 3; i++)
    {
        gpio_set_level(d->buzzer, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(d->buzzer, 0);
        vTaskDelay(pdMS_TO_TICKS(80));
    }
    vTaskDelay(pdMS_TO_TICKS(400));
    gpio_set_level(PIN_LED_RED, 0);
}

static void log_event(const char *uid, const char *dir, const char *result)
{
    int h, m, s;
    get_current_time(&h, &m, &s);
    int occ;
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    occ = g_state.occupancy;
    xSemaphoreGive(g_state_mutex);

    ESP_LOGI(TAG, "LOG %02d:%02d:%02d | UID=%s | %s | %s | occ=%d",
             h, m, s, uid, dir, result, occ);

    if (g_sd_ok)
    {
        FILE *f = fopen(MOUNT_POINT "/access_log.txt", "a");
        if (f)
        {
            fprintf(f, "%02d:%02d:%02d,UID=%s,%s,%s,occ=%d\n",
                    h, m, s, uid, dir, result, occ);
            fclose(f);
        }
    }
}

/* ===================== ACCESS LOGIC ===================== */
static bool is_registered(const char *uid)
{
    for (int i = 0; i < g_user_count; i++)
    {
        if (strcmp(g_users[i].uid, uid) == 0)
            return true;
    }
    return false;
}

static void process_entry(const char *uid, bool valid_card)
{
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);

    if (!is_within_schedule())
    {
        xSemaphoreGive(g_state_mutex);
        log_event(uid, "IN", "DENIED_SCHEDULE");
        access_denied(&g_door_in);
        return;
    }
    if (!valid_card)
    {
        xSemaphoreGive(g_state_mutex);
        log_event(uid, "IN", "DENIED_INVALID");
        access_denied(&g_door_in);
        return;
    }

    int idx = -1;
    for (int i = 0; i < g_user_count; i++)
    {
        if (strcmp(g_users[i].uid, uid) == 0)
        {
            idx = i;
            break;
        }
    }
    if (idx >= 0 && g_users[idx].inside)
    {
        xSemaphoreGive(g_state_mutex);
        log_event(uid, "IN", "DENIED_ALREADY_IN");
        access_denied(&g_door_in);
        return;
    }

    g_state.occupancy++;
    if (idx >= 0)
        g_users[idx].inside = true;
    xSemaphoreGive(g_state_mutex);

    log_event(uid, "IN", "GRANTED");
    door_open(&g_door_in);
}

static void process_exit(const char *uid, bool valid_card)
{
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);

    if (!valid_card)
    {
        xSemaphoreGive(g_state_mutex);
        log_event(uid, "OUT", "DENIED_INVALID");
        access_denied(&g_door_out);
        return;
    }

    if (!is_within_schedule() && g_state.occupancy > 0)
    {
        xSemaphoreGive(g_state_mutex);
        log_event(uid, "OUT", "DENIED_SCHEDULE");
        access_denied(&g_door_out);
        return;
    }

    int idx = -1;
    for (int i = 0; i < g_user_count; i++)
    {
        if (strcmp(g_users[i].uid, uid) == 0)
        {
            idx = i;
            break;
        }
    }
    if (idx >= 0 && !g_users[idx].inside)
    {
        xSemaphoreGive(g_state_mutex);
        log_event(uid, "OUT", "DENIED_NOT_INSIDE");
        access_denied(&g_door_out);
        return;
    }

    if (g_state.occupancy > 0)
        g_state.occupancy--;
    if (idx >= 0)
        g_users[idx].inside = false;
    xSemaphoreGive(g_state_mutex);

    log_event(uid, "OUT", "GRANTED");
    door_open(&g_door_out);
}

static void handle_rfid_entry(const char *uid)
{
    bool registered = is_registered(uid);
    process_entry(uid, registered);
}

static void handle_rfid_exit(const char *uid)
{
    bool registered = is_registered(uid);
    process_exit(uid, registered);
}

/* ===================== DISPLAY TASK ===================== */
static void display_task(void *arg)
{
    char line[22];

    while (1)
    {
        int h, m, s;
        get_current_time(&h, &m, &s);

        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        int32_t occ = g_state.occupancy;
        bool door_in = g_door_in.open;
        bool door_out = g_door_out.open;
        bool r1 = g_state.room1_light;
        bool r2 = g_state.room2_light;
        bool sched = is_within_schedule();
        uint8_t sh = g_state.schedule.start_hour;
        uint8_t sm = g_state.schedule.start_min;
        uint8_t eh = g_state.schedule.end_hour;
        uint8_t em = g_state.schedule.end_min;
        xSemaphoreGive(g_state_mutex);

        oled_clear();

        oled_draw_string(0, 0, "COCIS SMART");
        snprintf(line, sizeof(line), "RF:%c%c", g_rfid_in.ok ? 'Y' : 'N', g_rfid_out.ok ? 'Y' : 'N');
        oled_draw_string(90, 0, line);
        snprintf(line, sizeof(line), "TIME %02d:%02d:%02d", h, m, s);
        oled_draw_string(0, 10, line);
        snprintf(line, sizeof(line), "OCC:%ld IN:%c OUT:%c", (long)occ,
                 door_in ? 'O' : 'C', door_out ? 'O' : 'C');
        oled_draw_string(0, 20, line);
        snprintf(line, sizeof(line), "R1:%s  R2:%s", r1 ? "ON " : "OFF", r2 ? "ON " : "OFF");
        oled_draw_string(0, 30, line);
        snprintf(line, sizeof(line), "SCH %02d:%02d-%02d:%02d", sh, sm, eh, em);
        oled_draw_string(0, 40, line);
        snprintf(line, sizeof(line), "ACCESS:%s", sched ? "ALLOWED" : "DENIED");
        oled_draw_string(0, 50, line);

        snprintf(line, sizeof(line), "SD:%s", g_sd_ok ? "OK" : "FAIL");
        oled_draw_string(90, 50, line);

        oled_update();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ===================== RFID TASKS ===================== */
static void rfid_task(void *arg)
{
    rfid_reader_t *rd = (rfid_reader_t *)arg;
    bool is_entry = (rd == &g_rfid_in);

    while (1)
    {
        char uid[UID_STR_LEN];
        if (rfid_check_card(rd, uid, sizeof(uid)))
        {
            if (is_entry)
                handle_rfid_entry(uid);
            else
                handle_rfid_exit(uid);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ===================== BUTTON TASK ===================== */
/* Pushbuttons (active low) that simulate card taps:
 *   ENTRY_OK  -> registered card A1B2C3D4 at the entry door
 *   ENTRY_BAD -> unknown card DEADBEEF at the entry door
 *   EXIT      -> registered card A1B2C3D4 at the exit door */
static void button_task(void *arg)
{
    bool last[3] = {false, false, false};
    TickType_t last_press[3] = {0, 0, 0};

    while (1)
    {
        bool cur[3] = {
            gpio_get_level(PIN_BTN_ENTRY_OK) == 0,
            gpio_get_level(PIN_BTN_ENTRY_BAD) == 0,
            gpio_get_level(PIN_BTN_EXIT) == 0,
        };
        TickType_t now = xTaskGetTickCount();

        for (int i = 0; i < 3; i++)
        {
            if (cur[i] && !last[i] && (now - last_press[i] > pdMS_TO_TICKS(DEBOUNCE_MS)))
            {
                last_press[i] = now;
                if (i == 0)
                    process_entry("A1B2C3D4", true);
                else if (i == 1)
                    process_entry("DEADBEEF", false);
                else
                    process_exit("A1B2C3D4", true);
                now = xTaskGetTickCount();
            }
            last[i] = cur[i];
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void lighting_task(void *arg)
{
    int64_t last_motion1 = 0, last_motion2 = 0;

    while (1)
    {
        bool pir1 = gpio_get_level(PIN_PIR1) == 1;
        bool pir2 = gpio_get_level(PIN_PIR2) == 1;
        int64_t now = esp_timer_get_time() / 1000;

        xSemaphoreTake(g_state_mutex, portMAX_DELAY);

        if (pir1)
        {
            last_motion1 = now;
            g_state.room1_occupied = true;
            if (!g_state.room1_light)
            {
                g_state.room1_light = true;
                gpio_set_level(PIN_LED_ROOM1, 1);
            }
        }
        else if (g_state.room1_light && (now - last_motion1 > LIGHT_TIMEOUT_MS))
        {
            g_state.room1_occupied = false;
            g_state.room1_light = false;
            gpio_set_level(PIN_LED_ROOM1, 0);
        }

        if (pir2)
        {
            last_motion2 = now;
            g_state.room2_occupied = true;
            if (!g_state.room2_light)
            {
                g_state.room2_light = true;
                gpio_set_level(PIN_LED_ROOM2, 1);
            }
        }
        else if (g_state.room2_light && (now - last_motion2 > LIGHT_TIMEOUT_MS))
        {
            g_state.room2_occupied = false;
            g_state.room2_light = false;
            gpio_set_level(PIN_LED_ROOM2, 0);
        }

        xSemaphoreGive(g_state_mutex);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/* ===================== SERIAL MENU ===================== */
static void print_menu(void)
{
    printf("\n========== CoCIS Management Menu ==========\n");
    printf("1. Show building occupancy\n");
    printf("2. Show room status & lights\n");
    printf("3. Show current time & schedule\n");
    printf("4. Change access schedule (demo 07:00-19:00)\n");
    printf("5. Force Room1 light ON/OFF\n");
    printf("6. Force Room2 light ON/OFF\n");
    printf("7. Advance demo time (+1 hour)\n");
    printf("8. System status\n");
    printf("9. Simulate ENTRY card scan\n");
    printf("10. Simulate EXIT card scan\n");
    printf("0. Logout\n");
    printf("===========================================\n");
    printf("Choice: ");
    fflush(stdout);
}

static void serial_menu_task(void *arg)
{
    char line[64];
    int idx = 0;
    int awaiting_uid = 0;

    printf("\n\n*** CoCIS Smart Control System ***\n");
    printf("Password: ");
    fflush(stdout);

    while (1)
    {
        int c = getchar();
        if (c == EOF)
        {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (c == '\n' || c == '\r')
        {
            line[idx] = 0;
            idx = 0;

            if (strlen(line) == 0)
            {
                continue;
            }

            if (!g_state.authenticated)
            {
                if (strcmp(line, MENU_PASSWORD) == 0)
                {
                    g_state.authenticated = true;
                    printf("Login successful.\n");
                    print_menu();
                }
                else
                {
                    printf("Wrong password. Try again: ");
                    fflush(stdout);
                }
                continue;
            }

            if (awaiting_uid)
            {
                int which = awaiting_uid;
                awaiting_uid = 0;
                for (char *p = line; *p; p++)
                    *p = (char)toupper((unsigned char)*p);

                if (strlen(line) > 0 && strlen(line) < UID_STR_LEN)
                {
                    if (which == 1)
                        handle_rfid_entry(line);
                    else
                        handle_rfid_exit(line);
                }
                print_menu();
                continue;
            }

            if (strcmp(line, "0") == 0)
            {
                g_state.authenticated = false;
                printf("Logged out.\nPassword: ");
                fflush(stdout);
            }
            else if (strcmp(line, "1") == 0)
            {
                xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                printf("Building occupancy: %ld\n", (long)g_state.occupancy);
                xSemaphoreGive(g_state_mutex);
                fflush(stdout);
                print_menu();
            }
            else if (strcmp(line, "2") == 0)
            {
                xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                printf("Room 1: occupied=%s  light=%s\n",
                       g_state.room1_occupied ? "YES" : "NO",
                       g_state.room1_light ? "ON" : "OFF");
                printf("Room 2: occupied=%s  light=%s\n",
                       g_state.room2_occupied ? "YES" : "NO",
                       g_state.room2_light ? "ON" : "OFF");
                xSemaphoreGive(g_state_mutex);
                fflush(stdout);
                print_menu();
            }
            else if (strcmp(line, "3") == 0)
            {
                int h, m, s;
                get_current_time(&h, &m, &s);
                printf("Current time : %02d:%02d:%02d\n", h, m, s);
                printf("Schedule     : %02d:%02d - %02d:%02d\n",
                       g_state.schedule.start_hour, g_state.schedule.start_min,
                       g_state.schedule.end_hour, g_state.schedule.end_min);
                printf("Within schedule: %s\n", is_within_schedule() ? "YES" : "NO");
                fflush(stdout);
                print_menu();
            }
            else if (strcmp(line, "4") == 0)
            {
                g_state.schedule.start_hour = 7;
                g_state.schedule.start_min = 0;
                g_state.schedule.end_hour = 19;
                g_state.schedule.end_min = 0;
                save_schedule_to_nvs();
                printf("Schedule set to 07:00 - 19:00\n");
                fflush(stdout);
                print_menu();
            }
            else if (strcmp(line, "5") == 0)
            {
                xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                g_state.room1_light = !g_state.room1_light;
                gpio_set_level(PIN_LED_ROOM1, g_state.room1_light ? 1 : 0);
                printf("Room 1 light forced %s\n", g_state.room1_light ? "ON" : "OFF");
                xSemaphoreGive(g_state_mutex);
                fflush(stdout);
                print_menu();
            }
            else if (strcmp(line, "6") == 0)
            {
                xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                g_state.room2_light = !g_state.room2_light;
                gpio_set_level(PIN_LED_ROOM2, g_state.room2_light ? 1 : 0);
                printf("Room 2 light forced %s\n", g_state.room2_light ? "ON" : "OFF");
                xSemaphoreGive(g_state_mutex);
                fflush(stdout);
                print_menu();
            }
            else if (strcmp(line, "7") == 0)
            {
                g_time_manual_offset_sec += 3600;
                int h, m, s;
                get_current_time(&h, &m, &s);
                printf("Time advanced -> now %02d:%02d:%02d\n", h, m, s);
                fflush(stdout);
                print_menu();
            }
            else if (strcmp(line, "8") == 0)
            {
                xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                printf("--- System Status ---\n");
                printf("Occupancy    : %ld\n", (long)g_state.occupancy);
                printf("SD Card      : %s\n", g_sd_ok ? "OK" : "FAIL");
                printf("Entry reader : %s\n", g_rfid_in.ok ? "OK" : "NOT DETECTED");
                printf("Exit reader  : %s\n", g_rfid_out.ok ? "OK" : "NOT DETECTED");
                xSemaphoreGive(g_state_mutex);
                fflush(stdout);
                print_menu();
            }
            else if (strcmp(line, "9") == 0)
            {
                awaiting_uid = 1;
                printf("ENTRY scan UID: ");
                fflush(stdout);
            }
            else if (strcmp(line, "10") == 0)
            {
                awaiting_uid = 2;
                printf("EXIT scan UID: ");
                fflush(stdout);
            }
            else
            {
                print_menu();
            }
        }
        else if (idx < (int)sizeof(line) - 1)
        {
            line[idx++] = (char)c;
            putchar(c);
            fflush(stdout);
        }
    }
}

/* ===================== INIT ===================== */
static void gpio_init_all(void)
{
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << PIN_LED_GREEN) | (1ULL << PIN_LED_RED) |
                        (1ULL << PIN_LED_ROOM1) | (1ULL << PIN_LED_ROOM2) |
                        (1ULL << PIN_BUZZER_IN) | (1ULL << PIN_BUZZER_OUT),
        .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&out);

    gpio_config_t pir = {
        .pin_bit_mask = (1ULL << PIN_PIR1) | (1ULL << PIN_PIR2),
        .mode = GPIO_MODE_INPUT
    };
    gpio_config(&pir);

    /* GPIO36/39: input-only, no internal pull-up (external 10k pull-ups in the diagram) */
    gpio_config_t btn_ext = {
        .pin_bit_mask = (1ULL << PIN_BTN_ENTRY_OK) | (1ULL << PIN_BTN_ENTRY_BAD),
        .mode = GPIO_MODE_INPUT
    };
    gpio_config(&btn_ext);

    /* GPIO0: internal pull-up */
    gpio_config_t btn_int = {
        .pin_bit_mask = (1ULL << PIN_BTN_EXIT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&btn_int);

    ledc_timer_config_t tcfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&tcfg);

    ledc_channel_config_t ccfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };

    ccfg.gpio_num = PIN_SERVO_IN;
    ccfg.channel = g_door_in.channel;
    ledc_channel_config(&ccfg);

    ccfg.gpio_num = PIN_SERVO_OUT;
    ccfg.channel = g_door_out.channel;
    ledc_channel_config(&ccfg);

    servo_set_angle(g_door_in.channel, 0);
    servo_set_angle(g_door_out.channel, 0);
}

static void init_users(void)
{
    strcpy(g_users[0].uid, "A1B2C3D4");
    g_users[0].inside = false;
    g_user_count = 1;
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting CoCIS Smart Control System...");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        nvs_flash_init();
    }

    obtain_time();

    g_state.schedule.start_hour = 8;
    g_state.schedule.start_min = 0;
    g_state.schedule.end_hour = 18;
    g_state.schedule.end_min = 0;
    g_state.occupancy = 0;
    g_state.authenticated = false;
    g_state.room1_light = false;
    g_state.room2_light = false;

    load_schedule_from_nvs();
    init_users();

    g_state_mutex = xSemaphoreCreateMutex();
    gpio_init_all();

    /* SD card & OLED initialized on shared SPI2 Host */
    g_sd_ok = sd_init();
    oled_init();

    /* Both RC522 readers on shared SPI3 Host */
    bool rfid_bus_ok = rfid_bus_init();
    g_rfid_in.ok = rfid_bus_ok && rfid_reader_init(&g_rfid_in);
    g_rfid_out.ok = rfid_bus_ok && rfid_reader_init(&g_rfid_out);

    if (g_rfid_in.ok)
        xTaskCreate(rfid_task, "rfid_in", 5120, &g_rfid_in, 5, NULL);
    if (g_rfid_out.ok)
        xTaskCreate(rfid_task, "rfid_out", 5120, &g_rfid_out, 5, NULL);

    xTaskCreate(button_task, "buttons", 5120, NULL, 5, NULL);
    xTaskCreate(lighting_task, "lights", 4096, NULL, 4, NULL);
    xTaskCreate(serial_menu_task, "menu", 4096, NULL, 3, NULL);
    xTaskCreate(display_task, "display", 4096, NULL, 3, NULL);

    ESP_LOGI(TAG, "System ready. Password: %s", MENU_PASSWORD);
}