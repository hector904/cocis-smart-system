#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
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

/* ===================== PINS (NON-CONFLICTING) ===================== */
#define PIN_SERVO ((gpio_num_t)13)
#define PIN_LED_GREEN ((gpio_num_t)25)
#define PIN_LED_RED ((gpio_num_t)26)
#define PIN_LED_ROOM1 ((gpio_num_t)27)
#define PIN_LED_ROOM2 ((gpio_num_t)14)
#define PIN_BUZZER ((gpio_num_t)32)
#define PIN_PIR1 ((gpio_num_t)34)
#define PIN_PIR2 ((gpio_num_t)35)

/* Buttons */
#define PIN_BTN_ENTRY_OK ((gpio_num_t)18)
#define PIN_BTN_ENTRY_BAD ((gpio_num_t)19)
#define PIN_BTN_EXIT ((gpio_num_t)21)

/* OLED I2C */
#define PIN_OLED_SDA ((gpio_num_t)23)
#define PIN_OLED_SCL ((gpio_num_t)22)
#define OLED_I2C_ADDR 0x3C
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

/* microSD (SPI2 Host) */
#define PIN_SD_CS ((gpio_num_t)5)
#define PIN_SD_SCK ((gpio_num_t)15)
#define PIN_SD_MOSI ((gpio_num_t)16)
#define PIN_SD_MISO ((gpio_num_t)17)
#define MOUNT_POINT "/sdcard"

/* RFID RC522 (SPI3 Host - Completely Separate Pins) */
#define PIN_RFID_CS ((gpio_num_t)2)
#define PIN_RFID_SCK ((gpio_num_t)4)
#define PIN_RFID_MOSI ((gpio_num_t)12)
#define PIN_RFID_MISO ((gpio_num_t)33)

/* ===================== CONSTANTS ===================== */
#define DOOR_OPEN_MS 2500
#define LIGHT_TIMEOUT_MS 8000
#define MENU_PASSWORD "cocis2026"
#define MAX_USERS 8
#define DEBOUNCE_MS 250
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000

/* ===================== STRUCTURES ===================== */
typedef struct
{
    char uid[16];
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
    bool door_open;
    bool authenticated;
} system_state_t;

/* ===================== GLOBALS ===================== */
static system_state_t g_state;
static SemaphoreHandle_t g_state_mutex = NULL;
static user_t g_users[MAX_USERS];
static int g_user_count = 0;
static time_t g_time_manual_offset_sec = 0;
static bool g_sd_ok = false;
static spi_device_handle_t g_rfid_spi = NULL;

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

/* ===================== OLED DRIVER ===================== */
static esp_err_t oled_cmd(uint8_t cmd)
{
    uint8_t buf[2] = {0x00, cmd};
    return i2c_master_write_to_device(I2C_MASTER_NUM, OLED_I2C_ADDR, buf, 2, pdMS_TO_TICKS(100));
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
    oled_cmd(0x21);
    oled_cmd(0);
    oled_cmd(127);
    oled_cmd(0x22);
    oled_cmd(0);
    oled_cmd(7);

    uint8_t buf[17];
    buf[0] = 0x40;
    for (int i = 0; i < (int)sizeof(g_oled_fb); i += 16)
    {
        memcpy(&buf[1], &g_oled_fb[i], 16);
        i2c_master_write_to_device(I2C_MASTER_NUM, OLED_I2C_ADDR, buf, 17, pdMS_TO_TICKS(100));
    }
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
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));

    vTaskDelay(pdMS_TO_TICKS(100));

    oled_cmd(0xAE);
    oled_cmd(0xD5);
    oled_cmd(0x80);
    oled_cmd(0xA8);
    oled_cmd(0x3F);
    oled_cmd(0xD3);
    oled_cmd(0x00);
    oled_cmd(0x40);
    oled_cmd(0x8D);
    oled_cmd(0x14);
    oled_cmd(0x20);
    oled_cmd(0x00);
    oled_cmd(0xA1);
    oled_cmd(0xC8);
    oled_cmd(0xDA);
    oled_cmd(0x12);
    oled_cmd(0x81);
    oled_cmd(0xCF);
    oled_cmd(0xD9);
    oled_cmd(0xF1);
    oled_cmd(0xDB);
    oled_cmd(0x40);
    oled_cmd(0xA4);
    oled_cmd(0xA6);
    oled_cmd(0xAF);

    oled_clear();
    oled_update();
    ESP_LOGI(TAG, "OLED initialized (SDA=%d SCL=%d)", PIN_OLED_SDA, PIN_OLED_SCL);
}

/* ===================== RFID RC522 DRIVER (SPI3 Host) ===================== */
static uint8_t rfid_read_reg(uint8_t reg)
{
    uint8_t tx_data = ((reg << 1) & 0x7E) | 0x80;
    uint8_t rx_data = 0;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &tx_data;
    t.rx_buffer = &rx_data;
    spi_device_transmit(g_rfid_spi, &t);
    return rx_data;
}

static void rfid_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx_data[2] = { (uint8_t)((reg << 1) & 0x7E), val };
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 16;
    t.tx_buffer = tx_data;
    spi_device_transmit(g_rfid_spi, &t);
}

static void rfid_set_bitmask(uint8_t reg, uint8_t mask)
{
    rfid_write_reg(reg, rfid_read_reg(reg) | mask);
}

static void rfid_antenna_on(void)
{
    uint8_t val = rfid_read_reg(0x14);
    if (!(val & 0x03))
    {
        rfid_set_bitmask(0x14, 0x03);
    }
}

static void rfid_init(void)
{
    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.mosi_io_num = PIN_RFID_MOSI;
    buscfg.miso_io_num = PIN_RFID_MISO;
    buscfg.sclk_io_num = PIN_RFID_SCK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 64;

    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(devcfg));
    devcfg.clock_speed_hz = 5000000;
    devcfg.mode = 0;
    devcfg.spics_io_num = PIN_RFID_CS;
    devcfg.queue_size = 7;

    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &devcfg, &g_rfid_spi));

    vTaskDelay(pdMS_TO_TICKS(50));

    rfid_write_reg(0x01, 0x0F);
    vTaskDelay(pdMS_TO_TICKS(50));

    rfid_write_reg(0x2A, 0x8D);
    rfid_write_reg(0x2B, 0x3E);
    rfid_write_reg(0x2C, 0x30);
    rfid_write_reg(0x2D, 0x00);
    rfid_write_reg(0x15, 0x40);
    rfid_write_reg(0x11, 0x3D);
    rfid_antenna_on();
    ESP_LOGI(TAG, "RFID RC522 initialized on SPI3");
}

static bool rfid_check_card(char *uid_str)
{
    uint8_t status;
    uint8_t buffer[18];
    uint8_t len;

    rfid_write_reg(0x0E, 0x7F);
    rfid_write_reg(0x09, 0x20);

    buffer[0] = 0x26;
    rfid_write_reg(0x0A, 1);
    rfid_write_reg(0x09, buffer[0]);
    rfid_write_reg(0x07, 0x0C);
    rfid_set_bitmask(0x08, 0x80);

    int wait = 200;
    while (wait--)
    {
        status = rfid_read_reg(0x04);
        if (status & 0x01) break;
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    if (wait <= 0) return false;

    rfid_write_reg(0x0E, 0x7F);
    rfid_write_reg(0x09, 0x20);
    buffer[0] = 0x93;
    buffer[1] = 0x20;
    rfid_write_reg(0x0A, 2);
    for(int i = 0; i < 2; i++) rfid_write_reg(0x09, buffer[i]);
    rfid_write_reg(0x07, 0x0C);
    rfid_set_bitmask(0x08, 0x80);

    wait = 200;
    while (wait--)
    {
        status = rfid_read_reg(0x04);
        if (status & 0x01) break;
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    if (wait <= 0) return false;

    len = rfid_read_reg(0x1A);
    if (len < 5) return false;

    uint8_t raw_uid[5];
    for (int i = 0; i < 5; i++)
    {
        raw_uid[i] = rfid_read_reg(0x09);
    }

    snprintf(uid_str, 16, "%02X%02X%02X%02X", raw_uid[0], raw_uid[1], raw_uid[2], raw_uid[3]);
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
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
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
    const int retry_count = 15;

    while (timeinfo.tm_year < (2025 - 1900) && ++retry < retry_count)
    {
        ESP_LOGI(TAG, "Waiting for time... (%d/%d)", retry, retry_count);
        vTaskDelay(pdMS_TO_TICKS(2000));
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    setenv("TZ", "UTC-3", 1);
    tzset();
    ESP_LOGI(TAG, "Time synchronized");
}

/* ===================== TIME ===================== */
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
static void servo_set_angle(int angle)
{
    uint32_t duty = 1638 + (angle * (8191 - 1638)) / 180;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void door_open(void)
{
    servo_set_angle(90);
    gpio_set_level(PIN_LED_GREEN, 1);
    gpio_set_level(PIN_LED_RED, 0);
    gpio_set_level(PIN_BUZZER, 1);
    vTaskDelay(pdMS_TO_TICKS(80));
    gpio_set_level(PIN_BUZZER, 0);

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_state.door_open = true;
    xSemaphoreGive(g_state_mutex);

    vTaskDelay(pdMS_TO_TICKS(DOOR_OPEN_MS));

    servo_set_angle(0);
    gpio_set_level(PIN_LED_GREEN, 0);

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_state.door_open = false;
    xSemaphoreGive(g_state_mutex);
}

static void access_denied(void)
{
    gpio_set_level(PIN_LED_RED, 1);
    for (int i = 0; i < 3; i++)
    {
        gpio_set_level(PIN_BUZZER, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(PIN_BUZZER, 0);
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
            ESP_LOGI(TAG, "Event written to SD card");
        }
        else
        {
            ESP_LOGW(TAG, "Failed to open access_log.txt on SD");
        }
    }
}

/* ===================== ACCESS LOGIC ===================== */
static void process_entry(const char *uid, bool valid_card)
{
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);

    if (!is_within_schedule())
    {
        xSemaphoreGive(g_state_mutex);
        log_event(uid, "IN", "DENIED_SCHEDULE");
        access_denied();
        return;
    }
    if (!valid_card)
    {
        xSemaphoreGive(g_state_mutex);
        log_event(uid, "IN", "DENIED_INVALID");
        access_denied();
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
        access_denied();
        return;
    }

    g_state.occupancy++;
    if (idx >= 0)
        g_users[idx].inside = true;
    xSemaphoreGive(g_state_mutex);

    log_event(uid, "IN", "GRANTED");
    door_open();
}

static void process_exit(const char *uid)
{
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);

    if (!is_within_schedule() && g_state.occupancy > 0)
    {
        xSemaphoreGive(g_state_mutex);
        log_event(uid, "OUT", "DENIED_SCHEDULE");
        access_denied();
        return;
    }

    if (g_state.occupancy > 0)
        g_state.occupancy--;
    for (int i = 0; i < g_user_count; i++)
    {
        if (strcmp(g_users[i].uid, uid) == 0)
        {
            g_users[i].inside = false;
            break;
        }
    }
    xSemaphoreGive(g_state_mutex);

    log_event(uid, "OUT", "GRANTED");
    door_open();
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
        bool door = g_state.door_open;
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
        snprintf(line, sizeof(line), "TIME %02d:%02d:%02d", h, m, s);
        oled_draw_string(0, 10, line);
        snprintf(line, sizeof(line), "OCC:%ld  DOOR:%s", (long)occ, door ? "OPEN" : "CLSD");
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

/* ===================== ACCESS & RFID TASKS ===================== */
static void access_button_task(void *arg)
{
    bool last_ok = true, last_bad = true, last_exit = true;
    TickType_t last_press = 0;

    while (1)
    {
        bool ok = gpio_get_level(PIN_BTN_ENTRY_OK) == 0;
        bool bad = gpio_get_level(PIN_BTN_ENTRY_BAD) == 0;
        bool exitb = gpio_get_level(PIN_BTN_EXIT) == 0;
        TickType_t now = xTaskGetTickCount();

        if (ok && !last_ok && (now - last_press > pdMS_TO_TICKS(DEBOUNCE_MS)))
        {
            last_press = now;
            process_entry("A1B2C3D4", true);
        }
        if (bad && !last_bad && (now - last_press > pdMS_TO_TICKS(DEBOUNCE_MS)))
        {
            last_press = now;
            process_entry("DEADBEEF", false);
        }
        if (exitb && !last_exit && (now - last_press > pdMS_TO_TICKS(DEBOUNCE_MS)))
        {
            last_press = now;
            process_exit("A1B2C3D4");
        }

        char rfid_uid[16];
        if (rfid_check_card(rfid_uid))
        {
            ESP_LOGI(TAG, "RFID Card Scanned! UID: %s", rfid_uid);
            bool registered = false;
            for (int i = 0; i < g_user_count; i++)
            {
                if (strcmp(g_users[i].uid, rfid_uid) == 0)
                {
                    registered = true;
                    break;
                }
            }
            process_entry(rfid_uid, registered);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        last_ok = ok;
        last_bad = bad;
        last_exit = exitb;
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
                ESP_LOGI(TAG, "Room 1 lights ON");
            }
        }
        else if (g_state.room1_light && (now - last_motion1 > LIGHT_TIMEOUT_MS))
        {
            g_state.room1_occupied = false;
            g_state.room1_light = false;
            gpio_set_level(PIN_LED_ROOM1, 0);
            ESP_LOGI(TAG, "Room 1 lights OFF");
        }

        if (pir2)
        {
            last_motion2 = now;
            g_state.room2_occupied = true;
            if (!g_state.room2_light)
            {
                g_state.room2_light = true;
                gpio_set_level(PIN_LED_ROOM2, 1);
                ESP_LOGI(TAG, "Room 2 lights ON");
            }
        }
        else if (g_state.room2_light && (now - last_motion2 > LIGHT_TIMEOUT_MS))
        {
            g_state.room2_occupied = false;
            g_state.room2_light = false;
            gpio_set_level(PIN_LED_ROOM2, 0);
            ESP_LOGI(TAG, "Room 2 lights OFF");
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
    printf("0. Logout\n");
    printf("===========================================\n");
    printf("Choice: ");
    fflush(stdout);
}

static void serial_menu_task(void *arg)
{
    char line[64];
    int idx = 0;

    printf("\n\n*** CoCIS Smart Control System ***\n");
    printf("Type password to login.\n");
    printf("Default password: %s\n\nPassword: ", MENU_PASSWORD);
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

            if (strcmp(line, "0") == 0)
            {
                g_state.authenticated = false;
                printf("Logged out.\nPassword: ");
                fflush(stdout);
            }
            else if (strcmp(line, "1") == 0)
            {
                xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                printf("Building occupancy: %ld people\n", (long)g_state.occupancy);
                xSemaphoreGive(g_state_mutex);
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
                print_menu();
            }
            else if (strcmp(line, "5") == 0)
            {
                xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                g_state.room1_light = !g_state.room1_light;
                gpio_set_level(PIN_LED_ROOM1, g_state.room1_light ? 1 : 0);
                printf("Room 1 light forced %s\n", g_state.room1_light ? "ON" : "OFF");
                xSemaphoreGive(g_state_mutex);
                print_menu();
            }
            else if (strcmp(line, "6") == 0)
            {
                xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                g_state.room2_light = !g_state.room2_light;
                gpio_set_level(PIN_LED_ROOM2, g_state.room2_light ? 1 : 0);
                printf("Room 2 light forced %s\n", g_state.room2_light ? "ON" : "OFF");
                xSemaphoreGive(g_state_mutex);
                print_menu();
            }
            else if (strcmp(line, "7") == 0)
            {
                g_time_manual_offset_sec += 3600;
                int h, m, s;
                get_current_time(&h, &m, &s);
                printf("Time advanced → now %02d:%02d:%02d\n", h, m, s);
                print_menu();
            }
            else if (strcmp(line, "8") == 0)
            {
                xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                printf("--- System Status ---\n");
                printf("Occupancy    : %ld\n", (long)g_state.occupancy);
                printf("Door         : %s\n", g_state.door_open ? "OPEN" : "CLOSED");
                printf("Schedule OK  : %s\n", is_within_schedule() ? "YES" : "NO");
                printf("SD Card      : %s\n", g_sd_ok ? "OK" : "FAIL");
                printf("Free heap    : %lu\n", (unsigned long)esp_get_free_heap_size());
                xSemaphoreGive(g_state_mutex);
                print_menu();
            }
            else
            {
                printf("Unknown command\n");
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
    gpio_config_t out;
    memset(&out, 0, sizeof(out));
    out.pin_bit_mask = (1ULL << PIN_LED_GREEN) | (1ULL << PIN_LED_RED) |
                       (1ULL << PIN_LED_ROOM1) | (1ULL << PIN_LED_ROOM2) |
                       (1ULL << PIN_BUZZER);
    out.mode = GPIO_MODE_OUTPUT;
    gpio_config(&out);

    gpio_config_t btn;
    memset(&btn, 0, sizeof(btn));
    btn.pin_bit_mask = (1ULL << PIN_BTN_ENTRY_OK) | (1ULL << PIN_BTN_ENTRY_BAD) |
                       (1ULL << PIN_BTN_EXIT);
    btn.mode = GPIO_MODE_INPUT;
    btn.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&btn);

    gpio_config_t pir;
    memset(&pir, 0, sizeof(pir));
    pir.pin_bit_mask = (1ULL << PIN_PIR1) | (1ULL << PIN_PIR2);
    pir.mode = GPIO_MODE_INPUT;
    gpio_config(&pir);

    ledc_timer_config_t tcfg;
    memset(&tcfg, 0, sizeof(tcfg));
    tcfg.speed_mode = LEDC_LOW_SPEED_MODE;
    tcfg.duty_resolution = LEDC_TIMER_13_BIT;
    tcfg.timer_num = LEDC_TIMER_0;
    tcfg.freq_hz = 50;
    tcfg.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&tcfg);

    ledc_channel_config_t ccfg;
    memset(&ccfg, 0, sizeof(ccfg));
    ccfg.gpio_num = PIN_SERVO;
    ccfg.speed_mode = LEDC_LOW_SPEED_MODE;
    ccfg.channel = LEDC_CHANNEL_0;
    ccfg.timer_sel = LEDC_TIMER_0;
    ccfg.duty = 0;
    ccfg.hpoint = 0;
    ledc_channel_config(&ccfg);

    servo_set_angle(0);
}

static void init_users(void)
{
    strcpy(g_users[0].uid, "A1B2C3D4");
    g_users[0].inside = false;
    g_user_count = 1;
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting CoCIS Smart Control System (Pure C) with separate SPI buses...");

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
    g_state.door_open = false;

    load_schedule_from_nvs();
    init_users();

    g_state_mutex = xSemaphoreCreateMutex();
    gpio_init_all();
    oled_init();

    // Independent Peripherals Initialization
    g_sd_ok = sd_init();
    if (g_sd_ok)
    {
        ESP_LOGI(TAG, "SD card ready on SPI2 – access events will be logged");
    }
    else
    {
        ESP_LOGW(TAG, "SD card not available");
    }

    rfid_init();

    xTaskCreate(access_button_task, "access", 4096, NULL, 5, NULL);
    xTaskCreate(lighting_task, "lights", 4096, NULL, 4, NULL);
    xTaskCreate(serial_menu_task, "menu", 4096, NULL, 3, NULL);
    xTaskCreate(display_task, "display", 4096, NULL, 3, NULL);

    ESP_LOGI(TAG, "System ready. Password: %s", MENU_PASSWORD);
}