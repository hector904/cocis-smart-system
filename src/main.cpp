/*
 * CoCIS Smart Control and Monitoring System 
 * ESP-IDF + Wokwi
 * MADETE HECTOR 24/U/25933/EVE 2400725933
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_timer.h"

static const char *TAG = "CoCIS";

/* ===================== PINS ===================== */
#define PIN_SERVO ((gpio_num_t)13)
#define PIN_LED_GREEN ((gpio_num_t)25)
#define PIN_LED_RED ((gpio_num_t)26)
#define PIN_LED_ROOM1 ((gpio_num_t)27)
#define PIN_LED_ROOM2 ((gpio_num_t)14)
#define PIN_BUZZER ((gpio_num_t)32)
#define PIN_PIR1 ((gpio_num_t)34)
#define PIN_PIR2 ((gpio_num_t)35)
#define PIN_BTN_ENTRY_OK ((gpio_num_t)18)
#define PIN_BTN_ENTRY_BAD ((gpio_num_t)19)
#define PIN_BTN_EXIT ((gpio_num_t)21)

/* ===================== CONSTANTS ===================== */
#define DOOR_OPEN_MS 2500
#define LIGHT_TIMEOUT_MS 8000
#define MENU_PASSWORD "cocis2026"
#define MAX_USERS 8
#define DEBOUNCE_MS 250

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
static int64_t g_time_offset_sec = 8 * 3600; // start at 08:00

/* ===================== TIME ===================== */
static void get_current_time(int *hour, int *min, int *sec)
{
    int64_t now = (esp_timer_get_time() / 1000000LL) + g_time_offset_sec;
    int total_sec = (int)(now % 86400);
    if (total_sec < 0)
        total_sec += 86400;
    *hour = total_sec / 3600;
    *min = (total_sec % 3600) / 60;
    *sec = total_sec % 60;
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

/* ===================== TASKS ===================== */
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

        last_ok = ok;
        last_bad = bad;
        last_exit = exitb;
        vTaskDelay(pdMS_TO_TICKS(30));
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
                g_time_offset_sec += 3600;
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

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting CoCIS Smart Control System...");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // Default schedule 08:00 - 18:00
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

    xTaskCreate(access_button_task, "access", 4096, NULL, 5, NULL);
    xTaskCreate(lighting_task, "lights", 4096, NULL, 4, NULL);
    xTaskCreate(serial_menu_task, "menu", 4096, NULL, 3, NULL);

    ESP_LOGI(TAG, "System ready. Password: %s", MENU_PASSWORD);
}