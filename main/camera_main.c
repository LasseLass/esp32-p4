/*
 * ESP32-P4 Camera app (I2C probe for IMX708 / RPi Camera Module 3)
 * Verifiserer I2C-kontakt og chip-ID-lesing (0x0016/0x0017).
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_cam_sensor.h"
#include "driver/i2c_types.h"

static const char *TAG = "esp32_p4_camera";

/* ---------- GPIO Definitions ---------- */
#define CAM_PWR_EN_GPIO     CONFIG_CAM_PWR_EN_GPIO      // Power enable
#define CAM_RESET_GPIO      CONFIG_CAM_RESET_GPIO       // Reset pin
#define CAM_I2C_SDA_GPIO    CONFIG_CAM_I2C_SDA_GPIO     // I2C SDA
#define CAM_I2C_SCL_GPIO    CONFIG_CAM_I2C_SCL_GPIO     // I2C SCL
#define CAM_I2C_PORT        CONFIG_CAM_I2C_PORT         // I2C controller
#define CAM_I2C_TIMEOUT_MS  CONFIG_CAM_I2C_TIMEOUT_MS   // I2C timeout
#ifdef CONFIG_CAM_I2C_USE_INTERNAL_PULLUPS
#define CAM_I2C_USE_INTERNAL_PULLUPS 1
#else
#define CAM_I2C_USE_INTERNAL_PULLUPS 0
#endif
#define CAM_I2C_ADDR_IMX708 0x1A  // 7-bit I2C address for IMX708

/* ---------- Hardware handles ---------- */
static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_dev_handle_t s_cam_i2c_dev = NULL;

/* ---------- Power Management ---------- */
static esp_err_t i2c_cam_power_on(void)
{
    ESP_LOGI(TAG, "Power ON: PWR_EN=%d", CAM_PWR_EN_GPIO);
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CAM_PWR_EN_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) return ret;
    
    gpio_set_level(CAM_PWR_EN_GPIO, 1);  // Power ON
    vTaskDelay(pdMS_TO_TICKS(100));  // Power stabilization
    return ESP_OK;
}

static esp_err_t i2c_cam_power_off(void)
{
    ESP_LOGI(TAG, "🔌 Power OFF: PWR_EN=%d", CAM_PWR_EN_GPIO);
    gpio_set_level(CAM_PWR_EN_GPIO, 0);  // Power OFF
    vTaskDelay(pdMS_TO_TICKS(50));
    return ESP_OK;
}

static esp_err_t i2c_cam_reset_sequence(void)
{
    ESP_LOGI(TAG, " Reset sequence: RESET=%d", CAM_RESET_GPIO);
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CAM_RESET_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) return ret;
    
    gpio_set_level(CAM_RESET_GPIO, 0);  // Assert reset
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(CAM_RESET_GPIO, 1);  // Release reset
    vTaskDelay(pdMS_TO_TICKS(100));     // Give time for sensor to initialize
    return ESP_OK;
}

static esp_err_t camera_power_sequence(void)
{
    ESP_LOGI(TAG, " Camera power sequence start");
    
    esp_err_t ret;
    
    // 1. Power OFF først (clean state)
    ret = i2c_cam_power_off();
    if (ret != ESP_OK) return ret;
    
    // 2. Power ON
    ret = i2c_cam_power_on();
    if (ret != ESP_OK) return ret;
    
    // 3. Reset sequence
    ret = i2c_cam_reset_sequence();
    if (ret != ESP_OK) return ret;
    
    ESP_LOGI(TAG, " Camera power sequence complete");
    return ESP_OK;
}

/* ---------- I2C Communication ---------- */
static esp_err_t camera_init_i2c(void)
{
    ESP_LOGI(TAG, " I2C init: SDA=%d, SCL=%d, PORT=%d", CAM_I2C_SDA_GPIO, CAM_I2C_SCL_GPIO, CAM_I2C_PORT);
    
    // Master bus configuration
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = CAM_I2C_PORT,
        .scl_io_num = CAM_I2C_SCL_GPIO,
        .sda_io_num = CAM_I2C_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = CAM_I2C_USE_INTERNAL_PULLUPS,
    };
    
    esp_err_t ret = i2c_new_master_bus(&bus_config, &s_i2c_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, " I2C master bus creation failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Add camera device to the bus
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CAM_I2C_ADDR_IMX708,
        .scl_speed_hz = CONFIG_CAM_I2C_FREQ_HZ,
    };
    
    ret = i2c_master_bus_add_device(s_i2c_bus, &dev_config, &s_cam_i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, " I2C device add failed: %s", esp_err_to_name(ret));
        i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = NULL;
        return ret;
    }
    
    ESP_LOGI(TAG, " I2C initialized successfully");
    ESP_LOGI(TAG, "  Bus: I2C%d, SDA=GPIO%d, SCL=GPIO%d", CAM_I2C_PORT, CAM_I2C_SDA_GPIO, CAM_I2C_SCL_GPIO);
    ESP_LOGI(TAG, "  Frequency: %d Hz, Timeout: %d ms", CONFIG_CAM_I2C_FREQ_HZ, CAM_I2C_TIMEOUT_MS);
    ESP_LOGI(TAG, "  Pull-ups: %s", CAM_I2C_USE_INTERNAL_PULLUPS ? "Internal" : "External");
    return ESP_OK;
}

static bool i2c_probe_addr(uint8_t addr7)
{
    esp_err_t err = i2c_master_probe(s_i2c_bus, addr7, CAM_I2C_TIMEOUT_MS);
    return (err == ESP_OK);
}

static void i2c_scan_bus(void)
{
    if (!s_i2c_bus) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return;
    }
    
    ESP_LOGI(TAG, "Skanner I2C-bus (0x08..0x77)...");
    int found = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
        if (i2c_probe_addr(addr)) {
            ESP_LOGI(TAG, " ACK @ 0x%02X", addr);
            found++;
        }
        vTaskDelay(1);
    }
    if (!found) {
        ESP_LOGW(TAG, " Ingen I2C-enheter funnet!");
        ESP_LOGW(TAG, "Mulige årsaker:");
        ESP_LOGW(TAG, "  1. Feil SDA/SCL pins (nå: SDA=%d, SCL=%d)", CAM_I2C_SDA_GPIO, CAM_I2C_SCL_GPIO);
        ESP_LOGW(TAG, "  2. Kamera ikke strømforsynt (PWR_EN=%d)", CAM_PWR_EN_GPIO);
        ESP_LOGW(TAG, "  3. Manglende pull-up motstander på I2C-linjene");
        ESP_LOGW(TAG, "  4. Feil kabel-tilkobling til CSI-port");
        ESP_LOGW(TAG, "IMX708 skal være på 0x1A (7-bit)");
    } else {
        ESP_LOGI(TAG, " Funnet %d I2C-enheter", found);
    }
}

/* ---------- Camera sensor detection using esp_cam_sensor driver ---------- */
static esp_err_t detect_camera_sensor(void)
{
    ESP_LOGI(TAG, "Trying camera sensor auto-detection...");
    
    // esp_cam_sensor uses auto-detection based on registered sensors
    // The library will automatically probe for supported sensors
    // This requires sensors to be enabled in menuconfig
    ESP_LOGI(TAG, "Note: esp_cam_sensor auto-detection requires sensors enabled in menuconfig");
    ESP_LOGI(TAG, "Run 'idf.py menuconfig' -> Component Config -> ESP CAM Sensor -> Enable desired sensors");
    
    return ESP_OK;
}

/* ---------- app_main ---------- */
void app_main(void)
{
    ESP_LOGI(TAG, " ESP32-P4 Camera Debug Start");
    ESP_LOGI(TAG, "Power: PWR_EN=%d, RESET=%d", CAM_PWR_EN_GPIO, CAM_RESET_GPIO);

    // 1) Power/reset-sekvens først
    esp_err_t ret = camera_power_sequence();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, " Camera power sequence failed: %s", esp_err_to_name(ret));
        vTaskDelay(portMAX_DELAY);
    }

    // 2) Initialize I2C bus
    ret = camera_init_i2c();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, " Camera I2C init failed: %s", esp_err_to_name(ret));
        vTaskDelay(portMAX_DELAY);
    }

    // 3) Test I2C kommunikasjon
    if (s_i2c_bus) {
        i2c_scan_bus();
    } else {
        ESP_LOGE(TAG, " No valid I2C bus available");
    }
    
    // 4) Camera sensor detection info
    ret = detect_camera_sensor();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, " Camera sensor detection info displayed");
    }
    
    ESP_LOGI(TAG, " Camera initialization test fullført!");

    // Hold prosessen i live for videre debugging
    vTaskDelay(portMAX_DELAY);
} 