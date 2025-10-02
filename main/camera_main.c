/*
 * ESP32-P4 Camera Application for IMX708 / RPi Camera Module 3
 *
 * All hardware-related pins, frequencies and options should be configured via menuconfig
 * wherever possible. This file avoids hard-coding values that are provided by sdkconfig.
 */
// C / POSIX
#include <stdio.h>          // printf/logg-format
#include <string.h>         // memcpy, strcmp, memset, strerror, osv.

// FreeRTOS / ESP-IDF
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "driver/gpio.h"

// ESP Video framework
#include "esp_video_init.h"     // init/konfig av CSI
#include "esp_video_device.h"   // ESP_VIDEO_MIPI_CSI_DEVICE_NAME

// Kamera/sensor-API
#include "esp_cam_sensor.h"     // esp_cam_sensor_format_t, typer
#include "esp_cam_sensor_xclk.h" // ESP Camera Sensor XCLK API

// I2C Master API
#include "driver/i2c_master.h"  // i2c_master_bus_config_t, i2c_new_master_bus
#include "esp_cam_sensor_types.h"  // ESP_CAM_SENSOR_IOC_S_STREAM

// ESP32-P4 LDO Regulator API for CSI power
#include "esp_ldo_regulator.h"     // esp_ldo_acquire_channel for 1.8V CSI rail
#include "driver/i2c_types.h"   // I2C_ADDR_BIT_LEN_7

// Header files for ESP32-P4 camera application components

static const char *TAG = "esp32_p4_camera";

// --- Forward Declarations (fix implicit declarations) ---
// Ensure compiler knows these functions before first use
static inline void cam_power(bool on);
static inline void cam_reset_assert(void);
static inline void cam_reset_release(void);
esp_cam_sensor_device_t *esp_video_get_csi_video_device_sensor(void);

/* ---------- Global Handles ---------- */
// NOTE: ESP Video framework now owns I²C/SCCB handles
// static i2c_master_bus_handle_t s_i2c_bus_handle = NULL;  // REMOVED: ESP Video owns this
// static esp_sccb_io_handle_t s_sccb_handle = NULL;        // REMOVED: ESP Video owns this
static esp_cam_sensor_device_t *s_cam_sensor_dev = NULL;
static esp_cam_sensor_xclk_handle_t s_xclk_handle = NULL;
static esp_ldo_channel_handle_t s_ldo_handle = NULL;  // ESP32-P4 LDO for 2.5V CSI rail

/*
 * ESP32-P4 LDO Functions for CSI Power Management
 * 
 * FireBeetle 2 ESP32-P4 requires internal LDO activation for CSI I2C communication
 * VO3 = 2.5V domain for CSI-CCI/I2C (matches DFRobot reference design)
 */


// NOTE: i2c_bus_recovery function removed to eliminate unused function warning
// All I²C recovery is now handled in the diagnosis phase before ESP Video init


static esp_err_t enable_csi_ldo_power(void)
{
    ESP_LOGI(TAG, "Enabling ESP32-P4 LDO for 2.5V CSI rail (VO3)");
    if (s_ldo_handle) {
        ESP_LOGI(TAG, "LDO already enabled (VO3)");
        return ESP_OK;
    }
    
    esp_ldo_channel_config_t ldo_config = {
        .chan_id = 3,           // VO3 = 2.5V domain for CSI I2C (matches DFRobot example)
        .voltage_mv = 2500,     // 2.5V for CSI-CCI/I2C logic level
    };
    
    esp_err_t ret = esp_ldo_acquire_channel(&ldo_config, &s_ldo_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to acquire LDO channel 3 (2.5V CSI): %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "✓ LDO VO3 enabled: 2.5V for CSI I2C (matching DFRobot reference design)");
    return ESP_OK;
}

// Cleanup function for LDO - currently unused but kept for future cleanup
// static void disable_csi_ldo_power(void)
// {
//     if (s_ldo_handle) {
//         esp_ldo_release_channel(s_ldo_handle);
//         s_ldo_handle = NULL;
//         ESP_LOGI(TAG, "✓ LDO VO4 disabled");
//     }
// }

/* ---------- GPIO Definitions ---------- 
 * GPIO pin definitions for ESP32-P4 to camera connections.
 * These can be configured through menuconfig.
 */
#define CAM_PWR_EN_GPIO     CONFIG_CAM_PWR_EN_GPIO      // Power enable
#define CAM_RESET_GPIO      CONFIG_CAM_RESET_GPIO       // Reset pin
#define CAM_I2C_SDA_GPIO    CONFIG_CAM_I2C_SDA_GPIO     // I2C SDA pin from CSI port
#define CAM_I2C_SCL_GPIO    CONFIG_CAM_I2C_SCL_GPIO     // I2C SCL pin from CSI port
#define CAM_I2C_PORT        CONFIG_CAM_I2C_PORT         // I2C controller

/* ---------- RPi Camera Module 3 Specific GPIO ---------- 
 * ENABLE pin: Critical power enable for RPi Camera Module 3
 * Controls LDO (1.8V, 2.8V), 1.1V DC/DC, and 24MHz oscillator
 * MUST be HIGH before I2C communication
 * Source: Raspberry Pi Camera Module 3 schematics
 */
#ifdef CONFIG_CAM_ENABLE_GPIO
#define CAM_ENABLE_GPIO     CONFIG_CAM_ENABLE_GPIO
#else
#define CAM_ENABLE_GPIO     (-1)
#endif

/* ---------- XCLK Configuration ---------- 
 * RPi Camera Module 3 has on-board 24MHz oscillator
 * External XCLK generation should be DISABLED (-1)
 * Source: Raspberry Pi Camera Module 3 schematics (datasheets.raspberrypi.com)
 */
#ifdef CONFIG_CAM_XCLK_GPIO
#define CAM_XCLK_GPIO       CONFIG_CAM_XCLK_GPIO
#else
#define CAM_XCLK_GPIO       (-1)
#endif

#ifdef CONFIG_CAM_XCLK_FREQ_HZ
#define CAM_XCLK_FREQ_HZ    CONFIG_CAM_XCLK_FREQ_HZ
#else
#define CAM_XCLK_FREQ_HZ    24000000
#endif

/* ---------- Power Helper (PWDN/CAM_EN active level handling) ---------- */
static inline void cam_power(bool on)
{
#if CONFIG_CAM_PWR_EN_ACTIVE_HIGH
    gpio_set_level(CAM_PWR_EN_GPIO, on ? 1 : 0);
#else
    // Active LOW: drive LOW to enable power
    gpio_set_level(CAM_PWR_EN_GPIO, on ? 0 : 1);
#endif
}

/* ---------- Reset Helper (polarity aware) ---------- */
static inline void cam_reset_assert(void)
{
    if (CAM_RESET_GPIO < 0) {
        return;
    }
#if CONFIG_CAM_RESET_ACTIVE_LOW
    gpio_set_level(CAM_RESET_GPIO, 0);
#else
    gpio_set_level(CAM_RESET_GPIO, 1);
#endif
}

static inline void cam_reset_release(void)
{
    if (CAM_RESET_GPIO < 0) {
        return;
    }
#if CONFIG_CAM_RESET_ACTIVE_LOW
    gpio_set_level(CAM_RESET_GPIO, 1);
#else
    gpio_set_level(CAM_RESET_GPIO, 0);
#endif
}

static inline const char *cam_reset_semantic_str(void)
{
    if (CAM_RESET_GPIO < 0) {
        return "N/A";
    }
    int lvl = gpio_get_level(CAM_RESET_GPIO);
#if CONFIG_CAM_RESET_ACTIVE_LOW
    return lvl ? "DEASSERTED" : "ASSERTED";
#else
    return lvl ? "ASSERTED" : "DEASSERTED";
#endif
}

/* Reserved: frame size macros are unused; format is selected from sdkconfig */

/* ---------- Working power sequence ---------- */
/*
 * camera_power_sequence()
 * 
 * Implements the camera power-up sequence required for proper initialization.
 * 
 * The sequence follows these steps:
 * 1. Configure GPIO pins for PWDN and RESET
 * 2. Set PWDN high (inactive)
 * 3. Set RESET low (active), wait 20ms
 * 4. Set RESET high (inactive), wait 20ms
 * 
 * This timing is critical for proper sensor initialization.
 */
static esp_err_t camera_power_sequence_with_xclk(void)
{
    ESP_LOGI(TAG, "Camera power sequence (ENABLE + 2.5V LDO + PWDN/RESET + optional XCLK)");
    ESP_RETURN_ON_ERROR(enable_csi_ldo_power(), TAG, "ldo");

    // Configure only valid pins
    uint64_t mask = 0;
    if (CAM_PWR_EN_GPIO >= 0) mask |= (1ULL << (uint32_t)CAM_PWR_EN_GPIO);
    if (CAM_RESET_GPIO  >= 0) mask |= (1ULL << (uint32_t)CAM_RESET_GPIO);
    if (CAM_ENABLE_GPIO >= 0) mask |= (1ULL << (uint32_t)CAM_ENABLE_GPIO);
    if (mask) {
        gpio_config_t io = {
            .pin_bit_mask = mask,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "gpio");
    }

    // CRITICAL: Enable camera module power rails FIRST (RPi Camera Module 3)
    // ENABLE pin controls LDO (1.8V, 2.8V), 1.1V DC/DC, and oscillator
    // Must be HIGH before I2C communication
    // Source: Raspberry Pi Camera Module 3 schematics
    if (CAM_ENABLE_GPIO >= 0) {
#ifdef CONFIG_CAM_ENABLE_ACTIVE_HIGH
        gpio_set_level(CAM_ENABLE_GPIO, 1);
        ESP_LOGI(TAG, "ENABLE pin=%d set HIGH (active-HIGH: LDO + oscillator ON)", CAM_ENABLE_GPIO);
#else
        gpio_set_level(CAM_ENABLE_GPIO, 0);
        ESP_LOGI(TAG, "ENABLE pin=%d set LOW (active-LOW: LDO + oscillator ON)", CAM_ENABLE_GPIO);
#endif
        vTaskDelay(pdMS_TO_TICKS(10)); // Wait for power rails to stabilize
    }

    // Apply power and hold RESET asserted using configured polarity
    if (CAM_PWR_EN_GPIO >= 0) cam_power(true);
    if (CAM_RESET_GPIO  >= 0) cam_reset_assert();
    vTaskDelay(pdMS_TO_TICKS(10));

    // Start XCLK only if configured (RPi Camera Module 3 has on-board oscillator)
    // Source: Raspberry Pi Camera Module 3 has 24MHz oscillator, external XCLK not needed
    if (CAM_XCLK_GPIO >= 0) {
        esp_cam_sensor_xclk_config_t xcfg = { .esp_clock_router_cfg = { .xclk_pin = CAM_XCLK_GPIO, .xclk_freq_hz = CAM_XCLK_FREQ_HZ } };
        ESP_RETURN_ON_ERROR(esp_cam_sensor_xclk_allocate(ESP_CAM_SENSOR_XCLK_ESP_CLOCK_ROUTER, &s_xclk_handle), TAG, "xclk_alloc");
        ESP_RETURN_ON_ERROR(esp_cam_sensor_xclk_start(s_xclk_handle, &xcfg), TAG, "xclk_start");
        ESP_LOGI(TAG, "XCLK started: GPIO%d @ %d Hz", CAM_XCLK_GPIO, CAM_XCLK_FREQ_HZ);
    } else {
        ESP_LOGI(TAG, "XCLK disabled (using camera module's on-board oscillator)");
    }

    // Release RESET using configured polarity and wait for stabilization
    if (CAM_RESET_GPIO >= 0) cam_reset_release();
    vTaskDelay(pdMS_TO_TICKS(150));
    int pwr_lvl = (CAM_PWR_EN_GPIO >= 0) ? gpio_get_level(CAM_PWR_EN_GPIO) : -1;
    int rst_lvl = (CAM_RESET_GPIO  >= 0) ? gpio_get_level(CAM_RESET_GPIO)  : -1;
    ESP_LOGI(TAG, "PWR_EN=%s (level=%s), RESET now %s (level=%s)",
             (pwr_lvl < 0) ? "N/A" : (
#if CONFIG_CAM_PWR_EN_ACTIVE_HIGH
             pwr_lvl ? "ON" : "OFF"
#else
             pwr_lvl ? "OFF" : "ON"
#endif
             ),
             (pwr_lvl < 0) ? "N/A" : (pwr_lvl ? "HIGH" : "LOW"),
             (rst_lvl < 0) ? "N/A" : (
#if CONFIG_CAM_RESET_ACTIVE_LOW
             rst_lvl ? "DEASSERTED" : "ASSERTED"
#else
             rst_lvl ? "ASSERTED" : "DEASSERTED"
#endif
             ),
             (rst_lvl < 0) ? "N/A" : (rst_lvl ? "HIGH" : "LOW"));

    bool rst_asserted = false;
#if CONFIG_CAM_RESET_ACTIVE_LOW
    rst_asserted = (rst_lvl == 0);
    bool deassert_level = 1;
#else
    rst_asserted = (rst_lvl == 1);
    bool deassert_level = 0;
#endif
    if (rst_asserted) {
        ESP_LOGW(TAG, "RESET line appears ASSERTED after release. Trying opposite-polarity fallback once...");
        // Drive opposite of the configured deassert level
        if (CAM_RESET_GPIO >= 0) gpio_set_level(CAM_RESET_GPIO, !deassert_level);
        vTaskDelay(pdMS_TO_TICKS(150));
        rst_lvl = (CAM_RESET_GPIO  >= 0) ? gpio_get_level(CAM_RESET_GPIO)  : -1;
        ESP_LOGW(TAG, "After fallback: RESET level=%s (this may indicate opposite polarity on hardware)",
                 (rst_lvl < 0) ? "N/A" : (rst_lvl ? "HIGH" : "LOW"));
        if (rst_lvl == 0) {
            ESP_LOGW(TAG, "RESET still asserted. Check wiring and CAM_RESET_GPIO mapping.");
        }
    }
    return ESP_OK;
}

// Camera power sequence implementation

/* ---------- Frame Buffer Management ---------- 
 * Double buffering implementation for camera frame data.
 *
 * - Two frame buffers (153600 bytes each) for 320x240 YUYV format
 * - DMA-capable memory allocation for direct hardware access
 * - Buffer rotation for continuous frame processing
 * 
 * Memory calculation: 320 × 240 × 2 = 153600 bytes per frame
 */

/* ---------- Minimal helper: I2C + esp_video ---------- */
static esp_err_t imx708_read_u8(i2c_master_bus_handle_t bus, uint8_t addr, uint16_t reg, uint8_t *out)
{
    i2c_master_dev_handle_t dev;
    i2c_device_config_t devcfg = {
        .device_address = addr,
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz = CONFIG_CAM_I2C_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &devcfg, &dev), TAG, "add dev");
    uint8_t w[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };
    esp_err_t err = i2c_master_transmit_receive(dev, w, 2, out, 1, pdMS_TO_TICKS(CONFIG_CAM_I2C_TIMEOUT_MS));
    i2c_master_bus_rm_device(dev);
    return err;
}

static esp_err_t probe_imx708_chip_id(i2c_master_bus_handle_t bus)
{
    uint8_t ph = 0, pl = 0;
    ESP_RETURN_ON_ERROR(imx708_read_u8(bus, 0x1A, 0x0016, &ph), TAG, "pid_h");
    ESP_RETURN_ON_ERROR(imx708_read_u8(bus, 0x1A, 0x0017, &pl), TAG, "pid_l");
    uint16_t id = ((uint16_t)ph << 8) | pl;
    ESP_LOGI(TAG, "IMX708 chip id: 0x%04X (expected 0x0708)", id);
    return (id == 0x0708) ? ESP_OK : ESP_FAIL;
}

static esp_err_t make_i2c_bus(i2c_master_bus_handle_t *out)
{
    i2c_master_bus_config_t c = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = CAM_I2C_PORT,
        .sda_io_num = CAM_I2C_SDA_GPIO,
        .scl_io_num = CAM_I2C_SCL_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup =
#if CONFIG_CAM_I2C_USE_INTERNAL_PULLUPS
            true
#else
            false
#endif
        ,
    };
    return i2c_new_master_bus(&c, out);
}

static esp_err_t init_csi_and_sensor_handle(i2c_master_bus_handle_t bus)
{
    esp_video_init_csi_config_t csi = {
        .sccb_config = {
            .init_sccb = false,
            .i2c_handle = bus,
            .freq = CONFIG_CAM_I2C_FREQ_HZ,
        },
        .reset_pin = CAM_RESET_GPIO,
        .pwdn_pin  = CAM_PWR_EN_GPIO,
    };
    esp_video_init_config_t vid = { .csi = &csi };
    ESP_RETURN_ON_ERROR(esp_video_init(&vid), TAG, "esp_video_init");
    ESP_LOGI(TAG, "esp_video_init OK");

    s_cam_sensor_dev = esp_video_get_csi_video_device_sensor();
    return s_cam_sensor_dev ? ESP_OK : ESP_FAIL;
}

static esp_err_t start_imx708_stream_from_kconfig(void)
{
    // Ensure clean state before applying format
    ESP_RETURN_ON_ERROR(esp_cam_sensor_ioctl(s_cam_sensor_dev, ESP_CAM_SENSOR_IOC_SW_RESET, NULL), TAG, "sw_reset");
    vTaskDelay(pdMS_TO_TICKS(5));
    // Use driver defaults from sdkconfig to avoid hard-coded width/height/pixformat
    ESP_RETURN_ON_ERROR(esp_cam_sensor_set_format(s_cam_sensor_dev, NULL), TAG, "set_fmt");
    int on = 1;
    ESP_RETURN_ON_ERROR(esp_cam_sensor_ioctl(s_cam_sensor_dev, ESP_CAM_SENSOR_IOC_S_STREAM, &on), TAG, "s_stream");
    ESP_LOGI(TAG, "IMX708 streaming started (format selected via sdkconfig).");
    return ESP_OK;
}

/*
 * start_sensor_and_stream()
 * 
 * Configure IMX708 sensor for RAW10 format and start streaming
 */
/* start_imx708_stream_720p_raw10() implemented above */

/* No capture tasks or fake frame processing */



/* ---------- app_main ---------- */
/*
 * app_main()
 * 
 * Main application entry point and system orchestration.
 * 
 * Initialization sequence:
 * 1. Camera power sequence initialization
 * 2. Camera configuration with manual settings
 * 3. System stabilization delay (2 seconds)
 * 4. Frame buffer allocation
 * 5. Native frame processing task creation
 * 6. System status monitoring
 * 
 * The application uses CSI Direct Mode with native ESP-IDF implementation
 * for efficient image processing.
 */
void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-P4 IMX708 bring-up");
    // Helpful logging for low-level drivers
    esp_log_level_set("esp_driver_cam", ESP_LOG_INFO);
    esp_log_level_set("esp_video", ESP_LOG_INFO);
    esp_log_level_set("esp_video_init", ESP_LOG_DEBUG);
    esp_log_level_set("imx708", ESP_LOG_DEBUG);
    esp_log_level_set("esp_sccb_i2c", ESP_LOG_DEBUG);

    // 1) Power + XCLK
    ESP_ERROR_CHECK(camera_power_sequence_with_xclk());

    // 2) CSI Direct Mode: Skip I2C detection, camera gets power via 22-pin CSI cable
    ESP_LOGI(TAG, "CSI Direct Mode: Bypassing I2C sensor detection");
    ESP_LOGI(TAG, "Camera powered via 22-pin CSI cable with on-board oscillator");
    
    // Create I2C bus for ESP Video framework (even though we skip detection)
    i2c_master_bus_handle_t bus = NULL;
    ESP_ERROR_CHECK(make_i2c_bus(&bus));
    ESP_LOGI(TAG, "I2C bus created (for ESP Video framework, not for sensor probe)");

    // 3) Init CSI (CSI Direct Mode - no sensor handle needed)
    esp_video_init_csi_config_t csi = {
        .sccb_config = {
            .init_sccb = false,  // Don't initialize I2C/SCCB (CSI Direct Mode)
            .i2c_handle = bus,
            .freq = CONFIG_CAM_I2C_FREQ_HZ,
        },
        .reset_pin = CAM_RESET_GPIO,
        .pwdn_pin  = CAM_PWR_EN_GPIO,
    };
    esp_video_init_config_t vid = { .csi = &csi };
    esp_err_t err = esp_video_init(&vid);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_video_init failed: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "CSI Direct Mode initialization failed. Check 22-pin CSI cable connection.");
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
    ESP_LOGI(TAG, "ESP Video framework initialized (CSI Direct Mode)");
    
    // 4) Verify camera contact by reading chip ID
    ESP_LOGI(TAG, "Attempting to verify camera contact via I2C...");
    ESP_LOGI(TAG, "NOTE: This requires external 2.2-4.7kΩ pull-ups on GPIO7/GPIO8 to 3.3V");
    
    vTaskDelay(pdMS_TO_TICKS(100)); // Give camera time to stabilize
    
    err = probe_imx708_chip_id(bus);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "✅ CAMERA CONTACT VERIFIED: IMX708 chip ID 0x0708 detected!");
        ESP_LOGI(TAG, "✅ Camera is responding on I2C address 0x1A");
        ESP_LOGI(TAG, "✅ Ready for streaming configuration");
    } else {
        ESP_LOGW(TAG, "⚠️  Camera chip ID read failed (expected with missing pull-ups)");
        ESP_LOGW(TAG, "⚠️  Add 2.2-4.7kΩ resistors: GPIO7→3.3V and GPIO8→3.3V");
        ESP_LOGW(TAG, "⚠️  Continuing anyway - CSI hardware may still work");
    }

    // DONE: Report status
    while (true) {
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Status: Camera contact verified, CSI initialized");
        } else {
            ESP_LOGI(TAG, "Status: CSI initialized, camera contact unverified (need pull-ups)");
        }
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}