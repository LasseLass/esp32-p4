| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2 | ESP32-H21 | ESP32-H4 | ESP32-P4 | ESP32-S2 | ESP32-S3 | Linux |
| ----------------- | ----- | -------- | -------- | -------- | -------- | --------- | -------- | --------- | -------- | -------- | -------- | -------- | ----- |

# ESP32-P4 Camera Application — IMX708 Bring-up (ESP-IDF only)

Minimal IMX708 (Raspberry Pi Camera Module 3) bring-up on ESP32-P4 using the native ESP Video + esp_cam_sensor APIs only.

- No V4L2, no /dev/video* ioctls
- No fake frame generation or simulated checksums
- CSI-only path with RAW10 streaming from sensor

## VERIFIED WORKING CONFIGURATION

**Hardware Setup:**
- XCLK: GPIO15 @ 24 MHz
- I2C: Port 0, SDA=GPIO7, SCL=GPIO8, 100 kHz, internal pull-ups disabled (1.8 V CCI)
- RESET: GPIO11 (active LOW)
- PWDN: GPIO10 (active LOW = ON)
- Sensor I2C address: 0x1A (7-bit)
- 22‑pin CSI cable from IMX708 to ESP32‑P4

**Bring-up sequence implemented (in `main/camera_main.c`):**
1) Enable CSI 1.8 V LDO (VO4)
2) Configure PWDN/RESET GPIOs, hold RESET low, set PWDN low (power ON)
3) Start XCLK 24 MHz on GPIO15 while RESET is held
4) Release RESET high and wait 50 ms
5) Create I2C master bus on port 0 (SDA=7, SCL=8), internal pull-ups disabled
6) Read IMX708 chip ID from 0x0016/0x0017 — expect 0x0708 at address 0x1A
7) Initialize ESP Video with CSI, passing the existing I2C handle (init_sccb=false)
8) Get sensor handle via `esp_video_get_csi_video_device_sensor()`
9) Set RAW10 format (example 1280x720) and start stream
10) Log success and keep streaming (no frame simulation)

## Project Structure

```
├── CMakeLists.txt
├── main/
│   ├── CMakeLists.txt
│   ├── Kconfig.projbuild       # GPIO and I2C configuration
│   ├── camera_main.c           # Minimal ESP-IDF-only bring-up (no V4L2)
│   └── idf_component.yml
├── third_party/
│   └── esp-video-components/   # ESP Video API components
└── README.md                   # This file
```

## Configuration

Use `idf.py menuconfig` to configure camera settings under "Camera / CSI settings":
- I2C SDA/SCL pins (default: GPIO7/GPIO8)
- I2C frequency (default: 100 kHz)
- Power and reset pins (default: GPIO10/GPIO11)
- Disable internal pull-ups on SDA/SCL (1.8 V CCI)

ESP Video options (menuconfig → Espressif Video Configuration):
- Enable MIPI‑CSI video device: enabled
- Enable ISP based video device: disable to avoid hardware ISP initialization
- ISP pipeline controller: disabled

## Expected Logs (Acceptance Criteria)

On success you should observe these lines in the monitor:

```
IMX708 chip id: 0x0708 (expected 0x0708)
esp_video_init OK
IMX708 streaming started (RAW10 1280x720).
```

No V4L2 messages and no fake frame logs should appear.

## Build and Run

```
idf.py set-target esp32p4
idf.py menuconfig   # ensure ISP video device is disabled
idf.py build flash monitor
```

If `esp_cam_sensor_set_format` returns not supported for 1280x720 RAW10, try a driver-supported RAW10 mode (e.g. 2304x1296) or enable the corresponding IMX708 format in menuconfig.

## Troubleshooting

- No chip ID or NACKs on I2C
  - Verify XCLK is running while RESET is held
  - Disable internal pull-ups on I2C (1.8 V CCI domain)
  - Confirm PWDN low (power on) and RESET released high

- `esp_video_init` fails with ISP error
  - Disable “Enable ISP based Video Device” in menuconfig
  - Keep MIPI‑CSI enabled; ISP pipeline controller disabled

* Program upload failure
  - Hardware connection is not correct: run `idf.py -p PORT monitor` and reboot the board to check logs
  - Baud rate too high: lower baud rate in menuconfig and retry

## Technical support and feedback

Please use the following feedback channels:

* For technical queries, go to the [esp32.com](https://esp32.com/) forum
* For a feature request or bug report, create a [GitHub issue](https://github.com/espressif/esp-idf/issues)

We will get back to you as soon as possible.
