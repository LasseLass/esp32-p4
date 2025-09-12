| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2 | ESP32-H21 | ESP32-H4 | ESP32-P4 | ESP32-S2 | ESP32-S3 | Linux |
| ----------------- | ----- | -------- | -------- | -------- | -------- | --------- | -------- | --------- | -------- | -------- | -------- | -------- | ----- |

# ESP32-P4 Camera Application - IMX708 Support

ESP32-P4 camera application with verified IMX708 (Raspberry Pi Camera Module 3) support.

## VERIFIED WORKING CONFIGURATION

**Hardware Setup:**
- **XCLK:** GPIO15, 24MHz (critical requirement)
- **I2C:** Port 0, SDA=GPIO7, SCL=GPIO8, 100kHz
- **RESET:** GPIO11 (active LOW)
- **PWDN:** GPIO10 (active LOW for power on)
- **I2C Address:** 0x1A (7-bit)

**Current Status:**
- IMX708 sensor detection successful
- I2C communication working (ACK received)
- Chip ID 0x0708 verified
- ISP pipeline configuration in progress

## Project Structure

```
├── CMakeLists.txt
├── main/
│   ├── CMakeLists.txt
│   ├── Kconfig.projbuild       # GPIO and I2C configuration
│   ├── camera_main.c           # Main application
│   └── idf_component.yml
├── third_party/
│   └── esp-video-components/   # ESP Video API components
└── README.md                   # This file
```

## Configuration

Use `idf.py menuconfig` to configure camera settings under "Camera / CSI settings":
- I2C SDA/SCL pins (default: GPIO7/GPIO8)
- I2C frequency (default: 100kHz)
- Power and reset pins (default: GPIO10/GPIO11)

## Troubleshooting

* Program upload failure

    * Hardware connection is not correct: run `idf.py -p PORT monitor`, and reboot your board to see if there are any output logs.
    * The baud rate for downloading is too high: lower your baud rate in the `menuconfig` menu, and try again.

## Technical support and feedback

Please use the following feedback channels:

* For technical queries, go to the [esp32.com](https://esp32.com/) forum
* For a feature request or bug report, create a [GitHub issue](https://github.com/espressif/esp-idf/issues)

We will get back to you as soon as possible.
