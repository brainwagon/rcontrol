# ESP32 8BitDo Robot Controller

This firmware transforms an ESP32 into a wireless robot controller. It connects to an **8BitDo Ultimate 2C Wireless Controller** (or compatible gamepad) and drives a robot using an **L298 Dual H-Bridge motor driver** with Tank Drive steering.

## System Architecture

The project is refactored into modular components for easy integration into larger firmware projects:

- **`gamepad` Component:** Handles Bluetooth/HID discovery, connection, and report parsing. It normalizes stick inputs to a range of `-1.0` to `1.0`.
- **`motor_driver` Component:** Provides a high-level API for controlling L298-style motor drivers using ESP32 `LEDC` (PWM) and GPIO.
- **`ssd1306` Component:** I2C driver for the OLED display. Handles initialization, text rendering (8x8 font), and frame buffer management.
- **`st7735` Component:** SPI driver for 1.8" TFT displays. Supports 16-bit color, multiple instances, and basic graphics primitives.
- **`main` Application:** Maps the Left Stick Y-axis to the Left Motor and the Right Stick Y-axis to the Right Motor (**Tank Drive**). It also updates the enabled display(s) with status and speed data.

## Hardware Wiring (Pinouts)

### Wiring Checklist

| Category | L298N Terminal | ESP32 Pin | Description |
| :--- | :--- | :--- | :--- |
| **Power** | 12V | Battery (+) | 7V - 12V Input |
| | GND | Battery (-) **AND** ESP32 GND | **Common Ground (Essential)** |
| | 5V | ESP32 VIN / 5V | Logic Power for ESP32 |
| **Left Motor** | ENA | **GPIO 32** | PWM Speed Control |
| | IN1 | **GPIO 33** | Direction Control |
| | IN2 | **GPIO 25** | Direction Control |
| **Right Motor** | ENB | **GPIO 26** | PWM Speed Control |
| | IN3 | **GPIO 27** | Direction Control |
| | IN4 | **GPIO 14** | Direction Control |
| **OLED Display** | SDA | **GPIO 21** | I2C Data (if enabled) |
| | SCL | **GPIO 22** | I2C Clock (if enabled) |
| | VCC | 3.3V | Power |
| | GND | GND | Common Ground |
| **TFT Display** | SCLK | **GPIO 18** | SPI Clock |
| | MOSI | **GPIO 23** | SPI Data |
| | DC | **GPIO 17** | Data/Command |
| | RST | **GPIO 16** | Reset |
| | CS1 | **GPIO 5** | Chip Select (Display 1) |
| | CS2 | **GPIO 4** | Chip Select (Display 2 - Optional) |
| | VCC | 3.3V | Power |
| | GND | GND | Common Ground |
| | BL | 3.3V | Backlight (or use GPIO/PWM) |

### Configuration (Menuconfig)

The project supports flexible display configurations. You can enable or disable displays using:

```bash
idf.py menuconfig
```

Navigate to **Display Configuration**:
- **SSD1306 (I2C) Configuration:** Enable/Disable the I2C OLED.
- **ST7735 (SPI) Configuration:**
    - Enable support for ST7735 TFTs.
    - Select number of displays (1 or 2).
    - Configure Pins (SCLK, MOSI, DC, RST, CS1, CS2).


### L298N Jumper Hints

The L298N module has several jumpers that are critical for correct operation:

1.  **5V Enable Jumper (EN_5V):**
    *   **Keep it ON** if your battery is between 7V and 12V. This enables the onboard 78M05 regulator to provide 5V power to the logic terminal and the ESP32.
    *   **Remove it** if your battery is > 12V. You will need to provide a separate 5V supply to the 5V terminal.
2.  **ENA / ENB Jumpers:**
    *   **REMOVE these jumpers.** These jumpers typically pull the Enable pins to 5V (Full Speed). Since the ESP32 is controlling the speed via PWM on GPIO 32 and 26, you must remove the jumpers and connect the ESP32 pins to the top pins of these headers.
3.  **GND Connection:**
    *   Ensure the **GND** terminal of the L298N is connected to both the battery negative and the ESP32 GND pin. Without this common ground, the PWM signals will not work.

## Features
- **Dual Mode Bluetooth:** Supports both Classic Bluetooth and BLE connection attempts.
- **Auto-Connect:** Automatically scans for and connects to devices matching "8BitDo", "Wireless Controller", or "Pro Controller".
- **Tank Drive Steering:** Intuitive control using the two analog sticks.
- **OLED Status Display:** Real-time feedback of motor speeds and connection status on an SSD1306 128x64 display.
- **PWM Speed Control:** Smooth acceleration and deceleration using the ESP32 LEDC peripheral (10kHz frequency).

## Build Environment
- **SDK:** ESP-IDF v6.1-dev
- **Target:** ESP32

## How to Build and Flash

1.  **Set up ESP-IDF:** Ensure your `idf.py` environment is active.
2.  **Navigate to project directory:**
    ```bash
    cd rcontrol
    ```
3.  **Build:**
    ```bash
    idf.py build
    ```
4.  **Flash and Monitor:**
    Replace `/dev/ttyUSB0` with your device port.
    ```bash
    idf.py -p /dev/ttyUSB0 flash monitor
    ```

## Development History & Troubleshooting

This project evolved through several iterations to address specific crashes and connectivity issues:

1.  **Connection Deadlocks:** Implemented a FreeRTOS Queue to handle connection requests outside of the Bluetooth callback context.
2.  **Protocol Mode:** The gamepad component automatically sends a request to switch the controller to **Report Protocol Mode** upon connection. This is required to receive analog stick data rather than just digital "boot" mode events.
3.  **Modularization:** Extracted the core Bluetooth and Motor logic into standalone components (`gamepad` and `motor_driver`) to make the main application clean and easy to modify.

## Troubleshooting Motor Direction

If your robot moves backward when pushing the sticks forward:

- **Option A:** Swap the wires connecting the motor to the L298.

- **Option B:** Swap the `IN1`/`IN2` (or `IN3`/`IN4`) pin definitions in `main/rcontrol.c`.

- **Option C:** Invert the speed value in `input_callback` (e.g., `motor_driver_set_speed(-state->left_stick_y, -state->right_stick_y)`).
