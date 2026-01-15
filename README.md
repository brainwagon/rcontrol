# rcontrol - ESP32 Robot Firmware

This project is ESP-IDF firmware for a skid-steering robot controlled by a Bluetooth HID gamepad.

## Hardware Requirements

*   **ESP32 Development Board**
*   **L298N Motor Driver** (or compatible)
*   **2x DC Motors** (with wheels)
*   **HC-SR04 Ultrasonic Sensor**
*   **4x Microswitches** (Bumpers)
*   **2x LEDs** (Turn signals/Rear lights)
*   **Bluetooth Gamepad** (Generic HID)

## Pin Configuration

Pin mappings are defined in `main/config.h`. Default configuration:

*   **Motors (L298N):**
    *   Left ENA: GPIO 14
    *   Left IN1: GPIO 27
    *   Left IN2: GPIO 26
    *   Right ENB: GPIO 12
    *   Right IN3: GPIO 25
    *   Right IN4: GPIO 33
*   **Sensors:**
    *   Ultrasonic Trigger: GPIO 18
    *   Ultrasonic Echo: GPIO 19
    *   Bumper Front Left: GPIO 32
    *   Bumper Front Right: GPIO 35
    *   Bumper Rear Left: GPIO 34
    *   Bumper Rear Right: GPIO 39
*   **Lights:**
    *   Left Turn Signal: GPIO 2
    *   Right Turn Signal: GPIO 4

## Wi-Fi Configuration

The robot connects to a Wi-Fi network to provide a web dashboard. You must configure your network credentials before building:

1.  Run the configuration menu:
    ```bash
    idf.py menuconfig
    ```
2.  Navigate to **WiFi Configuration**.
3.  Enter your **WiFi SSID** and **WiFi Password**.
4.  Press `S` (Save) and `Esc` (Exit).

These credentials are saved in your local `sdkconfig` and will not be committed to version control.

## Setting up ESP-IDF

To build this project, you need the Espressif IoT Development Framework (ESP-IDF).

### Linux / macOS

1.  **Install Prerequisites:**
    *   **Ubuntu/Debian:**
        ```bash
        sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
        ```
    *   **macOS:**
        ```bash
        brew install cmake ninja dfu-util python3
        ```

2.  **Get ESP-IDF:**
    ```bash
    mkdir -p ~/esp
    cd ~/esp
    git clone --recursive https://github.com/espressif/esp-idf.git
    ```

3.  **Install Tools:**
    ```bash
    cd ~/esp/esp-idf
    ./install.sh esp32
    ```

4.  **Set up Environment Variables:**
    *   You need to source the export script in every terminal where you use ESP-IDF.
    ```bash
    . $HOME/esp/esp-idf/export.sh
    ```
    *   *Tip: Create an alias in your `.bashrc` or `.zshrc`: `alias get_idf='. $HOME/esp/esp-idf/export.sh'`*

### Windows

1.  Download the **ESP-IDF Online Installer** from the [Espressif website](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/windows-setup.html).
2.  Run the installer and follow the prompts. It will set up a shortcut "ESP-IDF Command Prompt" which has all the necessary environment variables set.

## Building and Flashing

1.  **Navigate to project directory:**
    ```bash
    cd path/to/rcontrol
    ```

2.  **Set target:**
    ```bash
    idf.py set-target esp32
    ```

3.  **Configure (Optional):**
    To change Bluetooth or other SDK settings.
    ```bash
    idf.py menuconfig
    ```
    *   *Note: Bluetooth Classic is enabled by default in this project's code logic, but ensure "Component config -> Bluetooth -> Bluedroid Enable" and "Classic Bluetooth" are selected if you started from scratch.*

4.  **Build:**
    ```bash
    idf.py build
    ```

5.  **Flash:**
    Connect your ESP32 via USB. Replace `/dev/ttyUSB0` with your port (COMx on Windows).
    ```bash
    idf.py -p /dev/ttyUSB0 flash
    ```

    **Note for WSL (Windows Subsystem for Linux) Users:**
    To access the USB port from WSL, you need to use `usbipd-win`.
    1.  Install [usbipd-win](https://github.com/dorssel/usbipd-win/releases) on Windows.
    2.  Open PowerShell as Administrator on Windows and list devices:
        ```powershell
        usbipd list
        ```
    3.  Find your ESP32 (e.g., "Silicon Labs CP210x" or "USB Serial Port") and note its BUSID.
    4.  Bind and attach it (replace `<busid>` with your value):
        ```powershell
        usbipd bind --busid <busid>
        usbipd attach --wsl --busid <busid>
        ```
    5.  In WSL, the device should now appear as `/dev/ttyUSB0`.

6.  **Monitor:**
    To see log output.
    ```bash
    idf.py -p /dev/ttyUSB0 monitor
    ```
    (Exit monitor with `Ctrl+]`)

## Usage

1.  Power on the robot.
2.  Put your Bluetooth gamepad into pairing mode.
3.  The robot will automatically scan and pair with the first HID device it finds.
4.  Use the Left Stick or D-Pad for movement (Skid steering).
5.  Bumpers will stop the motors if triggered.
