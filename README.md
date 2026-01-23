# ESP32 8BitDo Ultimate 2C Gamepad Receiver

This firmware allows an ESP32 to connect to an **8BitDo Ultimate 2C Wireless Controller** (and compatible devices) via Bluetooth and logs input events to the serial console.

## Features
- **Dual Mode Bluetooth:** Supports both Classic Bluetooth and BLE connection attempts.
- **Auto-Connect:** Automatically scans for and connects to devices matching "8BitDo", "Wireless Controller", or "Pro Controller".
- **Input Logging:** Parses and prints HID input reports to the serial console (deduplicated to prevent log flooding).
- **Robustness:** Handles asynchronous connection events via FreeRTOS queues to prevent deadlocks and Watchdog Timeouts.

## Build Environment
- **SDK:** ESP-IDF v6.1-dev (specifically tested with `v6.1-dev-2002-gfa47e101d5`)
- **Target:** ESP32

## How to Build and Flash

1.  **Set up ESP-IDF:** Ensure your `idf.py` environment is active.
2.  **Navigate to project directory:**
    ```bash
    cd esp32_8bitdo_gamepad
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

1.  **Initial Approach:** Started with a standard HID Host implementation.
    - **Issue:** Crashes (`ESP_ERR_INVALID_ARG`) during initialization.
    - **Cause:** Double-initialization of the Bluedroid stack (once manually, once by `esp_hid`).
    - **Fix:** Removed redundant manual initialization, though later restored it with correct ordering.

2.  **Memory Optimization Attempt:** Tried to force "Classic Bluetooth Only" to save RAM.
    - **Issue:** Boot loops and initialization failures.
    - **Cause:** The `esp_hid` component has dependencies on BLE (GATT) components being enabled in the configuration, even if not explicitly used. Disabling BLE at the controller level caused mismatches with the host stack configuration.
    - **Fix:** Reverted to **Dual Mode (BTDM)** in both the controller (`esp_bt_controller_enable`) and `sdkconfig`.

3.  **Connection Deadlocks:**
    - **Issue:** The firmware would hang or stall upon finding the device.
    - **Cause:** Calling the blocking function `esp_hidh_dev_open` directly inside the Bluetooth GAP callback. The callback runs in the Bluetooth task context; waiting for a connection event (which must be processed by the same task) caused a deadlock.
    - **Fix:** Implemented a **FreeRTOS Queue**. The callback now pushes a connection request to the queue, and the main application task (`app_main`) consumes it and initiates the connection safely.

4.  **BLE Discovery:**
    - **Issue:** The specific 8BitDo controller presented itself as a BLE device in some modes, which was not being picked up by the Classic scan.
    - **Fix:** Enabled explicit BLE scanning alongside Classic scanning and added logic to parse BLE advertisement data (including Shortened Local Names) to identify the controller.

5.  **Watchdog Timeouts:**
    - **Issue:** High-frequency input reports (joystick movement) flooded the UART, triggering the Task Watchdog Timer (TWDT).
    - **Fix:** Added logic to only log input reports when the data payload actually changes.

## Configuration
The project relies on `sdkconfig.defaults` to ensure the Bluetooth stack is correctly configured for Dual Mode with HID Host support.

## TODO
- [ ] Investigate how to trigger/enable the vibration sensor (rumble) for this controller. Previous attempts using standard HID output reports were unsuccessful.
