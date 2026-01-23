# Project: ESP32 8BitDo Gamepad Receiver

## Overview
This project implements an ESP32 firmware that acts as a Bluetooth HID Host for **8BitDo Ultimate 2C Wireless Controllers** (and compatible devices). It supports **Dual Mode Bluetooth** (Classic + BLE) to ensure connectivity across different controller modes.

The firmware automatically scans for specific controller names ("8BitDo", "Wireless Controller", "Pro Controller"), connects to them, and parses their HID input reports.

## Architecture & Conventions

### Core Logic
- **Entry Point:** `main/esp32_8bitdo_gamepad.c`
- **Bluetooth Stack:** Bluedroid (Dual Mode: Classic BT + BLE).
- **HID Component:** Uses a local copy of `esp_hid` in `components/esp_hid` to handle HID transport and report parsing.

### Concurrency Model
To prevent deadlocks common in Bluetooth callbacks:
1.  **Scanning/Callbacks:** Bluetooth GAP/GATT callbacks run in the Bluetooth task context. When a device is found, they **do not** initiate a blocking connection immediately.
2.  **Queue:** Instead, they post a `connect_req_t` to a FreeRTOS queue (`s_connect_queue`).
3.  **Main Task:** `app_main` consumes this queue and safely calls `esp_hidh_dev_open`, which is a blocking operation.

### Data Handling
- **Input Logging:** HID Input reports are high-frequency. To prevent flooding the UART and triggering the Task Watchdog Timer (TWDT), reports are only logged if the data payload has changed (`memcmp`).

## Build & Run

**Prerequisites:** ESP-IDF v6.1-dev (or compatible).

1.  **Build:**
    ```bash
    idf.py build
    ```
    *Note: The build system automatically uses `sdkconfig.defaults` to configure the Bluetooth stack for Dual Mode and HID Host support.*

2.  **Flash & Monitor:**
    ```bash
    idf.py -p /dev/ttyUSB0 flash monitor
    ```
    *(Replace `/dev/ttyUSB0` with your actual port)*

## Key Files

- **`main/esp32_8bitdo_gamepad.c`**:
    - `app_main`: Initializes BT stack, HID Host, and processes connection queue.
    - `bt_gap_cb` / `ble_gap_cb`: Handles scanning and discovery.
    - `hidh_callback`: Handles HID events (Open, Input, Close).
    - `console_task`: Reads UART input for interactive commands.

- **`components/esp_hid/`**:
    - Contains the implementation for HID over Bluetooth (Classic and BLE).
    - `esp_hidh.c`: HID Host implementation.

## Interactive Console Commands
Once running, you can type these characters into the serial monitor:

- `p`: Toggle Protocol Mode (Report <-> Boot)
- `d`: Dump device info (features, report maps)
