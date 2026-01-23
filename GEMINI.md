# Project: ESP32 8BitDo Gamepad Receiver

## Overview
This project implements an ESP32 firmware that acts as a Bluetooth HID Host for **8BitDo Ultimate 2C Wireless Controllers** (and compatible devices). It supports **Dual Mode Bluetooth** (Classic + BLE) to ensure connectivity across different controller modes.

The firmware automatically scans for specific controller names ("8BitDo", "Wireless Controller", "Pro Controller"), connects to them, and parses their HID input reports.

## Architecture & Conventions

### Core Logic
- **Entry Point:** `main/rcontrol.c`
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

- **`main/rcontrol.c`**:
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

## Hardware Mapping (Linux Environment)
**Crucial:** This mapping was empirically determined on Linux. Mappings may shift on Windows/Android.

*   **Vendor ID:** `2dc8`
*   **Product ID:** `301b` (sometimes varies)

### Buttons
| Logic | ID | Physical |
| :--- | :--- | :--- |
| **0** | `btn-a` | A |
| **1** | `btn-b` | B |
| **2** | `btn-l4` | L4 (Extra Left Bumper) |
| **3** | `btn-x` | X |
| **4** | `btn-y` | Y |
| **5** | `btn-r4` | R4 (Extra Right Bumper) |
| **6** | `btn-l1` | L1 (Bumper) |
| **7** | `btn-r1` | R1 (Bumper) |
| **8** | `btn-l2` | L2 (Trigger Digital) |
| **9** | `btn-r2` | R2 (Trigger Digital) |
| **10** | `btn-view` | View (Select/Minus) |
| **11** | `btn-menu` | Menu (Start/Plus) |
| **12** | `btn-home` | Home (Center) |
| **13** | `btn-l3` | L3 (Stick Click) |
| **14** | `btn-r3` | R3 (Stick Click) |

### Axes
*   **Ax0 / Ax1:** Left Stick (X / Y)
*   **Ax2:** Right Stick X
*   **Ax5:** Right Stick Y (Note: Non-standard index)
*   **Ax3:** Left Trigger (Analog)
*   **Ax4:** Right Trigger (Analog)
*   **Ax9:** D-Pad (Hat Switch)
    *   Up: ~ -1.0
    *   Right: ~ -0.43
    *   Down: ~ 0.14
    *   Left: ~ 0.71
