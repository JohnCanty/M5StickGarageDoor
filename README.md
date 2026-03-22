# M5CycleHomeZero GPS Redo

GPS-triggered automatic garage door controller for M5StickC Plus.

This firmware runs on an M5StickC Plus (ESP32) with an AT6668 GPS module and controls a garage door via MQTT. It uses a persisted 3-mode state machine, an initial Wi-Fi setup portal, and GPS-based approach detection to automate door open/close actions.

## Highlights

- PlatformIO-based Arduino firmware for M5StickC Plus
- GPS serial auto-detection across multiple baud/frame combinations
- Stored GPS serial config is retried first on future boots
- Captive setup AP for first-time configuration
- Live GPS status in setup portal:
  - Position
  - Satellite count
  - Detected serial config
  - Set Here buttons for home latitude/longitude
- Mode-driven behavior persisted in NVS
- MQTT command and status integration for garage door control
- XOR-obfuscated credential storage in NVS using EFuse MAC-derived key

## Hardware

- Board: M5StickC Plus (ESP32)
- GPS: AT6668 (SoftwareSerial)
- Relay outputs:
  - Relay1: GPIO 26 (speed-gated output)
  - Relay2: GPIO 25 (reserved)

GPS SoftwareSerial pins:

- RX: GPIO 33
- TX: GPIO 32

## Firmware Behavior

The firmware uses a persisted counter in NVS (lastAction/counter) to select mode at boot:

- Mode 0 (counter == 0): GPS monitoring mode
  - Reads speed and distance-to-home continuously
  - Drives Relay1 HIGH when speed is below speed setpoint (10 mph)
  - Detects approach to home:
    - Distance must be inside 500 m zone
    - 3 consecutive closer readings required
  - On approach trigger, reboots into Mode 2

- Mode 1 (1 <= counter <= 99): close-door flow
  - Connects Wi-Fi, syncs NTP, connects MQTT (limited retry)
  - Waits for either:
    - BtnA press, or
    - IMU movement threshold exceeded
  - Publishes:
    - Arrived topic payload: Gone
    - Garage command payload: clos
  - Waits for MQTT status callback confirmation, then reboots to Mode 0

- Mode 2 (counter >= 100): open-door flow
  - Connects Wi-Fi/NTP/MQTT (infinite retry)
  - Publishes:
    - Arrived topic payload: Home
    - Garage command payload: open
  - Waits for MQTT status callback confirmation, then powers off
  - Stores next wake behavior as Mode 1

## Buttons

- BtnA
  - During boot: hold to force config portal
  - In Mode 1: manual close trigger
  - In config portal: skip and reboot

- BtnB (Mode 0)
  - Short press: immediate reboot into Mode 2 (open door)
  - Hold >= 5 seconds: set Mode 1 on next wake, then power off

## Initial Configuration Portal

If configuration is missing (or BtnA is held at boot), device enters config mode:

- AP SSID: M5CycleSetup
- URL: http://192.168.4.1
- Form fields:
  - WiFi SSID
  - WiFi Password
  - MQTT Server
  - NTP Server
  - Motorcycle Base MQTT Topic
  - Garage Door Command Topic (publish)
  - Garage Door Status Topic (subscribe)
  - Home Latitude
  - Home Longitude

Portal GPS enhancements:

- GPS is initialized before portal starts
- Live GPS status is shown in the page
- Set Here buttons copy current GPS position into Home Latitude/Home Longitude
- Detected GPS serial settings are stored and used as first try on later boots

## MQTT Interface

Configured at runtime via portal.

Derived topic:

- LWT topic: <base> + LWT

Arrival topic:

- Arrived topic: <base> + Arrived
- Published payloads:
  - Home (opening flow)
  - Gone (closing flow)

Garage topics:

- Publish command topic: user-configured garage publish topic
  - open
  - clos
- Subscribe status topic: user-configured garage subscribe topic
  - Expected status payloads include open or clos

## Configuration Storage

Namespace: config

Encrypted/obfuscated fields (XOR with EFuse MAC-derived key):

- WiFi SSID/password
- MQTT server
- NTP server
- base topic
- garage publish/subscribe topics
- home latitude/longitude

Plaintext fields:

- gpsbaud
- gpsconfig

State namespace: lastAction

- counter (mode value)
- away (boolean)

## Build Environment

From platformio.ini:

- Platform: espressif32@6.8.1
- Board: m5stick-c
- Framework: arduino
- Monitor speed: 115200

Libraries:

- M5StickCPlus
- PubSubClient
- TinyGPSPlus
- EspSoftwareSerial

## Build and Flash

1. Install PlatformIO Core or use PlatformIO in VS Code.
2. From project root, build:

```bash
pio run
```

3. Flash firmware:

```bash
pio run -t upload
```

4. Open serial monitor:

```bash
pio device monitor
```

## Typical First Boot Flow

1. Power on the device.
2. Connect phone/computer to Wi-Fi SSID M5CycleSetup.
3. Browse to http://192.168.4.1.
4. Fill all required fields.
5. Optionally use Set Here buttons for home coordinates once GPS fix is available.
6. Save; device reboots into normal operation.

## GPS Serial Detection Strategy

At startup, GPS is initialized as follows:

1. Try stored gpsbaud/gpsconfig first (if present)
2. If that fails, scan 24 combinations:
   - Baud: 9600, 38400, 115200, 19200, 57600, 4800
   - Frame: 8N1, 8E1, 8O1, 8N2
3. Validate by checking for NMEA sentence prefixes ($GP, $GN, $GL, $GA, $BD, $GB)
4. Fallback to 9600 8N1 if no valid combination is detected

## Notes and Limitations

- Credential protection is lightweight obfuscation, not strong cryptography.
- Garage status matching currently checks payload prefixes open and clos.
- Home coordinate fallback (if not configured) defaults to Area 51 coordinates in code.

## Project Layout

- src/main.cpp: firmware implementation
- platformio.ini: PlatformIO environment and dependencies

## License

No license file is currently included in this repository.
