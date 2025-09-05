# ESP32 Framework

A minimal Arduino sketch for ESP32 boards demonstrating use of a 128×64 SH1106 OLED display, two rotary encoders and I2S audio output. Use this as a starting point for your own applications.

## Required Libraries

Install these libraries in the Arduino IDE or with `arduino-cli` before compiling:

- **ESP32 board support** (`esp32` by Espressif)
- **Adafruit GFX Library**
- **Adafruit SH110X**
- **SimpleRotary**

## Hardware Connections

- **SH1106 128×64 OLED** (I²C): SDA → GPIO21, SCL → GPIO22, plus 3.3 V and GND.
- **MAX98357A I2S amplifier**: BCLK → GPIO17, LRCLK → GPIO16, DIN → GPIO27, SD → GPIO19, 3.3 V and GND.
- **Volume rotary encoder**: A → GPIO33, B → GPIO4, switch → GPIO23.
- **Channel rotary encoder**: A → GPIO25, B → GPIO32, switch → GPIO2.

## Setup

1. Adjust the I2S pin numbers in `config.h` if required.
2. Ensure the libraries above are installed.
3. Open `framework.ino` in the Arduino IDE, select your ESP32 board and the correct port.
4. Compile and upload.

## Operation

- The display shows current volume and channel values.
- Rotate the volume encoder to change beep volume (0–20). Press it to play a high-pitched beep.
- Rotate the channel encoder to change an integer counter. Press it to play a low-pitched beep.
- Expand the sketch with your own application logic.

## Building in a Codex/Codespace Environment

The following setup script prepares a Codespace container with all required tools and libraries for this project. Run it in your container before compiling:

```bash
#!/bin/bash
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install -y --no-install-recommends \
  curl ca-certificates git python3 tar xz-utils

curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
install -m 0755 bin/arduino-cli /usr/local/bin/arduino-cli
rm -rf bin
arduino-cli config init --overwrite
arduino-cli config set board_manager.additional_urls \
  https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32
arduino-cli lib install "Adafruit GFX Library"
arduino-cli lib install "Adafruit SH110X"
arduino-cli lib install "SimpleRotary"
```

Compile with:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 framework.ino
```
