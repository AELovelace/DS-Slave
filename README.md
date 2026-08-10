# DS-Slave

DS-Slave bridges BLE and wired USB keyboards to DOLL-OS over UART1. It can
also keep a BLE gamepad connected at the same time.

## Wired USB keyboard

The wired path accepts standard HID boot-protocol keyboards and hot-plugging.
It shares the BLE keyboard decoder, so modifiers, function and navigation
keys, Caps Lock, F12 game-mode switching, and Game Boy controls behave the
same way on either transport.

Use these Arduino settings for an ESP32-S3 build:

- **USB Mode:** `USB-OTG (TinyUSB)`
- **USB CDC On Boot:** `Disabled`
- **USB MSC On Boot:** `Disabled`
- **USB DFU On Boot:** `Disabled`

The native USB D- and D+ signals are GPIO19 and GPIO20. Connect the keyboard
through the board's OTG-capable socket or a USB host breakout. The keyboard
also needs a 5 V VBUS supply; many ESP32-S3 device-only USB-C sockets do not
source VBUS, so use a powered OTG adapter or host breakout when the board does
not provide it. Keep the grounds common.

TinyUSB CDC cannot run at the same time because it owns the same USB-OTG
peripheral. Debug output remains available through UART0 and Telnet. Unplug
the keyboard when using the same physical socket to flash the board.

At boot, the console prints `USB keyboard host ready`. On attachment it prints
the keyboard VID/PID followed by `USB keyboard ready (boot protocol)`.
`STATUS` reports `usbKeyboard=connected` while it is active.

## Link wiring

- GPIO17: UART1 TX to DOLL-OS
- GPIO18: UART1 RX from DOLL-OS
- 115200 baud, 8-N-1

## SSD1306 status OLED

The slave can drive a secondary SSD1306 I2C OLED at address `0x3C`. The screen
shows pairing mode, BLE scan/connect state, both peer slots, USB keyboard state,
game mode, Wi-Fi/Telnet state, saved devices, and the keyboard LED mask.

By default the sketch uses a `128x64` panel on the Lonely Binary ESP32-S3 I2C
pair:

```cpp
#define SLAVE_OLED_SDA_PIN 8
#define SLAVE_OLED_SCL_PIN 9
#define SLAVE_OLED_WIDTH 128
#define SLAVE_OLED_HEIGHT 64
```

Override those defines before build/upload if your OLED is wired differently.
Set `SLAVE_OLED_ENABLED` to `0` if the slave is built without the OLED.

## Rotary volume encoder

The slave can read a simple two-pin quadrature rotary encoder and send
out-of-band volume nudges to DOLL-OS. Wire the encoder common pin to `GND`, then:

- `CLK` / `A` -> `GPIO6`
- `DT` / `B` -> `GPIO7`
- `SW` / push button -> `GPIO5`

Connect the encoder's common pin and the other side of the push button to
`GND`. The sketch enables internal pullups on all three inputs. Press the encoder
once to open Settings, rotate it to highlight Pair Device, Game Mode, Reconnect,
or Exit, then press again to apply that choice and return to volume control. Set
`SLAVE_ROTARY_REVERSED` in `BoardVariant.h` to `1` if clockwise turns the volume down.

```cpp
#define SLAVE_ROTARY_A_PIN 6
#define SLAVE_ROTARY_B_PIN 7
#define SLAVE_ROTARY_BUTTON_PIN 5
```

Set `SLAVE_ROTARY_ENCODER_ENABLED` to `0` if the encoder is not installed.
