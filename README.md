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
