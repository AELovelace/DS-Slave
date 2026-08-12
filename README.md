# DS-Slave

DS-Slave bridges BLE keyboards to DOLL-OS over UART1. It can also keep a BLE
gamepad connected at the same time.

The complete end-to-end assembly instructions, including both FNK0104 UART pin
variants, power arrangements, OLED, encoder, and antenna, are in the
[DOLL-OS hardware build guide](https://github.com/AELovelace/Doll-OS-FNK0104/blob/main/HARDWARE_BUILD_GUIDE.md).

## Link wiring

- GPIO17: UART1 TX to DOLL-OS
- GPIO18: UART1 RX from DOLL-OS
- 115200 baud, 8-N-1

## SSD1306 status OLED

The slave can drive a secondary SSD1306 I2C OLED at address `0x3C`. The screen
shows pairing mode, BLE scan/connect state, both peer slots, game mode,
Wi-Fi/Telnet state, saved devices, and the keyboard LED mask.

By default the sketch uses a `128x64` panel on the Lonely Binary ESP32-S3 I2C
pair:

```cpp
#define SLAVE_OLED_SDA_PIN 8
#define SLAVE_OLED_SCL_PIN 9
#define SLAVE_OLED_WIDTH 128
#define SLAVE_OLED_HEIGHT 64
```

Connect the four OLED header pins by their printed labels:

- `GND` / power and signal ground -> DS-Slave `GND`
- `VCC`, `VDD`, or `VIN` / OLED power -> DS-Slave `3V3`
- `SCL`, `SCK`, or `CLK` / I2C clock -> DS-Slave `GPIO9`
- `SDA`, `DAT`, or `DIN` / I2C data -> DS-Slave `GPIO8`

The frequently seen physical order `GND, VCC, SCL, SDA` is not universal;
follow the module silkscreen rather than assuming a left-to-right order.

Override those defines before build/upload if your OLED is wired differently.
Set `SLAVE_OLED_ENABLED` to `0` if the slave is built without the OLED.

## Rotary volume encoder

The slave can read a simple two-channel quadrature rotary encoder and send
out-of-band volume nudges to DOLL-OS. Wire the encoder common pin to `GND`, then:

- `SW` / shaft push-button signal -> `GPIO5`
- `CLK` / rotation clock / channel A -> `GPIO6`
- `DT` / rotation data / channel B -> `GPIO7`
- module `+` / `VCC` -> `3V3` (never 5 V)
- module `GND` -> `GND`

For a bare EC11 encoder, connect one of the separate push-switch terminals to
GPIO5 and the other to GND. In the three-terminal rotation group, the middle
terminal is normally common/GND; connect the outer A/CLK and B/DT terminals to
GPIO6 and GPIO7. Verify an unlabeled encoder with its datasheet or a continuity
meter because physical terminal layouts can vary.

Connect the encoder's common pin and the other side of the push button to
`GND`. The sketch enables internal pullups on all three inputs. Press the encoder
once to open Settings, rotate it to highlight Pair Device, Game Mode, Reconnect,
Sleep, or Exit, then press again to apply that choice and return to volume control. Set
`SLAVE_ROTARY_REVERSED` in `BoardVariant.h` to `1` if clockwise turns the volume down.

```cpp
#define SLAVE_ROTARY_A_PIN 6       // CLK: rotation clock / channel A
#define SLAVE_ROTARY_B_PIN 7       // DT: rotation data / channel B
#define SLAVE_ROTARY_BUTTON_PIN 5  // SW: shaft push-button signal
```

Set `SLAVE_ROTARY_ENCODER_ENABLED` to `0` if the encoder is not installed.

## Paired sleep mode

Choosing **Sleep** sends the private `0xF6` control byte to DOLL-OS, waits until
the selecting button press is released, switches off the slave OLED and status
LED, stops Wi-Fi, and places the slave ESP32-S3 in deep sleep. GPIO5 is the
active-low deep-sleep wake source.

Press the rotary dial once to wake. Deep-sleep wake resets and boots the slave;
early in `setup()` it sends several private `0xF7` wake bytes. The first UART
start bit wakes the main unit from light sleep and the remaining bytes are
discarded as controls. The slave holds GPIO17 high while asleep so the main
unit's RX wake input cannot float low.

Bench test the pair after flashing both boards:

1. Open the slave Settings menu and select **Sleep**.
2. Confirm both displays and both status LEDs turn off.
3. Wait several seconds and press, then release, the rotary dial once.
4. Confirm the slave boot banner returns and the main TFT immediately restores
   its prior frame; Wi-Fi may take a few more seconds to reassociate.
5. Confirm rotating the dial changes volume and a second button press opens
   Settings, proving the wake press was not reused as menu input.
