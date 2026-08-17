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

The [complete Settings guide](https://github.com/AELovelace/Doll-OS-FNK0104/blob/main/SETTINGS_GUIDE.md)
documents every persistent DOLL-OS key and every rotary menu action.

```cpp
#define SLAVE_ROTARY_A_PIN 6       // CLK: rotation clock / channel A
#define SLAVE_ROTARY_B_PIN 7       // DT: rotation data / channel B
#define SLAVE_ROTARY_BUTTON_PIN 5  // SW: shaft push-button signal
```

Set `SLAVE_ROTARY_ENCODER_ENABLED` to `0` if the encoder is not installed.

## Analog thumb joystick

The slave can read a KY-023-style analog joystick module (two potentiometers and
a push button) and send the same signals the arrow keys send:

- module `VRx` / X axis -> `GPIO1`
- module `VRy` / Y axis -> `GPIO2`
- module `SW` / push button -> `GPIO4`
- module `+5V` / `VCC` -> `3V3` (never 5 V; the wipers must stay inside the
  ADC's input range)
- module `GND` -> `GND`

Both axes have to stay on ADC1 pins (`GPIO1`-`GPIO10` on the ESP32-S3) because
ADC2 belongs to the radio while Wi-Fi is up. `GPIO4` uses an internal pullup, so
the button needs no resistor of its own.

Pushing the stick sends arrow keys in normal mode and Game Boy D-pad button
events in game mode, exactly as the arrows and the controller's D-pad already do,
including auto-repeat while a direction is held. Clicking the stick presses
Select in game mode, and sends Escape in normal mode -- the same pair the
controller's View button sends.

```cpp
#define SLAVE_JOYSTICK_X_PIN 1       // VRx: analog X axis
#define SLAVE_JOYSTICK_Y_PIN 2       // VRy: analog Y axis
#define SLAVE_JOYSTICK_BUTTON_PIN 4  // SW: stick push-button signal
```

At boot the sketch measures the resting position and uses it as centre, so leave
the stick alone through startup. An implausible rest position -- one of the
rails, or a reading that will not sit still -- is how an unconnected pin looks,
and the joystick is left disabled rather than jamming a direction on. `JOY` on
the console reports the live axis values, `JOY CAL` re-centres, and `JOY 1`
forces it on if detection was wrong about your wiring. If a direction comes out
backward, set `SLAVE_JOYSTICK_INVERT_X` or `SLAVE_JOYSTICK_INVERT_Y` in
`BoardVariant.h` to `1`.

Set `SLAVE_JOYSTICK_ENABLED` to `0` if the joystick is not installed.

## Three-button bar

Three momentary switches on a shared rail cover the Game Boy buttons the
joystick does not. Wire the bar's common rail to `3V3` and each switch to its
own pin:

- left button -> `GPIO10` -> **A**
- middle button -> `GPIO11` -> **B**
- right button -> `GPIO12` -> **Start**

Nothing else is needed; the sketch holds the three pins low with internal
pulldowns, so a press is the switch closing 3V3 onto the pin. If your bar is
built the other way round -- common rail to `GND`, switches pulling their pin
down -- set `SLAVE_BUTTON_BAR_ACTIVE_LOW` in `BoardVariant.h` to `1`. `BTN` on
the console prints the raw pin levels next to the decoded press state, which is
the quickest way to tell which one you have.

```cpp
#define SLAVE_BUTTON_A_PIN 10      // Game Boy A
#define SLAVE_BUTTON_B_PIN 11      // Game Boy B
#define SLAVE_BUTTON_C_PIN 12      // Game Boy Start
```

In game mode the three are the real A, B, and Start. Outside it they send what
the controller's matching buttons send: Enter, Escape, and Enter. They merge
with the joystick, the keyboard, and a paired controller rather than replacing
them, so a stick direction and a button held together come through as a chord.

Set `SLAVE_BUTTON_BAR_ENABLED` to `0` if the bar is not installed.

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
