/*
  DS-Slave: BLE/USB HID keyboard and BLE gamepad to UART bridge for ESP32-S3.

  Pins:
    GPIO17 = UART TX to the downstream device
    GPIO18 = UART RX from the downstream device

  A standard USB boot keyboard can be connected through the ESP32-S3 native
  USB-OTG port. ESP32-S3 supports BLE, not Bluetooth Classic. Put a BLE
  keyboard (or a
  Bluetooth-LE Xbox controller -- Series X|S / model 1914 and later, i.e. the
  ones that pair directly with a phone) in pairing mode and this sketch connects
  to the first device advertising the HID service. Input goes out UART1 as
  keystrokes, or as button events while game mode is on.

  Two devices can be connected at the same time -- typically a keyboard for the
  shell and a controller for the emulator -- and their input is merged into one
  stream, so the DS never has to know how many are attached. Run PAIR once per
  device; each is remembered and reconnects on its own afterwards.

  UART1 RX accepts line commands:
    STATUS
    SCAN
    LED <0-31>
    NUM 0|1
    CAPS 0|1
    SCROLL 0|1
    OUT <hex bytes>
    GAME 0|1
    PAD [slot] AUTO|KEYBOARD|GAMEPAD
    DUMP 0|1
    DROP

  The status WS2812 uses FastLED:
    red = nothing connected
    magenta = BLE or USB keyboard connected
    orange = gamepad connected (keystroke mode)
    yellow = keyboard and gamepad both connected
    cyan = game mode (button events)
    blue 10 ms flash = transmitted serial on GPIO17
    green 10 ms flash = received serial command input

  Optional SSD1306 I2C OLED status display:
    address = 0x3C
    GND = power/signal ground
    VCC/VDD/VIN = 3V3 power (never 5 V)
    SDA/DAT/DIN = GPIO8 I2C serial data
    SCL/SCK/CLK = GPIO9 I2C serial clock

  Optional rotary encoder volume control:
    GPIO5 = encoder shaft push-button signal / SW
    GPIO6 = encoder rotation clock / channel A / CLK
    GPIO7 = encoder rotation data / channel B / DT

  Optional analog thumb joystick (KY-023 and friends):
    GND = power/signal ground
    +5V/VCC = 3V3 power (the module is a pair of pots; 3V3 keeps the wipers
              inside the ADC's range -- never 5 V)
    VRx = GPIO1 analog X axis
    VRy = GPIO2 analog Y axis
    SW  = GPIO4 push-button signal, active low
  It drives the same signals the arrow keys do: CSI arrows in keystroke mode,
  D-pad button events in game mode. Clicking the stick is Select (Escape when
  not in game mode), the same as the pad's View button.

  Optional three-button bar, common rail to 3V3, one switch per pin. The bar reads
  Start, B, A from left to right, and the pins ascend in that same order:
    GPIO10 = Start (left)
    GPIO11 = B     (middle)
    GPIO12 = A     (right)
  In game mode these are the real Game Boy Start/B/A. Outside it they are the
  only input that does not send keystrokes: each press sends one private byte
  (0xF8 Start, 0xFA B, 0xFB A) and DOLL-OS reads it against whatever is running --
  previous/pause/next for the music player and the radio, or gb/radio/music
  launchers on an idle shell. See PadButtons.ino on the DS side.
*/

#include <Arduino.h>
#include <FastLED.h>
#include <NimBLEDevice.h>
#include <SPIFFS.h>
#include <Wire.h>
#include <WiFi.h>
#include <usb/usb_host.h>
#include <driver/gpio.h>
#include <driver/rtc_io.h>
#include <esp_sleep.h>
#include "BoardVariant.h"

#include <ctype.h>
#include <string.h>
#include <string>
#include <vector>

// USB host mode owns the ESP32-S3 USB-OTG peripheral. TinyUSB CDC uses the
// same peripheral, so a wired keyboard is enabled only with Tools > USB Mode >
// USB-OTG (TinyUSB) and Tools > USB CDC On Boot > Disabled. The hardware
// USB-Serial/JTAG choice remains available for flashing and diagnostics, but it
// cannot share the native D-/D+ pins with a host keyboard.
#if CONFIG_IDF_TARGET_ESP32S3 && ARDUINO_USB_MODE == 0 && ARDUINO_USB_CDC_ON_BOOT == 0
#define USB_KEYBOARD_HOST_ENABLED 1
#else
#define USB_KEYBOARD_HOST_ENABLED 0
#endif

// These types are declared up here, ahead of everything else, because functions
// further down return them. The Arduino builder generates prototypes for every
// function in a .ino and splices them in immediately after the last #include --
// a type defined next to the code that uses it is not yet visible at that splice
// point, and the generated prototype fails to compile.

// How many HID devices may be connected at once. Two covers the case this exists
// for -- a keyboard for the shell and a gamepad for the emulator, both live, no
// re-pairing to swap between them. NimBLE's own ceiling is
// CONFIG_BT_NIMBLE_MAX_CONNECTIONS (3 by default).
static constexpr uint8_t MAX_PEERS = 2;
static constexpr uint8_t MAX_INPUT_REPORTS = 8;

// What kind of HID device a peer is. Decided at connect time from the
// advertisement (appearance/name) and, failing that, latched from the shape of
// the first input report it sends -- a keyboard boot/report packet is 8 bytes,
// an Xbox pad's is 14-16. The "PAD" command can override a wrong guess.
enum PeerKind : uint8_t { PEER_UNKNOWN = 0, PEER_KEYBOARD = 1, PEER_GAMEPAD = 2 };

// One gamepad button's keystroke, for the non-game-mode mapping (padKeyBindings).
struct PadKeyBinding {
  uint32_t bit;
  const char *sequence;
  bool repeats;
};

// Which characteristic carries which HID report ID. A keyboard has one input
// report and we never needed to tell them apart; an Xbox pad has several (ID 1 =
// sticks/buttons, ID 2 = the Guide button on its own), and they arrive through
// the same notify callback, so the callback has to know which one it is holding.
// Notification payloads do not include the report ID -- it lives in the Report
// Reference descriptor (0x2908) read once at subscribe time.
struct InputReportRef {
  NimBLERemoteCharacteristic *characteristic;
  uint8_t reportId;
};

// Everything the bridge knows about one HID device. All of the decode state in
// here used to be file-global, back when only one device could be connected.
// With two, global state is actively wrong: each report would clear the other
// device's "keys currently held", so a key held on the keyboard would be
// released by the next packet from the pad and vice versa.
struct Peer {
  NimBLEClient *client;
  bool inUse;           // slot claimed -- connecting, or connected
  bool connected;       // HID service found, input reports subscribed
  bool needsConfigure;  // connected at the BLE level, waiting for loop() to set up HID
  PeerKind kind;
  NimBLEAddress address;
  String name;

  NimBLERemoteCharacteristic *hidOutputReport;
  NimBLERemoteCharacteristic *bootOutputReport;
  InputReportRef reports[MAX_INPUT_REPORTS];
  uint8_t reportCount;

  // keyboard decode state
  uint8_t lastKeys[6];
  uint8_t lastModifiers;
  bool f12Prev;

  // gamepad decode state
  uint32_t padButtons;
  uint32_t padPrevKeyButtons;
  uint32_t padRepeatBit;
  uint32_t padRepeatAt;
  bool guidePrev;

  // this peer's contribution to the merged game-mode button bitmap
  uint8_t gbMask;
  bool gbQuit;
  bool gbMenu;
};

static constexpr size_t TELNET_OUTPUT_BUFFER_SIZE = 4096;
static uint8_t telnetOutputBuffer[TELNET_OUTPUT_BUFFER_SIZE];
static volatile size_t telnetOutputHead = 0;
static volatile size_t telnetOutputTail = 0;
static volatile bool telnetOutputEnabled = false;
static portMUX_TYPE telnetOutputMux = portMUX_INITIALIZER_UNLOCKED;

static auto &UsbSerialPort = Serial;
#ifdef Serial
#undef Serial
#endif

static void queueTelnetOutput(const uint8_t *data, size_t length) {
  if (!telnetOutputEnabled) {
    return;
  }

  portENTER_CRITICAL(&telnetOutputMux);
  for (size_t i = 0; i < length; i++) {
    size_t next = (telnetOutputHead + 1) % TELNET_OUTPUT_BUFFER_SIZE;
    if (next == telnetOutputTail) {
      break;
    }
    telnetOutputBuffer[telnetOutputHead] = data[i];
    telnetOutputHead = next;
  }
  portEXIT_CRITICAL(&telnetOutputMux);
}

class MirroredConsole : public Stream {
 public:
  void begin(unsigned long baud) {
    UsbSerialPort.begin(baud);
  }

  int available() override {
    return UsbSerialPort.available();
  }

  int read() override {
    return UsbSerialPort.read();
  }

  int peek() override {
    return UsbSerialPort.peek();
  }

  void flush() override {
    UsbSerialPort.flush();
  }

  size_t write(uint8_t value) override {
    UsbSerialPort.write(value);
    queueTelnetOutput(&value, 1);
    return 1;
  }

  size_t write(const uint8_t *buffer, size_t size) override {
    UsbSerialPort.write(buffer, size);
    queueTelnetOutput(buffer, size);
    return size;
  }

  using Print::write;
};

static MirroredConsole DebugConsole;
#define Serial DebugConsole

static constexpr int UART_TX_PIN = 17;
static constexpr int UART_RX_PIN = 18;
static constexpr uint32_t UART_BAUD = 115200;
static constexpr uint32_t USB_BAUD = 115200;
static constexpr uint32_t SERIAL0_MIRROR_BAUD = 115200;
static constexpr char WIFI_SSID[] = "DollNet";
static constexpr char WIFI_PASSWORD[] = "WD10ears!";
static constexpr uint16_t TELNET_PORT = 23;
static constexpr uint32_t WIFI_RETRY_INTERVAL_MS = 15000;
static constexpr uint32_t SCAN_TIME_MS = 5000;
static constexpr uint32_t SCAN_RESTART_DELAY_MS = 500;
// Longer gap between scans once something is already connected: the scanner and
// a live connection share one radio, and hammering it while a controller is in
// use costs input latency. Only relevant while a slot is still free.
static constexpr uint32_t SCAN_IDLE_DELAY_MS = 10000;
static constexpr uint32_t CONNECT_RETRY_DELAY_MS = 1000;
static constexpr uint32_t DISCONNECTED_LOG_INTERVAL_MS = 5000;
static constexpr uint32_t STATUS_DISPLAY_REFRESH_MS = 250;
static constexpr uint8_t STATUS_LED_COUNT = 1;
static constexpr uint8_t STATUS_LED_BRIGHTNESS = 48;
static constexpr uint16_t SERIAL_FLASH_MS = 10;
static constexpr uint8_t MAX_SAVED_KEYBOARDS = 8;
static constexpr const char *KEYBOARD_STORE_PATH = "/keyboards.tsv";
static constexpr uint16_t APPEARANCE_GAMEPAD = 0x03C4;   // BLE "HID Gamepad"; what an Xbox pad advertises
static constexpr uint32_t PAD_REPEAT_DELAY_MS = 400;     // held direction -> first auto-repeat
static constexpr uint32_t PAD_REPEAT_INTERVAL_MS = 110;  // ...and every one after that

#ifndef STATUS_LED_PIN
#define STATUS_LED_PIN 48
#endif

#ifndef SLAVE_OLED_ENABLED
#define SLAVE_OLED_ENABLED 1
#endif

#if SLAVE_OLED_ENABLED
#ifndef SLAVE_OLED_ADDRESS
#define SLAVE_OLED_ADDRESS 0x3C
#endif
#ifndef SLAVE_OLED_SDA_PIN
#define SLAVE_OLED_SDA_PIN 8
#endif
#ifndef SLAVE_OLED_SCL_PIN
#define SLAVE_OLED_SCL_PIN 9
#endif
#ifndef SLAVE_OLED_WIDTH
#define SLAVE_OLED_WIDTH 128
#endif
#ifndef SLAVE_OLED_HEIGHT
#define SLAVE_OLED_HEIGHT 64
#endif
#endif

#ifndef SLAVE_ROTARY_ENCODER_ENABLED
#define SLAVE_ROTARY_ENCODER_ENABLED 1
#endif

#if SLAVE_ROTARY_ENCODER_ENABLED
#ifndef SLAVE_ROTARY_A_PIN
#define SLAVE_ROTARY_A_PIN 6       // Encoder CLK / quadrature channel A.
#endif
#ifndef SLAVE_ROTARY_B_PIN
#define SLAVE_ROTARY_B_PIN 7       // Encoder DT / quadrature channel B.
#endif
#ifndef SLAVE_ROTARY_BUTTON_PIN
#define SLAVE_ROTARY_BUTTON_PIN 5  // Encoder SW / shaft push-button signal.
#endif
#endif

#ifndef SLAVE_JOYSTICK_ENABLED
#define SLAVE_JOYSTICK_ENABLED 1
#endif

#if SLAVE_JOYSTICK_ENABLED
// Both axes must sit on ADC1 (GPIO1-10 on the S3): ADC2 is owned by the radio
// while WiFi is up, and this sketch keeps WiFi up for Telnet.
#ifndef SLAVE_JOYSTICK_X_PIN
#define SLAVE_JOYSTICK_X_PIN 1     // Joystick VRx / analog X axis.
#endif
#ifndef SLAVE_JOYSTICK_Y_PIN
#define SLAVE_JOYSTICK_Y_PIN 2     // Joystick VRy / analog Y axis.
#endif
#ifndef SLAVE_JOYSTICK_BUTTON_PIN
#define SLAVE_JOYSTICK_BUTTON_PIN 4  // Joystick SW / stick push-button signal.
#endif
#endif

#ifndef SLAVE_BUTTON_BAR_ENABLED
#define SLAVE_BUTTON_BAR_ENABLED 1
#endif

#if SLAVE_BUTTON_BAR_ENABLED
// Plain digital inputs, so any free pin will do. These three avoid the
// strapping pins (0, 3, 45, 46), the octal PSRAM's (33-37), and everything the
// UART, USB, I2C, encoder, and joystick already hold.
// Left to right the bar reads Start, B, A -- Start outermost like a Game Boy's
// pair of pill buttons, and B left of A the way a Game Boy's face buttons sit,
// so gameplay lands where a thumb expects. Pins ascend in that same order, so the
// bar can be wired straight across. Get A and Start crossed and the symptom is
// next and previous station (or track) swapping ends; the fix belongs here or in
// the loom, never on the DOLL-OS side, which only ever sees the button names.
#ifndef SLAVE_BUTTON_A_PIN
#define SLAVE_BUTTON_A_PIN 12      // Right button: Game Boy A / next track-station.
#endif
#ifndef SLAVE_BUTTON_B_PIN
#define SLAVE_BUTTON_B_PIN 11      // Middle button: Game Boy B / pause-resume-stop.
#endif
#ifndef SLAVE_BUTTON_C_PIN
#define SLAVE_BUTTON_C_PIN 10      // Left button: Game Boy Start / previous track-station.
#endif
#endif

static const NimBLEUUID HID_SERVICE_UUID((uint16_t)0x1812);
static const NimBLEUUID REPORT_UUID((uint16_t)0x2A4D);
static const NimBLEUUID REPORT_REF_UUID((uint16_t)0x2908);
static const NimBLEUUID PROTOCOL_MODE_UUID((uint16_t)0x2A4E);
static const NimBLEUUID BOOT_KEYBOARD_INPUT_UUID((uint16_t)0x2A22);
static const NimBLEUUID BOOT_KEYBOARD_OUTPUT_UUID((uint16_t)0x2A32);

HardwareSerial LinkSerial(1);
CRGB statusLed[STATUS_LED_COUNT];
WiFiServer telnetServer(TELNET_PORT);
WiFiClient telnetClient;

// Peer and InputReportRef are defined at the top of the file; see the note there.
static Peer peers[MAX_PEERS];
static Peer usbKeyboardPeer;
static volatile bool usbKeyboardConnected = false;

// The analog joystick is not a HID device, but it produces exactly what a pad's
// left stick produces, so it borrows a Peer to hold that state and then goes
// through the same padApplyState() path. That way it inherits the direction
// hysteresis, the keystroke auto-repeat, and the merged game-mode bitmap for
// free, and nothing downstream has to learn that a third input exists.
static Peer joystickPeer;
static volatile bool joystickPresent = false;

// Same trick for the three-button bar: its own Peer, so its buttons merge with
// everything else's instead of overwriting them, and it reaches the DS through
// the one path every other input already uses.
static Peer buttonBarPeer;

// True while a connection is being established. Only one can be in flight at a
// time: the radio can only initiate one, and the scanner has to be stopped while
// it happens. Set by beginConnect, cleared by the connect/fail/disconnect
// callbacks. What the scan found is copied straight into the peer slot rather
// than parked in globals, because a NimBLEAdvertisedDevice belongs to the scan
// results and does not survive the next scan.
static volatile bool connectPending = false;

struct SavedKeyboard {
  String address;
  uint8_t addressType;
  String name;
};

static volatile bool shouldScan = true;
static volatile bool pairingMode = false;
static volatile bool gameMode = false;   // declared early: baseStatusColor() (below) reads it

static volatile PeerKind peerKindForced = PEER_UNKNOWN;   // PEER_UNKNOWN = auto-detect
static volatile bool reportDump = false;                  // "DUMP 1": hex-log every input report

static volatile uint8_t ledMask = 0;
static volatile bool capsLockOn = false;
static volatile bool ledMaskWritePending = false;
static String uartLine;
static String consoleLine;
static String telnetLine;
static CRGB currentStatusColor = CRGB::Black;
static volatile uint8_t statusFlashCode = 0;
static volatile uint32_t statusFlashUntil = 0;
static SavedKeyboard savedKeyboards[MAX_SAVED_KEYBOARDS];
static uint8_t savedKeyboardCount = 0;
static bool spiffsReady = false;
static uint32_t lastDisconnectedLogAt = 0;
static uint32_t nextScanAt = 0;
static uint32_t nextWiFiAttemptAt = 0;
static wl_status_t previousWiFiStatus = WL_NO_SHIELD;
static bool telnetServerRunning = false;
static uint8_t telnetParserState = 0;
static uint8_t telnetPendingCommand = 0;
static bool telnetSawCarriageReturn = false;
static uint32_t lastStatusDisplayAt = 0;
static uint32_t lastStatusDisplayHash = 0;
// Terminate sits next to Game Mode because the two are the DS-facing pair: one
// changes what input means, the other gets you out of whatever is consuming it.
// It matters most on a build with no keyboard attached -- a bar-launched game or
// a runaway .dapp would otherwise have no exit at all (see rotarySettingsApply).
enum RotarySetting : uint8_t {
  ROTARY_SETTING_PAIR = 0,
  ROTARY_SETTING_GAME_MODE,
  ROTARY_SETTING_TERMINATE,
  ROTARY_SETTING_RECONNECT,
  ROTARY_SETTING_SLEEP,
  ROTARY_SETTING_EXIT,
  ROTARY_SETTING_COUNT
};
static bool rotarySettingsActive = false;
static RotarySetting rotarySetting = ROTARY_SETTING_PAIR;
#if SLAVE_ROTARY_ENCODER_ENABLED
static uint8_t rotaryEncoderLastState = 0;
static int8_t rotaryEncoderAccumulator = 0;
static bool rotaryButtonRawPressed = false;
static bool rotaryButtonPressed = false;
static uint32_t rotaryButtonChangedAt = 0;
#endif

static void startScan();
static void printCommandHelp();
static void listSavedKeyboards();
static void clearSavedKeyboards();
static const char *bleErrorText(int error);
static bool configurePeer(Peer &peer);
static void rememberPeer(Peer &peer);
static void releasePeer(Peer &peer);
static void usbKeyboardHostBegin();
static void usbKeyboardQueueLedMask(uint8_t mask);
static void applyGameMode(bool on, const char *reason);
static void statusDisplayBegin();
static void updateStatusDisplay(bool force);
static void statusDisplayRefresh();
static void statusDisplayPowerOff();
static void rotaryEncoderBegin();
static void serviceRotaryEncoder();
static void joystickBegin();
static void serviceJoystick();
static const char *joystickStateName();
static void buttonBarBegin();
static void serviceButtonBar();
static void buttonBarEmitLinkBytes(Peer &peer, uint32_t buttons);

static uint8_t connectedPeerCount() {
  uint8_t count = 0;
  for (Peer &peer : peers) {
    if (peer.connected) {
      count++;
    }
  }
  return count;
}

static uint8_t claimedPeerCount() {
  uint8_t count = 0;
  for (Peer &peer : peers) {
    if (peer.inUse) {
      count++;
    }
  }
  return count;
}

// True if some device in the saved registry is not currently in a slot. Used to
// decide whether an empty slot is worth scanning for -- see wantMorePeers().
static bool savedDeviceMissing();

// Whether to keep looking for devices. A free slot alone is not reason enough:
// the scanner and a live connection share one radio, so scanning on a 15-second
// cycle forever would add input latency to a controller that is already
// connected, in the hope of a second device that may not exist. So scan while
// nothing is connected at all, while pairing was explicitly asked for, or while
// a device we know about is missing -- which is exactly the keyboard-plus-pad
// case, where both are in the registry and only one has turned up so far.
static bool wantMorePeers() {
  if (claimedPeerCount() >= MAX_PEERS) {
    return false;
  }
  if (connectedPeerCount() == 0) {
    return true;
  }
  return pairingMode || savedDeviceMissing();
}

static bool anyPeerOfKind(PeerKind kind) {
  if (kind == PEER_KEYBOARD && usbKeyboardConnected) {
    return true;
  }
  for (Peer &peer : peers) {
    if (peer.connected && peer.kind == kind) {
      return true;
    }
  }
  return false;
}

static CRGB baseStatusColor() {
  if (gameMode) {
    return CRGB::Cyan;
  }

  const bool keyboard = anyPeerOfKind(PEER_KEYBOARD);
  const bool gamepad = anyPeerOfKind(PEER_GAMEPAD);

  // The colour says what is connected, because which devices the bridge thinks
  // it has decides how every report is parsed -- and a pad mistaken for a
  // keyboard just looks like a dead controller.
  //
  // A pad makes the colour warm: orange on its own, yellow with a keyboard
  // alongside it. Not white for the pair -- white is the one state that lights
  // all three dies at once, so it draws three times the current and renders with
  // a pink or blue cast on a WS2812 instead of reading as neutral.
  if (keyboard && gamepad) {
    return CRGB::Yellow;
  }
  if (gamepad) {
    return CRGB::Orange;
  }
  if (keyboard || connectedPeerCount() > 0) {
    return CRGB::Magenta;
  }
  return CRGB::Red;
}

static void setStatusColor(CRGB color) {
  if (currentStatusColor == color) {
    return;
  }

  statusLed[0] = color;
  currentStatusColor = color;
  FastLED.show();
}

static void requestStatusFlash(uint8_t code) {
  statusFlashCode = code;
  statusFlashUntil = millis() + SERIAL_FLASH_MS;
}

static void updateStatusLed() {
  uint8_t flashCode = statusFlashCode;
  if (flashCode != 0 && int32_t(millis() - statusFlashUntil) >= 0) {
    statusFlashCode = 0;
    flashCode = 0;
  }

  if (flashCode == 1) {
    setStatusColor(CRGB::Blue);
  } else if (flashCode == 2) {
    setStatusColor(CRGB::Green);
  } else {
    setStatusColor(baseStatusColor());
  }
}

#if SLAVE_OLED_ENABLED
static constexpr uint8_t STATUS_DISPLAY_WIDTH = SLAVE_OLED_WIDTH;
static constexpr uint8_t STATUS_DISPLAY_HEIGHT = SLAVE_OLED_HEIGHT;
static constexpr uint8_t STATUS_DISPLAY_COLUMNS = STATUS_DISPLAY_WIDTH / 6;
static constexpr uint8_t STATUS_DISPLAY_ROWS = STATUS_DISPLAY_HEIGHT / 8;
static constexpr size_t STATUS_DISPLAY_BUFFER_SIZE =
  (size_t(STATUS_DISPLAY_WIDTH) * STATUS_DISPLAY_HEIGHT) / 8;
static constexpr bool OLED_BLACK = false;
static constexpr bool OLED_WHITE = true;
static constexpr bool OLED_RED = true;
static constexpr bool OLED_GREEN = true;
static constexpr bool OLED_CYAN = true;
static constexpr bool OLED_YELLOW = true;
static constexpr bool OLED_MAGENTA = true;
static constexpr bool OLED_GRAY = true;

static uint8_t statusDisplayBuffer[STATUS_DISPLAY_BUFFER_SIZE];
static bool statusDisplayReady = false;
static uint32_t nextStatusDisplayProbeAt = 0;

// Standard 5x7 terminal font, packed as five vertical columns per ASCII char.
static const uint8_t STATUS_FONT_5X7[] PROGMEM = {
  0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x5F,0x00,0x00, 0x00,0x07,0x00,0x07,0x00, 0x14,0x7F,0x14,0x7F,0x14,
  0x24,0x2A,0x7F,0x2A,0x12, 0x23,0x13,0x08,0x64,0x62, 0x36,0x49,0x55,0x22,0x50, 0x00,0x05,0x03,0x00,0x00,
  0x00,0x1C,0x22,0x41,0x00, 0x00,0x41,0x22,0x1C,0x00, 0x14,0x08,0x3E,0x08,0x14, 0x08,0x08,0x3E,0x08,0x08,
  0x00,0x50,0x30,0x00,0x00, 0x08,0x08,0x08,0x08,0x08, 0x00,0x60,0x60,0x00,0x00, 0x20,0x10,0x08,0x04,0x02,
  0x3E,0x51,0x49,0x45,0x3E, 0x00,0x42,0x7F,0x40,0x00, 0x42,0x61,0x51,0x49,0x46, 0x21,0x41,0x45,0x4B,0x31,
  0x18,0x14,0x12,0x7F,0x10, 0x27,0x45,0x45,0x45,0x39, 0x3C,0x4A,0x49,0x49,0x30, 0x01,0x71,0x09,0x05,0x03,
  0x36,0x49,0x49,0x49,0x36, 0x06,0x49,0x49,0x29,0x1E, 0x00,0x36,0x36,0x00,0x00, 0x00,0x56,0x36,0x00,0x00,
  0x08,0x14,0x22,0x41,0x00, 0x14,0x14,0x14,0x14,0x14, 0x00,0x41,0x22,0x14,0x08, 0x02,0x01,0x51,0x09,0x06,
  0x32,0x49,0x79,0x41,0x3E, 0x7E,0x11,0x11,0x11,0x7E, 0x7F,0x49,0x49,0x49,0x36, 0x3E,0x41,0x41,0x41,0x22,
  0x7F,0x41,0x41,0x22,0x1C, 0x7F,0x49,0x49,0x49,0x41, 0x7F,0x09,0x09,0x09,0x01, 0x3E,0x41,0x49,0x49,0x7A,
  0x7F,0x08,0x08,0x08,0x7F, 0x00,0x41,0x7F,0x41,0x00, 0x20,0x40,0x41,0x3F,0x01, 0x7F,0x08,0x14,0x22,0x41,
  0x7F,0x40,0x40,0x40,0x40, 0x7F,0x02,0x0C,0x02,0x7F, 0x7F,0x04,0x08,0x10,0x7F, 0x3E,0x41,0x41,0x41,0x3E,
  0x7F,0x09,0x09,0x09,0x06, 0x3E,0x41,0x51,0x21,0x5E, 0x7F,0x09,0x19,0x29,0x46, 0x46,0x49,0x49,0x49,0x31,
  0x01,0x01,0x7F,0x01,0x01, 0x3F,0x40,0x40,0x40,0x3F, 0x1F,0x20,0x40,0x20,0x1F, 0x3F,0x40,0x38,0x40,0x3F,
  0x63,0x14,0x08,0x14,0x63, 0x07,0x08,0x70,0x08,0x07, 0x61,0x51,0x49,0x45,0x43, 0x00,0x7F,0x41,0x41,0x00,
  0x02,0x04,0x08,0x10,0x20, 0x00,0x41,0x41,0x7F,0x00, 0x04,0x02,0x01,0x02,0x04, 0x40,0x40,0x40,0x40,0x40,
  0x00,0x01,0x02,0x04,0x00, 0x20,0x54,0x54,0x54,0x78, 0x7F,0x48,0x44,0x44,0x38, 0x38,0x44,0x44,0x44,0x20,
  0x38,0x44,0x44,0x48,0x7F, 0x38,0x54,0x54,0x54,0x18, 0x08,0x7E,0x09,0x01,0x02, 0x0C,0x52,0x52,0x52,0x3E,
  0x7F,0x08,0x04,0x04,0x78, 0x00,0x44,0x7D,0x40,0x00, 0x20,0x40,0x44,0x3D,0x00, 0x7F,0x10,0x28,0x44,0x00,
  0x00,0x41,0x7F,0x40,0x00, 0x7C,0x04,0x18,0x04,0x78, 0x7C,0x08,0x04,0x04,0x78, 0x38,0x44,0x44,0x44,0x38,
  0x7C,0x14,0x14,0x14,0x08, 0x08,0x14,0x14,0x18,0x7C, 0x7C,0x08,0x04,0x04,0x08, 0x48,0x54,0x54,0x54,0x20,
  0x04,0x3F,0x44,0x40,0x20, 0x3C,0x40,0x40,0x20,0x7C, 0x1C,0x20,0x40,0x20,0x1C, 0x3C,0x40,0x30,0x40,0x3C,
  0x44,0x28,0x10,0x28,0x44, 0x0C,0x50,0x50,0x50,0x3C, 0x44,0x64,0x54,0x4C,0x44, 0x00,0x08,0x36,0x41,0x00,
  0x00,0x00,0x7F,0x00,0x00, 0x00,0x41,0x36,0x08,0x00, 0x10,0x08,0x08,0x10,0x08, 0x00,0x00,0x00,0x00,0x00
};

static bool statusDisplayWriteCommand(uint8_t command) {
  Wire.beginTransmission(SLAVE_OLED_ADDRESS);
  Wire.write(0x00);
  Wire.write(command);
  return Wire.endTransmission() == 0;
}

static bool statusDisplayWriteCommand(uint8_t command, uint8_t argument) {
  Wire.beginTransmission(SLAVE_OLED_ADDRESS);
  Wire.write(0x00);
  Wire.write(command);
  Wire.write(argument);
  return Wire.endTransmission() == 0;
}

static bool statusDisplayWriteCommand(uint8_t command, uint8_t arg0, uint8_t arg1) {
  Wire.beginTransmission(SLAVE_OLED_ADDRESS);
  Wire.write(0x00);
  Wire.write(command);
  Wire.write(arg0);
  Wire.write(arg1);
  return Wire.endTransmission() == 0;
}

static void statusDisplayFillRect(
  uint8_t x,
  uint8_t y,
  uint8_t width,
  uint8_t height,
  bool color
) {
  if (!statusDisplayReady || width == 0 || height == 0) {
    return;
  }

  uint16_t xLimit = uint16_t(x) + width;
  uint16_t yLimit = uint16_t(y) + height;
  if (xLimit > STATUS_DISPLAY_WIDTH) {
    xLimit = STATUS_DISPLAY_WIDTH;
  }
  if (yLimit > STATUS_DISPLAY_HEIGHT) {
    yLimit = STATUS_DISPLAY_HEIGHT;
  }
  const uint8_t xEnd = uint8_t(xLimit);
  const uint8_t yEnd = uint8_t(yLimit);
  for (uint8_t pixelY = y; pixelY < yEnd; pixelY++) {
    const uint8_t mask = 1 << (pixelY & 0x07);
    const size_t rowOffset = size_t(pixelY >> 3) * STATUS_DISPLAY_WIDTH;
    for (uint8_t pixelX = x; pixelX < xEnd; pixelX++) {
      uint8_t &cell = statusDisplayBuffer[rowOffset + pixelX];
      if (color) {
        cell |= mask;
      } else {
        cell &= ~mask;
      }
    }
  }
}

static void statusDisplayDrawChar(uint8_t column, uint8_t row, char ch, bool fg, bool bg) {
  if (!statusDisplayReady || column >= STATUS_DISPLAY_COLUMNS || row >= STATUS_DISPLAY_ROWS) {
    return;
  }
  if (ch < 32 || ch > 127) {
    ch = '?';
  }

  uint8_t glyph[5];
  const uint16_t offset = uint16_t(ch - 32) * 5;
  for (uint8_t i = 0; i < 5; i++) {
    glyph[i] = pgm_read_byte(&STATUS_FONT_5X7[offset + i]);
  }

  const uint8_t x = column * 6;
  const uint8_t y = row * 8;
  for (uint8_t pixelY = 0; pixelY < 8; pixelY++) {
    for (uint8_t pixelX = 0; pixelX < 6; pixelX++) {
      const bool lit = pixelX < 5 && pixelY < 7 && (glyph[pixelX] & (1 << pixelY));
      statusDisplayFillRect(x + pixelX, y + pixelY, 1, 1, lit ? fg : bg);
    }
  }
}

static void statusDisplayDrawLine(uint8_t row, const char *text, bool fg, bool bg) {
  char padded[STATUS_DISPLAY_COLUMNS + 1];
  uint8_t i = 0;
  for (; i < STATUS_DISPLAY_COLUMNS && text[i] != '\0'; i++) {
    padded[i] = text[i];
  }
  for (; i < STATUS_DISPLAY_COLUMNS; i++) {
    padded[i] = ' ';
  }
  padded[STATUS_DISPLAY_COLUMNS] = '\0';

  for (uint8_t column = 0; column < STATUS_DISPLAY_COLUMNS; column++) {
    statusDisplayDrawChar(column, row, padded[column], fg, bg);
  }
}

static bool statusDisplayFlush() {
  if (!statusDisplayReady) {
    return false;
  }

  if (!statusDisplayWriteCommand(0x21, 0, STATUS_DISPLAY_WIDTH - 1) ||
      !statusDisplayWriteCommand(0x22, 0, (STATUS_DISPLAY_HEIGHT / 8) - 1)) {
    return false;
  }

  for (size_t offset = 0; offset < STATUS_DISPLAY_BUFFER_SIZE; offset += 16) {
    Wire.beginTransmission(SLAVE_OLED_ADDRESS);
    Wire.write(0x40);
    for (uint8_t i = 0; i < 16 && offset + i < STATUS_DISPLAY_BUFFER_SIZE; i++) {
      Wire.write(statusDisplayBuffer[offset + i]);
    }
    if (Wire.endTransmission() != 0) {
      return false;
    }
  }
  return true;
}

static bool statusDisplayInitPanel() {
  return statusDisplayWriteCommand(0xAE) &&       // display off
         statusDisplayWriteCommand(0xD5, 0x80) && // clock
         statusDisplayWriteCommand(0xA8, STATUS_DISPLAY_HEIGHT - 1) &&
         statusDisplayWriteCommand(0xD3, 0x00) && // display offset
         statusDisplayWriteCommand(0x40) &&       // start line
         statusDisplayWriteCommand(0x8D, 0x14) && // charge pump on
         statusDisplayWriteCommand(0x20, 0x00) && // horizontal memory mode
         statusDisplayWriteCommand(0xA1) &&       // segment remap
         statusDisplayWriteCommand(0xC8) &&       // COM scan direction
         statusDisplayWriteCommand(0xDA, STATUS_DISPLAY_HEIGHT == 64 ? 0x12 : 0x02) &&
         statusDisplayWriteCommand(0x81, 0xCF) && // contrast
         statusDisplayWriteCommand(0xD9, 0xF1) && // precharge
         statusDisplayWriteCommand(0xDB, 0x40) && // VCOM detect
         statusDisplayWriteCommand(0xA4) &&       // resume RAM display
         statusDisplayWriteCommand(0xA6) &&       // normal polarity
         statusDisplayWriteCommand(0x2E) &&       // deactivate scroll
         statusDisplayWriteCommand(0xAF);         // display on
}

static char statusDisplayKindChar(PeerKind kind) {
  if (kind == PEER_KEYBOARD) {
    return 'K';
  }
  if (kind == PEER_GAMEPAD) {
    return 'G';
  }
  return '?';
}

static void statusDisplayPeerLine(uint8_t slot, char *line, size_t lineSize) {
  const Peer &peer = peers[slot];
  if (!peer.inUse) {
    snprintf(line, lineSize, "P%u -- FREE", slot + 1);
    return;
  }

  const char *state = peer.connected ? "OK" : peer.needsConfigure ? "CFG" : "LINK";
  const char *name = peer.name.length() > 0 ? peer.name.c_str() : "";
  snprintf(line, lineSize, "P%u %c %-4s %.7s",
           slot + 1, statusDisplayKindChar(peer.kind), state, name);
}

static uint32_t statusDisplayHashAdd(uint32_t hash, uint32_t value) {
  return (hash ^ value) * 16777619UL;
}

static uint32_t statusDisplayStateHash() {
  NimBLEScan *scan = NimBLEDevice::getScan();
  uint32_t hash = 2166136261UL;
  hash = statusDisplayHashAdd(hash, rotarySettingsActive ? 1 : 0);
  hash = statusDisplayHashAdd(hash, rotarySetting);
  hash = statusDisplayHashAdd(hash, pairingMode ? 1 : 0);
  hash = statusDisplayHashAdd(hash, connectPending ? 1 : 0);
  hash = statusDisplayHashAdd(hash, shouldScan ? 1 : 0);
  hash = statusDisplayHashAdd(hash, scan->isScanning() ? 1 : 0);
  hash = statusDisplayHashAdd(hash, gameMode ? 1 : 0);
  hash = statusDisplayHashAdd(hash, usbKeyboardConnected ? 1 : 0);
  hash = statusDisplayHashAdd(hash, connectedPeerCount());
  hash = statusDisplayHashAdd(hash, claimedPeerCount());
  hash = statusDisplayHashAdd(hash, savedKeyboardCount);
  hash = statusDisplayHashAdd(hash, ledMask);
  hash = statusDisplayHashAdd(hash, WiFi.status());
  hash = statusDisplayHashAdd(hash, telnetClient && telnetClient.connected() ? 1 : 0);
  for (Peer &peer : peers) {
    hash = statusDisplayHashAdd(hash, peer.inUse ? 1 : 0);
    hash = statusDisplayHashAdd(hash, peer.connected ? 1 : 0);
    hash = statusDisplayHashAdd(hash, peer.needsConfigure ? 1 : 0);
    hash = statusDisplayHashAdd(hash, peer.kind);
    hash = statusDisplayHashAdd(hash, peer.reportCount);
    hash = statusDisplayHashAdd(hash, peer.name.length());
    for (uint8_t i = 0; i < 4 && i < peer.name.length(); i++) {
      hash = statusDisplayHashAdd(hash, uint8_t(peer.name[i]));
    }
  }
  return hash;
}

static bool statusDisplayProbe(uint8_t attempts) {
  for (uint8_t i = 0; i < attempts; i++) {
    Wire.beginTransmission(SLAVE_OLED_ADDRESS);
    if (Wire.endTransmission() == 0) {
      return true;
    }
    delay(25);
  }
  return false;
}

static void statusDisplayMarkLost() {
  if (statusDisplayReady) {
    Serial.println("Status OLED I2C write failed; will retry");
  }
  statusDisplayReady = false;
  nextStatusDisplayProbeAt = millis() + 1000;
}

static bool statusDisplayStart(bool quiet, uint8_t probeAttempts) {
  if (SLAVE_OLED_SDA_PIN >= 0 && SLAVE_OLED_SCL_PIN >= 0) {
    Wire.begin(SLAVE_OLED_SDA_PIN, SLAVE_OLED_SCL_PIN);
  } else {
    Wire.begin();
  }
  Wire.setClock(400000);

  if (!statusDisplayProbe(probeAttempts)) {
    statusDisplayReady = false;
    nextStatusDisplayProbeAt = millis() + 1000;
    if (!quiet) {
      Serial.printf("Status OLED SSD1306 not found at I2C address 0x%02X\r\n",
                    uint8_t(SLAVE_OLED_ADDRESS));
    }
    return false;
  }

  statusDisplayReady = true;
  if (!statusDisplayInitPanel()) {
    statusDisplayMarkLost();
    return false;
  }

  statusDisplayFillRect(0, 0, STATUS_DISPLAY_WIDTH, STATUS_DISPLAY_HEIGHT, OLED_BLACK);
  statusDisplayDrawLine(0, "DS-SLAVE", OLED_BLACK, OLED_WHITE);
  statusDisplayDrawLine(1, "OLED READY", OLED_GREEN, OLED_BLACK);
  if (!statusDisplayFlush()) {
    statusDisplayMarkLost();
    return false;
  }

  lastStatusDisplayHash = 0;
  Serial.printf("Status OLED SSD1306 enabled: address=0x%02X SDA=%d SCL=%d\r\n",
                uint8_t(SLAVE_OLED_ADDRESS), SLAVE_OLED_SDA_PIN, SLAVE_OLED_SCL_PIN);
  return true;
}

static void statusDisplayBegin() {
  statusDisplayStart(false, 8);
}

static void updateStatusDisplay(bool force) {
  if (!statusDisplayReady) {
    const uint32_t now = millis();
    if (int32_t(now - nextStatusDisplayProbeAt) >= 0) {
      nextStatusDisplayProbeAt = now + 1000;
      statusDisplayStart(true, 1);
    }
    return;
  }

  const uint32_t now = millis();
  if (!force && int32_t(now - lastStatusDisplayAt) < int32_t(STATUS_DISPLAY_REFRESH_MS)) {
    return;
  }
  lastStatusDisplayAt = now;

  const uint32_t hash = statusDisplayStateHash();
  if (!force && hash == lastStatusDisplayHash) {
    return;
  }
  lastStatusDisplayHash = hash;

  char line[STATUS_DISPLAY_COLUMNS + 1];
  const bool scanActive = NimBLEDevice::getScan()->isScanning();

  if (!rotarySettingsActive) {
    statusDisplayDrawLine(0, "VOLUME MODE", OLED_BLACK, OLED_WHITE);
    snprintf(line, sizeof(line), "PAIR %-3s SCAN %c",
             pairingMode ? "ON" : "OFF", scanActive ? '*' : '-');
    statusDisplayDrawLine(1, line, pairingMode ? OLED_YELLOW : OLED_WHITE, OLED_BLACK);
    snprintf(line, sizeof(line), "BLE %u/%u CLAIM %u",
             connectedPeerCount(), MAX_PEERS, claimedPeerCount());
    statusDisplayDrawLine(2, line, connectedPeerCount() > 0 ? OLED_GREEN : OLED_RED, OLED_BLACK);
    statusDisplayPeerLine(0, line, sizeof(line));
    statusDisplayDrawLine(3, line, peers[0].connected ? OLED_GREEN : peers[0].inUse ? OLED_YELLOW : OLED_GRAY, OLED_BLACK);
    statusDisplayPeerLine(1, line, sizeof(line));
    statusDisplayDrawLine(4, line, peers[1].connected ? OLED_GREEN : peers[1].inUse ? OLED_YELLOW : OLED_GRAY, OLED_BLACK);
    snprintf(line, sizeof(line), "USB %-3s GAME %s",
             usbKeyboardConnected ? "ON" : "OFF", gameMode ? "PAD" : "KEY");
    statusDisplayDrawLine(5, line, usbKeyboardConnected || gameMode ? OLED_CYAN : OLED_WHITE, OLED_BLACK);
    snprintf(line, sizeof(line), "WIFI %-2s TEL %-2s",
             WiFi.status() == WL_CONNECTED ? "OK" : "--",
             (telnetClient && telnetClient.connected()) ? "ON" : "--");
    statusDisplayDrawLine(6, line, WiFi.status() == WL_CONNECTED ? OLED_GREEN : OLED_GRAY, OLED_BLACK);
    snprintf(line, sizeof(line), "SAVE %-2u LED %02X", savedKeyboardCount, ledMask);
    statusDisplayDrawLine(7, line, OLED_MAGENTA, OLED_BLACK);
  } else {
    statusDisplayDrawLine(0, "SETTINGS", OLED_BLACK, OLED_WHITE);
    snprintf(line, sizeof(line), "%c PAIR DEVICE",
             rotarySetting == ROTARY_SETTING_PAIR ? '>' : ' ');
    statusDisplayDrawLine(1, line, rotarySetting == ROTARY_SETTING_PAIR ? OLED_BLACK : OLED_WHITE,
                         rotarySetting == ROTARY_SETTING_PAIR ? OLED_WHITE : OLED_BLACK);
    snprintf(line, sizeof(line), "%c GAME MODE: %s",
             rotarySetting == ROTARY_SETTING_GAME_MODE ? '>' : ' ', gameMode ? "ON" : "OFF");
    statusDisplayDrawLine(2, line, rotarySetting == ROTARY_SETTING_GAME_MODE ? OLED_BLACK : OLED_WHITE,
                         rotarySetting == ROTARY_SETTING_GAME_MODE ? OLED_WHITE : OLED_BLACK);
    snprintf(line, sizeof(line), "%c TERMINATE APP",
             rotarySetting == ROTARY_SETTING_TERMINATE ? '>' : ' ');
    statusDisplayDrawLine(3, line, rotarySetting == ROTARY_SETTING_TERMINATE ? OLED_BLACK : OLED_WHITE,
                         rotarySetting == ROTARY_SETTING_TERMINATE ? OLED_WHITE : OLED_BLACK);
    snprintf(line, sizeof(line), "%c RECONNECT",
             rotarySetting == ROTARY_SETTING_RECONNECT ? '>' : ' ');
    statusDisplayDrawLine(4, line, rotarySetting == ROTARY_SETTING_RECONNECT ? OLED_BLACK : OLED_WHITE,
                         rotarySetting == ROTARY_SETTING_RECONNECT ? OLED_WHITE : OLED_BLACK);
    snprintf(line, sizeof(line), "%c SLEEP",
             rotarySetting == ROTARY_SETTING_SLEEP ? '>' : ' ');
    statusDisplayDrawLine(5, line, rotarySetting == ROTARY_SETTING_SLEEP ? OLED_BLACK : OLED_WHITE,
                         rotarySetting == ROTARY_SETTING_SLEEP ? OLED_WHITE : OLED_BLACK);
    snprintf(line, sizeof(line), "%c EXIT",
             rotarySetting == ROTARY_SETTING_EXIT ? '>' : ' ');
    statusDisplayDrawLine(6, line, rotarySetting == ROTARY_SETTING_EXIT ? OLED_BLACK : OLED_WHITE,
                         rotarySetting == ROTARY_SETTING_EXIT ? OLED_WHITE : OLED_BLACK);
    //The sixth item claimed the row that used to repeat the BLE/USB counts. They are
    //still one click away on the volume screen, and inside a menu the list is what
    //the dial is actually for.
    const char *detail = rotarySetting == ROTARY_SETTING_PAIR ? "ADD BLE HID" :
                         rotarySetting == ROTARY_SETTING_GAME_MODE ? "TOGGLE INPUT MODE" :
                         rotarySetting == ROTARY_SETTING_TERMINATE ? "QUIT APP TO SHELL" :
                         rotarySetting == ROTARY_SETTING_RECONNECT ? "SCAN SAVED DEVICES" :
                         rotarySetting == ROTARY_SETTING_SLEEP ? "LOW POWER MODE" :
                         "NO CHANGES";
    statusDisplayDrawLine(7, detail, OLED_WHITE, OLED_BLACK);
  }
  if (!statusDisplayFlush()) {
    statusDisplayMarkLost();
  }
}

static const char *statusDisplayStateName() {
  return statusDisplayReady ? "ready" : "missing";
}

static void statusDisplayPowerOff() {
  if (statusDisplayReady) {
    statusDisplayWriteCommand(0xAE);  // SSD1306 display-off keeps the OLED dark in deep sleep.
  }
}
#else
static void statusDisplayBegin() {}
static void updateStatusDisplay(bool force) {
  (void)force;
}
static const char *statusDisplayStateName() {
  return "disabled";
}
static void statusDisplayPowerOff() {}
#endif

static void statusDisplayRefresh() {
  lastStatusDisplayHash = 0;
  updateStatusDisplay(true);
}

static void linkWrite(uint8_t value) {
  LinkSerial.write(value);
  Serial0.write(value);
  requestStatusFlash(1);
}

static void linkWrite(char value) {
  LinkSerial.write(value);
  Serial0.write(value);
  requestStatusFlash(1);
}

static void linkWrite(const uint8_t *data, size_t length) {
  LinkSerial.write(data, length);
  Serial0.write(data, length);
  requestStatusFlash(1);
}

static void linkWrite(const char *text) {
  linkWrite((const uint8_t *)text, strlen(text));
}

static constexpr uint8_t LINK_SYSTEM_SLEEP = 0xF6;
static constexpr uint8_t LINK_SYSTEM_WAKE = 0xF7;

// Button-bar presses, one private byte per press edge, sent only while game mode
// is off (in game mode the same switches are part of the merged Game Boy bitmap
// instead). DOLL-OS decides what each one means from whatever is using its audio
// and screen -- transport for the music player and the radio, app launchers on an
// idle shell -- so the bar needs no notion of context itself. See PadButtons.ino
// on the DS side; these four values are a paired protocol, flash both boards.
static constexpr uint8_t LINK_PAD_START = 0xF8;
static constexpr uint8_t LINK_PAD_SELECT = 0xF9;
static constexpr uint8_t LINK_PAD_B = 0xFA;
static constexpr uint8_t LINK_PAD_A = 0xFB;

// The Settings > Terminate pair. Real control codes, not private bytes: DOLL-OS
// already treats these as its two abort chords, so this reuses machinery every
// modal screen over there has rather than asking for a new protocol byte. They
// cover different apps -- ^X aborts a running .dapp and exits the editor, Ctrl+T
// quits the Game Boy emulator, the music player, and an ssh/telnet session -- so
// Terminate sends both. Order is deliberate: see sendAppTerminate().
static constexpr uint8_t LINK_APP_ABORT = 0x18;   // ^X
static constexpr uint8_t LINK_APP_QUIT = 0x14;    // Ctrl+T (DC4)

// Ask DOLL-OS to close whatever app is on its panel and go back to the shell.
//
// ^X leads for a reason: a running .dapp's abort check *peeks* at the head of the
// keyboard stream for it (appPollAbortChord, AppRunner.ino), and a Ctrl+T parked in
// front would sit there unread until the script asked for a key -- which a runaway
// script never does. With ^X first the dapp aborts immediately, and the Ctrl+T
// behind it reaches whatever else was running. Every app that honours one ignores
// the other, and with nothing running the DS line editor drops both, so a stray
// Terminate at the shell prompt does nothing.
static void sendAppTerminate() {
  linkWrite(LINK_APP_ABORT);
  linkWrite(LINK_APP_QUIT);
}

static void sendMainWakeBeacon() {
  // Several bytes cover the main unit's light-sleep wake latency; all are private controls.
  for (uint8_t i = 0; i < 8; i++) {
    LinkSerial.write(LINK_SYSTEM_WAKE);
    LinkSerial.flush();
    delay(4);
  }
}

#if SLAVE_ROTARY_ENCODER_ENABLED
static constexpr uint8_t LINK_VOLUME_UP = 0xF4;
static constexpr uint8_t LINK_VOLUME_DOWN = 0xF5;
static constexpr uint32_t ROTARY_BUTTON_DEBOUNCE_MS = 35;

static bool enterSlaveDeepSleep() {
  const gpio_num_t wakePin = gpio_num_t(SLAVE_ROTARY_BUTTON_PIN);
  const esp_err_t wakeResult = esp_sleep_enable_ext0_wakeup(wakePin, 0);
  if (wakeResult != ESP_OK) {
    Serial.printf("Sleep rejected: GPIO%d cannot wake from deep sleep (error %d)\r\n",
                  SLAVE_ROTARY_BUTTON_PIN, int(wakeResult));
    return false;
  }

  linkWrite(LINK_SYSTEM_SLEEP);  // Tell DOLL-OS to darken its panel and enter light sleep.
  LinkSerial.flush();
  Serial0.flush();
  Serial.println("Sleep requested; release the rotary button, then press it to wake");

  // The selecting press is still down here; sleeping before release would wake immediately.
  while (digitalRead(SLAVE_ROTARY_BUTTON_PIN) == LOW) {
    delay(10);
  }
  delay(ROTARY_BUTTON_DEBOUNCE_MS);

  rtc_gpio_pullup_en(wakePin);     // Keep the active-low rotary switch biased while asleep.
  rtc_gpio_pulldown_dis(wakePin);
  statusDisplayPowerOff();         // Shut down the secondary OLED before power domains fall.
  FastLED.clear(true);             // Leave the status pixel physically dark during deep sleep.
  WiFi.disconnect(true, false);    // Stop the slave radio cleanly before deep sleep powers it down.
  WiFi.mode(WIFI_OFF);

  // The flushed UART is idle-high; hold that pad state so the main RX cannot float and false-trigger.
  delay(1);
  gpio_hold_en(gpio_num_t(UART_TX_PIN));
  gpio_deep_sleep_hold_en();

  Serial.flush();
  esp_deep_sleep_start();          // Deep-sleep wake performs the requested full slave restart.
  return true;                     // Unreachable, but keeps the failure-capable API explicit.
}

static uint8_t readRotaryEncoderState() {
  return (digitalRead(SLAVE_ROTARY_A_PIN) ? 0x02 : 0x00) |
         (digitalRead(SLAVE_ROTARY_B_PIN) ? 0x01 : 0x00);
}

static void rotaryEncoderBegin() {
  pinMode(SLAVE_ROTARY_A_PIN, INPUT_PULLUP);
  pinMode(SLAVE_ROTARY_B_PIN, INPUT_PULLUP);
  pinMode(SLAVE_ROTARY_BUTTON_PIN, INPUT_PULLUP);
  rotaryEncoderLastState = readRotaryEncoderState();
  rotaryEncoderAccumulator = 0;
  rotaryButtonRawPressed = digitalRead(SLAVE_ROTARY_BUTTON_PIN) == LOW;
  rotaryButtonPressed = rotaryButtonRawPressed;
  rotaryButtonChangedAt = millis();
  Serial.printf("Volume encoder enabled: A=%d B=%d SW=%d%s\r\n",
                SLAVE_ROTARY_A_PIN, SLAVE_ROTARY_B_PIN, SLAVE_ROTARY_BUTTON_PIN,
                SLAVE_ROTARY_REVERSED ? " reversed" : "");
}

static void rotarySettingsEnter() {
  rotarySettingsActive = true;
  rotarySetting = ROTARY_SETTING_PAIR;
  rotaryEncoderAccumulator = 0;
  Serial.println("Rotary settings opened");
  statusDisplayRefresh();
}

static void rotarySettingsApply() {
  const RotarySetting selected = rotarySetting;
  rotarySettingsActive = false;
  rotaryEncoderAccumulator = 0;

  if (selected == ROTARY_SETTING_PAIR) {
    NimBLEScan *scan = NimBLEDevice::getScan();
    if (scan->isScanning()) {
      scan->stop();
    }
    if (claimedPeerCount() >= MAX_PEERS) {
      Serial.println("All peer slots are in use; pairing was not started");
    } else {
      pairingMode = true;
      shouldScan = true;
      nextScanAt = millis() + SCAN_RESTART_DELAY_MS;
      Serial.println("Pairing mode enabled from rotary settings");
    }
  } else if (selected == ROTARY_SETTING_GAME_MODE) {
    applyGameMode(!gameMode, "rotary settings");
  } else if (selected == ROTARY_SETTING_TERMINATE) {
    // Game mode is left alone on purpose: if the emulator is what quits, it sends
    // its own "GAME 0" on the way out, and flipping the mode from this side first
    // would change the byte vocabulary under the quit that is still in flight.
    sendAppTerminate();
    Serial.println("App terminate sent from rotary settings (^X + Ctrl+T)");
  } else if (selected == ROTARY_SETTING_RECONNECT) {
    NimBLEScan *scan = NimBLEDevice::getScan();
    if (scan->isScanning()) {
      scan->stop();
    }
    pairingMode = false;
    shouldScan = true;
    nextScanAt = millis() + SCAN_RESTART_DELAY_MS;
    Serial.println("Reconnect scan requested from rotary settings");
  } else if (selected == ROTARY_SETTING_SLEEP) {
    if (!enterSlaveDeepSleep()) {
      rotarySettingsActive = true;  // Leave the failed choice visible so the hardware error is clear.
    }
  } else {
    Serial.println("Rotary settings closed");
  }

  statusDisplayRefresh();
}

static void rotarySettingsMove(int8_t direction) {
  int8_t next = int8_t(rotarySetting) + direction;
  if (next < 0) {
    next = ROTARY_SETTING_COUNT - 1;
  } else if (next >= ROTARY_SETTING_COUNT) {
    next = 0;
  }
  rotarySetting = RotarySetting(next);
  statusDisplayRefresh();
}

static void serviceRotaryButton() {
  const bool rawPressed = digitalRead(SLAVE_ROTARY_BUTTON_PIN) == LOW;
  const uint32_t now = millis();
  if (rawPressed != rotaryButtonRawPressed) {
    rotaryButtonRawPressed = rawPressed;
    rotaryButtonChangedAt = now;
  }

  if (rawPressed != rotaryButtonPressed &&
      uint32_t(now - rotaryButtonChangedAt) >= ROTARY_BUTTON_DEBOUNCE_MS) {
    rotaryButtonPressed = rawPressed;
    if (rotaryButtonPressed) {
      if (rotarySettingsActive) {
        rotarySettingsApply();
      } else {
        rotarySettingsEnter();
      }
    }
  }
}

static void serviceRotaryEncoder() {
  static const int8_t transitions[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
  };

  serviceRotaryButton();

  const uint8_t state = readRotaryEncoderState();
  if (state == rotaryEncoderLastState) {
    return;
  }

  int8_t delta = transitions[(rotaryEncoderLastState << 2) | state];
  rotaryEncoderLastState = state;
  if (delta == 0) {
    return;
  }
  if (SLAVE_ROTARY_REVERSED) {
    delta = -delta;
  }

  rotaryEncoderAccumulator += delta;
  if (rotaryEncoderAccumulator >= 4) {
    rotaryEncoderAccumulator = 0;
    if (rotarySettingsActive) {
      rotarySettingsMove(1);
    } else {
      linkWrite(LINK_VOLUME_UP);
    }
  } else if (rotaryEncoderAccumulator <= -4) {
    rotaryEncoderAccumulator = 0;
    if (rotarySettingsActive) {
      rotarySettingsMove(-1);
    } else {
      linkWrite(LINK_VOLUME_DOWN);
    }
  }
}
#else
static void rotaryEncoderBegin() {}
static void serviceRotaryEncoder() {}
#endif

static bool keyWasDown(const Peer &peer, uint8_t key) {
  for (uint8_t i = 0; i < sizeof(peer.lastKeys); i++) {
    if (peer.lastKeys[i] == key) {
      return true;
    }
  }
  return false;
}

static void rememberKeys(Peer &peer, const uint8_t *keys) {
  for (uint8_t i = 0; i < sizeof(peer.lastKeys); i++) {
    peer.lastKeys[i] = keys[i];
  }
}

static bool shifted(uint8_t modifiers) {
  return (modifiers & ((1 << 1) | (1 << 5))) != 0;
}

static bool controlled(uint8_t modifiers) {
  return (modifiers & ((1 << 0) | (1 << 4))) != 0;
}

static bool alted(uint8_t modifiers) {
  return (modifiers & ((1 << 2) | (1 << 6))) != 0;
}

static bool commanded(uint8_t modifiers) {
  return (modifiers & ((1 << 3) | (1 << 7))) != 0;
}

static uint8_t terminalModifierParam(uint8_t modifiers) {
  uint8_t param = 1;
  if (shifted(modifiers)) {
    param += 1;
  }
  if (alted(modifiers)) {
    param += 2;
  }
  if (controlled(modifiers)) {
    param += 4;
  }
  if (commanded(modifiers)) {
    param += 8;
  }
  return param;
}

static void writeCsiU(uint16_t codepoint, uint8_t modifiers) {
  char seq[18];
  snprintf(seq, sizeof(seq), "\x1B[%u;%uu", codepoint, terminalModifierParam(modifiers));
  linkWrite(seq);
}

static void writeModifiedAscii(char c, uint8_t modifiers) {
  if (commanded(modifiers)) {
    writeCsiU(uint8_t(c), modifiers);
    return;
  }

  if (alted(modifiers)) {
    linkWrite(uint8_t(0x1B));
  }
  linkWrite(c);
}

static void writeModifiedControl(uint8_t code, uint8_t modifiers) {
  if (commanded(modifiers)) {
    writeCsiU(code, modifiers);
    return;
  }

  if (alted(modifiers)) {
    linkWrite(uint8_t(0x1B));
  }
  linkWrite(code);
}

static bool writeCtrlPunctuation(char c, uint8_t modifiers) {
  if (!controlled(modifiers) || commanded(modifiers)) {
    return false;
  }

  switch (c) {
    case ' ':
    case '@': writeModifiedControl(0x00, modifiers); return true;
    case '[': writeModifiedControl(0x1B, modifiers); return true;
    case '\\': writeModifiedControl(0x1C, modifiers); return true;
    case ']': writeModifiedControl(0x1D, modifiers); return true;
    case '^': writeModifiedControl(0x1E, modifiers); return true;
    case '_': writeModifiedControl(0x1F, modifiers); return true;
    case '?': writeModifiedControl(0x7F, modifiers); return true;
    default: return false;
  }
}

static void writeModifiedCsiFinal(const char *plain, char final, uint8_t modifiers) {
  uint8_t modParam = terminalModifierParam(modifiers);
  if (modParam == 1) {
    linkWrite(plain);
    return;
  }

  char seq[14];
  snprintf(seq, sizeof(seq), "\x1B[1;%u%c", modParam, final);
  linkWrite(seq);
}

static void writeModifiedCsiTilde(const char *plain, uint8_t code, uint8_t modifiers) {
  uint8_t modParam = terminalModifierParam(modifiers);
  if (modParam == 1) {
    linkWrite(plain);
    return;
  }

  char seq[16];
  snprintf(seq, sizeof(seq), "\x1B[%u;%u~", code, modParam);
  linkWrite(seq);
}

static void writeEsc(const char *seq) {
  linkWrite(seq);
}

static String sanitizeKeyboardName(String name) {
  name.trim();
  name.replace('\t', ' ');
  name.replace('\r', ' ');
  name.replace('\n', ' ');
  if (name.length() == 0) {
    return "(unnamed)";
  }
  return name;
}

static int findSavedKeyboard(const String &address) {
  for (uint8_t i = 0; i < savedKeyboardCount; i++) {
    if (savedKeyboards[i].address.equalsIgnoreCase(address)) {
      return i;
    }
  }
  return -1;
}

static bool keyboardMatchesSavedIdentity(const NimBLEAdvertisedDevice *device) {
  if (findSavedKeyboard(device->getAddress().toString().c_str()) >= 0) {
    return true;
  }

  if (!device->haveName()) {
    return false;
  }

  String name = sanitizeKeyboardName(device->getName().c_str());
  for (uint8_t i = 0; i < savedKeyboardCount; i++) {
    if (savedKeyboards[i].name != "(unnamed)" &&
        savedKeyboards[i].name.equalsIgnoreCase(name)) {
      return true;
    }
  }
  return false;
}

static bool saveKeyboardStore() {
  if (!spiffsReady) {
    Serial.println("SPIFFS is not mounted; keyboard registry not saved");
    return false;
  }

  File file = SPIFFS.open(KEYBOARD_STORE_PATH, FILE_WRITE);
  if (!file) {
    Serial.println("Could not open keyboard registry for writing");
    return false;
  }

  for (uint8_t i = 0; i < savedKeyboardCount; i++) {
    file.print(savedKeyboards[i].address);
    file.print('\t');
    file.print(savedKeyboards[i].addressType);
    file.print('\t');
    file.println(savedKeyboards[i].name);
  }

  file.close();
  return true;
}

static void loadKeyboardStore() {
  savedKeyboardCount = 0;
  if (!spiffsReady || !SPIFFS.exists(KEYBOARD_STORE_PATH)) {
    return;
  }

  File file = SPIFFS.open(KEYBOARD_STORE_PATH, FILE_READ);
  if (!file) {
    Serial.println("Could not open keyboard registry for reading");
    return;
  }

  while (file.available() && savedKeyboardCount < MAX_SAVED_KEYBOARDS) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) {
      continue;
    }

    int firstTab = line.indexOf('\t');
    int secondTab = firstTab < 0 ? -1 : line.indexOf('\t', firstTab + 1);
    if (firstTab < 0 || secondTab < 0) {
      continue;
    }

    SavedKeyboard &slot = savedKeyboards[savedKeyboardCount++];
    slot.address = line.substring(0, firstTab);
    slot.addressType = uint8_t(line.substring(firstTab + 1, secondTab).toInt());
    slot.name = sanitizeKeyboardName(line.substring(secondTab + 1));
  }

  file.close();
}

static void saveKeyboardIdentity(const NimBLEAddress &address, String name) {
  if (address.isNull()) {
    Serial.println("Connected keyboard has null identity; not saving");
    return;
  }

  String addressText = address.toString().c_str();
  name = sanitizeKeyboardName(name);
  int index = findSavedKeyboard(addressText);

  if (index < 0 && name != "(unnamed)") {
    for (uint8_t i = 0; i < savedKeyboardCount; i++) {
      if (savedKeyboards[i].name.equalsIgnoreCase(name)) {
        index = i;
        break;
      }
    }
  }

  if (index < 0) {
    if (savedKeyboardCount >= MAX_SAVED_KEYBOARDS) {
      Serial.println("Keyboard registry full; newest keyboard was not saved");
      return;
    }
    index = savedKeyboardCount++;
  }

  savedKeyboards[index].address = addressText;
  savedKeyboards[index].addressType = address.getType();
  savedKeyboards[index].name = name;

  if (saveKeyboardStore()) {
    Serial.print("Saved keyboard identity to SPIFFS: ");
    Serial.print(addressText);
    Serial.print(" type=");
    Serial.print(address.getType());
    Serial.print(" ");
    Serial.println(name);
  }
}

static void rememberPeer(Peer &peer) {
  if (peer.client == nullptr || !peer.client->isConnected()) {
    return;
  }

  NimBLEConnInfo info = peer.client->getConnInfo();
  NimBLEAddress idAddress = info.getIdAddress();
  NimBLEAddress otaAddress = info.getAddress();
  String name = "(unnamed)";

  if (peer.name.length() > 0 && peer.name != "(unnamed)") {
    name = peer.name;
  } else {
    int savedIndex = findSavedKeyboard(idAddress.toString().c_str());
    if (savedIndex < 0) {
      savedIndex = findSavedKeyboard(otaAddress.toString().c_str());
    }
    if (savedIndex >= 0) {
      name = savedKeyboards[savedIndex].name;
    }
  }

  Serial.print("Connection security: bonded=");
  Serial.print(info.isBonded() ? "yes" : "no");
  Serial.print(" encrypted=");
  Serial.print(info.isEncrypted() ? "yes" : "no");
  Serial.print(" authenticated=");
  Serial.print(info.isAuthenticated() ? "yes" : "no");
  Serial.print(" ota=");
  Serial.print(otaAddress.toString().c_str());
  Serial.print(" id=");
  Serial.println(idAddress.toString().c_str());

  saveKeyboardIdentity(idAddress.isNull() ? otaAddress : idAddress, name);
}

static bool savedDeviceMissing() {
  for (uint8_t i = 0; i < savedKeyboardCount; i++) {
    bool present = false;
    for (Peer &peer : peers) {
      if (!peer.inUse) {
        continue;
      }
      // Address first, then name: a registry entry holds the *identity* address,
      // which is not necessarily the address the device advertised under and
      // connected with. Without the name fallback a connected device could look
      // missing, and the bridge would scan for it forever.
      if (savedKeyboards[i].address.equalsIgnoreCase(peer.address.toString().c_str()) ||
          (savedKeyboards[i].name != "(unnamed)" &&
           savedKeyboards[i].name.equalsIgnoreCase(peer.name))) {
        present = true;
        break;
      }
    }
    if (!present) {
      return true;
    }
  }
  return false;
}

static void listSavedKeyboards() {
  Serial.print("SPIFFS keyboard registry: ");
  Serial.print(savedKeyboardCount);
  Serial.println(" saved");

  for (uint8_t i = 0; i < savedKeyboardCount; i++) {
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(savedKeyboards[i].address);
    Serial.print(" type=");
    Serial.print(savedKeyboards[i].addressType);
    Serial.print(" name=");
    Serial.println(savedKeyboards[i].name);
  }
}

static void disconnectAllPeers() {
  connectPending = false;
  for (Peer &peer : peers) {
    if (peer.client == nullptr) {
      continue;
    }
    if (peer.client->isConnected()) {
      peer.client->disconnect();   // onDisconnect releases the slot
    } else if (peer.inUse) {
      peer.client->cancelConnect();
      releasePeer(peer);
    }
  }
}

static void clearSavedKeyboards() {
  NimBLEScan *scan = NimBLEDevice::getScan();
  if (scan->isScanning()) {
    scan->stop();
  }

  disconnectAllPeers();

  bool bondsCleared = NimBLEDevice::deleteAllBonds();
  savedKeyboardCount = 0;
  if (spiffsReady && SPIFFS.exists(KEYBOARD_STORE_PATH)) {
    SPIFFS.remove(KEYBOARD_STORE_PATH);
  }

  pairingMode = true;
  shouldScan = true;
  nextScanAt = millis() + SCAN_RESTART_DELAY_MS;
  Serial.println(bondsCleared
    ? "Cleared SPIFFS keyboard registry and all NimBLE bonds; pairing mode enabled"
    : "SPIFFS registry cleared, but one or more NimBLE bonds could not be removed");
}

static void emitKey(uint8_t key, uint8_t modifiers) {
  const bool shift = shifted(modifiers);
  const bool ctrl = controlled(modifiers);

  if (key >= 0x04 && key <= 0x1D) {
    char c = char('a' + key - 0x04);
    if (ctrl && !commanded(modifiers)) {
      writeModifiedControl(uint8_t(c - 'a' + 1), modifiers);
    } else {
      // HID semantics: Shift and Caps Lock invert each other for letters.
      writeModifiedAscii((shift ^ capsLockOn) ? char(toupper(c)) : c, modifiers);
    }
    return;
  }

  if (key >= 0x1E && key <= 0x27) {
    static const char normal[] = "1234567890";
    static const char shiftedNums[] = "!@#$%^&*()";
    char c = shift ? shiftedNums[key - 0x1E] : normal[key - 0x1E];
    if (!writeCtrlPunctuation(c, modifiers)) {
      writeModifiedAscii(c, modifiers);
    }
    return;
  }

  switch (key) {
    case 0x28: writeModifiedAscii('\r', modifiers); break;       // Enter
    case 0x29: writeModifiedControl(0x1B, modifiers); break;     // Escape
    case 0x2A: writeModifiedControl(0x08, modifiers); break;     // Backspace
    case 0x2B: writeModifiedAscii('\t', modifiers); break;
    case 0x2C: {
      char c = ' ';
      if (!writeCtrlPunctuation(c, modifiers)) writeModifiedAscii(c, modifiers);
      break;
    }
    case 0x2D: {
      char c = shift ? '_' : '-';
      if (!writeCtrlPunctuation(c, modifiers)) writeModifiedAscii(c, modifiers);
      break;
    }
    case 0x2E: writeModifiedAscii(shift ? '+' : '=', modifiers); break;
    case 0x2F: {
      char c = shift ? '{' : '[';
      if (!writeCtrlPunctuation(c, modifiers)) writeModifiedAscii(c, modifiers);
      break;
    }
    case 0x30: {
      char c = shift ? '}' : ']';
      if (!writeCtrlPunctuation(c, modifiers)) writeModifiedAscii(c, modifiers);
      break;
    }
    case 0x31: {
      char c = shift ? '|' : '\\';
      if (!writeCtrlPunctuation(c, modifiers)) writeModifiedAscii(c, modifiers);
      break;
    }
    case 0x32: writeModifiedAscii(shift ? '~' : '#', modifiers); break;
    case 0x33: writeModifiedAscii(shift ? ':' : ';', modifiers); break;
    case 0x34: writeModifiedAscii(shift ? '"' : '\'', modifiers); break;
    case 0x35: writeModifiedAscii(shift ? '~' : '`', modifiers); break;
    case 0x36: writeModifiedAscii(shift ? '<' : ',', modifiers); break;
    case 0x37: writeModifiedAscii(shift ? '>' : '.', modifiers); break;
    case 0x38: {
      char c = shift ? '?' : '/';
      if (!writeCtrlPunctuation(c, modifiers)) writeModifiedAscii(c, modifiers);
      break;
    }
    case 0x39: setCapsLockState(!capsLockOn); break;              // Caps Lock
    case 0x3A: writeModifiedCsiFinal("\x1BOP", 'P', modifiers); break;     // F1
    case 0x3B: writeModifiedCsiFinal("\x1BOQ", 'Q', modifiers); break;     // F2
    case 0x3C: writeModifiedCsiFinal("\x1BOR", 'R', modifiers); break;     // F3
    case 0x3D: writeModifiedCsiFinal("\x1BOS", 'S', modifiers); break;     // F4
    case 0x3E: writeModifiedCsiTilde("\x1B[15~", 15, modifiers); break;    // F5
    case 0x3F: writeModifiedCsiTilde("\x1B[17~", 17, modifiers); break;    // F6
    case 0x40: writeModifiedCsiTilde("\x1B[18~", 18, modifiers); break;    // F7
    case 0x41: writeModifiedCsiTilde("\x1B[19~", 19, modifiers); break;    // F8
    case 0x42: writeModifiedCsiTilde("\x1B[20~", 20, modifiers); break;    // F9
    case 0x43: writeModifiedCsiTilde("\x1B[21~", 21, modifiers); break;    // F10
    case 0x44: writeModifiedCsiTilde("\x1B[23~", 23, modifiers); break;    // F11
    case 0x45: writeModifiedCsiTilde("\x1B[24~", 24, modifiers); break;    // F12
    case 0x4A: writeModifiedCsiFinal("\x1B[1~", 'H', modifiers); break;    // Home
    case 0x4B: writeModifiedCsiTilde("\x1B[5~", 5, modifiers); break;      // Page Up
    case 0x4C: writeModifiedCsiTilde("\x1B[3~", 3, modifiers); break;      // Delete
    case 0x4D: writeModifiedCsiTilde("\x1B[6~", 6, modifiers); break;      // Page Down
    case 0x4E: writeModifiedCsiFinal("\x1B[4~", 'F', modifiers); break;    // End
    case 0x4F: writeModifiedCsiFinal("\x1B[C", 'C', modifiers); break;     // Right
    case 0x50: writeModifiedCsiFinal("\x1B[D", 'D', modifiers); break;     // Left
    case 0x51: writeModifiedCsiFinal("\x1B[B", 'B', modifiers); break;     // Down
    case 0x52: writeModifiedCsiFinal("\x1B[A", 'A', modifiers); break;     // Up
    default: break;
  }
}

// Gamepad mode (toggled by the "GAME 0|1" command from DS). While on, the
// bridge stops sending ASCII/CSI keystrokes and instead sends button events the
// DS emulator (Gameboy.ino) can turn into a held-button bitmap:
//   0xF0 <bit>  DOWN,   0xF1 <bit>  UP,   0xF2  QUIT (Ctrl+T),  0xF3  MENU (Esc)
// where <bit> is a Game Boy pad bit (GB_PAD_*): Right 0x01, Left 0x02, Up 0x04,
// Down 0x08, A 0x10, B 0x20, Select 0x40, Start 0x80. This exists because the
// normal path (emitKey) is rising-edge only and drops key-up entirely, so a game
// could never tell when a button was released or still held.
//
// Quit used to be a bare Escape keypress -- a single unmapped key was enough to
// trip it, which is exactly what closed the ROM on any non-game key press.
// Ctrl+T needs both the Ctrl modifier bit and the T usage code at once, so no
// single stray keycode can fake it. Escape now opens the emulator's own settings
// menu instead, which is the safe thing for a key that easy to hit: opening a
// menu by accident costs one keypress to undo, closing the ROM cost the session.
// (gameMode itself is declared earlier, near the other mode flags, so
// baseStatusColor() can read it.)
static uint8_t gamepadPrev = 0;   // merged GB button bitmap last sent to the DS
static bool gamepadQuitPrev = false;
static bool gamepadMenuPrev = false;

// Maps a HID usage code to a GB pad bit (0 = not a game button). Arrow keys and
// WASD both drive the D-pad; N = A, M = B, Enter = Start, backslash = Select.
//
// N/M sit under the right hand with WASD under the left, so the whole pad is
// reachable without moving either. Select is backslash rather than the old
// Backspace/Tab: both of those are easy to hit by reflex, and on a Game Boy
// Select is usually a "did I mean that?" key. Keep these in sync with the
// control list `gb` prints (../DS/Gameboy.ino).
static uint8_t hidToGb(uint8_t key) {
  switch (key) {
    case 0x4F: case 0x07: return 0x01;  // Right / D
    case 0x50: case 0x04: return 0x02;  // Left  / A
    case 0x52: case 0x1A: return 0x04;  // Up    / W
    case 0x51: case 0x16: return 0x08;  // Down  / S
    case 0x11: return 0x10;             // A button: N
    case 0x10: return 0x20;             // B button: M
    case 0x31: return 0x40;             // Select: backslash
    case 0x28: return 0x80;             // Start: Enter
    default: return 0x00;
  }
}

// Sends the merged state of every connected device to the DS.
//
// Each peer keeps its own gbMask/gbQuit/gbMenu (its last decoded report) and this
// ORs them together, so a keyboard and a pad can both be held at once and
// neither clears the other's buttons: with a single shared bitmap, the pad's
// next idle report would release a key the keyboard is still holding down. The
// DS only ever sees one stream of DOWN/UP events and does not know or care how
// many devices produced it.
static void emitMergedGamepadState() {
  uint8_t mask = 0;
  bool quit = false;
  bool menu = false;
  for (Peer &peer : peers) {
    if (!peer.connected) {
      continue;
    }
    mask |= peer.gbMask;
    quit = quit || peer.gbQuit;
    menu = menu || peer.gbMenu;
  }
  if (usbKeyboardConnected) {
    mask |= usbKeyboardPeer.gbMask;
    quit = quit || usbKeyboardPeer.gbQuit;
    menu = menu || usbKeyboardPeer.gbMenu;
  }
  if (joystickPresent) {
    mask |= joystickPeer.gbMask;
  }
  mask |= buttonBarPeer.gbMask;

  const uint8_t down = mask & ~gamepadPrev;
  const uint8_t up = gamepadPrev & ~mask;
  for (uint8_t bit = 1; bit != 0; bit <<= 1) {
    if (down & bit) { linkWrite((uint8_t)0xF0); linkWrite(bit); }
  }
  for (uint8_t bit = 1; bit != 0; bit <<= 1) {
    if (up & bit) { linkWrite((uint8_t)0xF1); linkWrite(bit); }
  }
  gamepadPrev = mask;

  if (quit && !gamepadQuitPrev) linkWrite((uint8_t)0xF2);
  gamepadQuitPrev = quit;

  // Edge-only, like quit: the menu is a toggle on the DS side, so a held Escape
  // must not flip it open and shut every report.
  if (menu && !gamepadMenuPrev) linkWrite((uint8_t)0xF3);
  gamepadMenuPrev = menu;
}

// Keyboard flavour of the above. Handles multiple keys at once (6-key rollover),
// so real chords -- run + jump, diagonal + button -- come through.
static void handleGamepadReport(Peer &peer, const uint8_t *report) {
  const uint8_t modifiers = report[0];
  const uint8_t *keys = report + 2;
  uint8_t mask = 0;
  bool quit = false;
  bool menu = false;
  for (uint8_t i = 0; i < 6; i++) {
    uint8_t k = keys[i];
    if (k == 0) continue;
    if (k == 0x17 && controlled(modifiers)) quit = true;   // Ctrl+T
    if (k == 0x29) menu = true;                            // Escape
    mask |= hidToGb(k);
  }

  peer.gbMask = mask;
  peer.gbQuit = quit;
  peer.gbMenu = menu;
  emitMergedGamepadState();
}

// Clears one peer's contribution to the merged state. Called when it goes away,
// so the DS is not left holding a button for a device that has disconnected --
// its bitmap is only ever cleared by our 0xF1 events.
static void clearPeerGameState(Peer &peer) {
  peer.gbMask = 0;
  peer.gbQuit = false;
  peer.gbMenu = false;
}

// Flips gamepad mode and resets the edge state every input path keeps. Anything
// still held when the mode changes is released first, for the same reason.
static void applyGameMode(bool on, const char *reason) {
  const bool wasOn = gameMode;
  for (Peer &peer : peers) {
    clearPeerGameState(peer);
    memset(peer.lastKeys, 0, sizeof(peer.lastKeys));
    peer.padPrevKeyButtons = 0;
    peer.padRepeatBit = 0;
  }
  clearPeerGameState(usbKeyboardPeer);
  memset(usbKeyboardPeer.lastKeys, 0, sizeof(usbKeyboardPeer.lastKeys));
  usbKeyboardPeer.f12Prev = false;

  clearPeerGameState(joystickPeer);
  joystickPeer.padPrevKeyButtons = 0;
  joystickPeer.padRepeatBit = 0;

  clearPeerGameState(buttonBarPeer);
  buttonBarPeer.padPrevKeyButtons = 0;

  if (on != wasOn && wasOn) {
    emitMergedGamepadState();   // every mask is zero now: releases the lot
  }

  gameMode = on;
  gamepadPrev = 0;
  gamepadQuitPrev = false;
  gamepadMenuPrev = false;
  Serial.print("gameMode=");
  Serial.print(on ? "on (sending button events)" : "off (sending keystrokes)");
  Serial.print(", ");
  Serial.println(reason);
}

//   ---- Xbox (and other BLE HID) gamepad support ----------------------------
//
// Targets the Bluetooth-LE Xbox Wireless Controller (model 1914 / Series X|S and
// the Xbox One S-era pads that got the September 2021 BLE firmware). The old
// 2013 Xbox One pad has no Bluetooth at all and cannot work here.
//
// Input report ID 1 is 15-16 bytes, little-endian:
//   [0..1]  left stick X    uint16, 0x0000 left  .. 0x8000 centre .. 0xFFFF right
//   [2..3]  left stick Y    uint16, 0x0000 up    .. 0x8000 centre .. 0xFFFF down
//   [4..5]  right stick X   [6..7] right stick Y
//   [8..9]  left trigger    uint10 (0..1023), [10..11] right trigger
//   [12]    D-pad hat       0 = centred, 1 = N, 2 = NE, ... 8 = NW
//   [13]    A 0x01  B 0x02  X 0x08  Y 0x10  LB 0x40  RB 0x80
//   [14]    View 0x04  Menu 0x08  LS-click 0x20  RS-click 0x40
//   [15]    Share 0x01 (only on pads/firmware that have the Share button)
// The Guide (Xbox logo) button is its own one-byte report, ID 2, bit 0.
//
// Firmware revisions have been known to shuffle the byte-13/14 bit positions.
// Rather than guess in the dark, "DUMP 1" hex-logs every report to the console
// and Telnet so the real layout can be read off the actual controller; adjust
// the masks below if yours disagrees. Nothing else in the bridge depends on
// them being right.
static constexpr uint32_t XB_A     = 1UL << 0;
static constexpr uint32_t XB_B     = 1UL << 1;
static constexpr uint32_t XB_X     = 1UL << 2;
static constexpr uint32_t XB_Y     = 1UL << 3;
static constexpr uint32_t XB_LB    = 1UL << 4;
static constexpr uint32_t XB_RB    = 1UL << 5;
static constexpr uint32_t XB_VIEW  = 1UL << 6;    // the two-panes button, "Back"
static constexpr uint32_t XB_MENU  = 1UL << 7;    // the hamburger button, "Start"
static constexpr uint32_t XB_LS    = 1UL << 8;
static constexpr uint32_t XB_RS    = 1UL << 9;
static constexpr uint32_t XB_GUIDE = 1UL << 10;
static constexpr uint32_t XB_SHARE = 1UL << 11;
static constexpr uint32_t XB_LT    = 1UL << 12;
static constexpr uint32_t XB_RT    = 1UL << 13;
static constexpr uint32_t XB_UP    = 1UL << 14;   // hat and left stick, merged
static constexpr uint32_t XB_DOWN  = 1UL << 15;
static constexpr uint32_t XB_LEFT  = 1UL << 16;
static constexpr uint32_t XB_RIGHT = 1UL << 17;

static constexpr uint32_t XB_DIRECTIONS = XB_UP | XB_DOWN | XB_LEFT | XB_RIGHT;

// Stick thresholds, as a distance from centre out of 32768. Deliberately two
// numbers, not one: a stick resting near a single threshold jitters across it
// several times a second, and each crossing would be a keystroke or a
// down/up pair. A direction turns on at ON and only back off at the lower OFF.
static constexpr int32_t STICK_ON = 11000;
static constexpr int32_t STICK_OFF = 7000;
static constexpr uint16_t TRIGGER_ON = 512;
static constexpr uint16_t TRIGGER_OFF = 256;

// The live "what is held right now" bitmap and its edge/repeat bookkeeping are
// per-peer (Peer::padButtons and friends) so two pads cannot overwrite each
// other. They are written from the BLE notify callback and read from loop() by
// serviceGamepadRepeat; 32-bit aligned loads and stores are atomic on this core,
// so the worst a race can do is fire one auto-repeat an interval late.
static uint16_t padRead16(const uint8_t *data, size_t index) {
  return uint16_t(data[index]) | (uint16_t(data[index + 1]) << 8);
}

// Applies the on/off hysteresis described above to one axis half.
static bool padAxisHeld(int32_t magnitude, bool wasHeld) {
  return magnitude >= (wasHeld ? STICK_OFF : STICK_ON);
}

static uint32_t padHatDirections(uint8_t hat) {
  switch (hat) {
    case 1: return XB_UP;
    case 2: return XB_UP | XB_RIGHT;
    case 3: return XB_RIGHT;
    case 4: return XB_DOWN | XB_RIGHT;
    case 5: return XB_DOWN;
    case 6: return XB_DOWN | XB_LEFT;
    case 7: return XB_LEFT;
    case 8: return XB_UP | XB_LEFT;
    default: return 0;
  }
}

// Rebuilds this peer's padButtons from one report-ID-1 packet. Guide and Share
// live in other reports, so their bits are carried over untouched.
static void padParseMainReport(Peer &peer, const uint8_t *data, size_t length) {
  if (length < 14) {
    return;
  }

  const uint32_t previous = peer.padButtons;
  uint32_t next = previous & (XB_GUIDE | XB_SHARE);

  const int32_t lx = int32_t(padRead16(data, 0)) - 32768;
  const int32_t ly = int32_t(padRead16(data, 2)) - 32768;   // HID Y grows downward

  if (padAxisHeld(-lx, previous & XB_LEFT))  next |= XB_LEFT;
  if (padAxisHeld(lx,  previous & XB_RIGHT)) next |= XB_RIGHT;
  if (padAxisHeld(-ly, previous & XB_UP))    next |= XB_UP;
  if (padAxisHeld(ly,  previous & XB_DOWN))  next |= XB_DOWN;

  const uint16_t lt = padRead16(data, 8) & 0x03FF;
  const uint16_t rt = padRead16(data, 10) & 0x03FF;
  if (lt >= ((previous & XB_LT) ? TRIGGER_OFF : TRIGGER_ON)) next |= XB_LT;
  if (rt >= ((previous & XB_RT) ? TRIGGER_OFF : TRIGGER_ON)) next |= XB_RT;

  next |= padHatDirections(data[12] & 0x0F);

  const uint8_t face = data[13];
  if (face & 0x01) next |= XB_A;
  if (face & 0x02) next |= XB_B;
  if (face & 0x08) next |= XB_X;
  if (face & 0x10) next |= XB_Y;
  if (face & 0x40) next |= XB_LB;
  if (face & 0x80) next |= XB_RB;

  // Byte 14 is absent on a 14-byte report (some firmware trims it); the pad then
  // simply has no View/Menu/stick-click bits this packet rather than us reading
  // one byte off the end of the notification buffer.
  if (length >= 15) {
    const uint8_t aux = data[14];
    if (aux & 0x04) next |= XB_VIEW;
    if (aux & 0x08) next |= XB_MENU;
    if (aux & 0x20) next |= XB_LS;
    if (aux & 0x40) next |= XB_RS;
  }

  if (length >= 16 && (data[15] & 0x01)) next |= XB_SHARE;

  peer.padButtons = next;
}

// Game mode: the pad's own layout folded onto the Game Boy's eight bits.
// D-pad and left stick both drive the pad; A/B are the Xbox buttons of the same
// name; Menu is Start and View is Select, matching where those sit on a Game Boy.
// X, Y, the sticks' clicks and the triggers are deliberately unmapped -- a Game
// Boy has nothing for them to do, and leaving them silent means a stray thumb
// cannot press anything.
static uint8_t padToGameBoy(uint32_t buttons) {
  uint8_t mask = 0;
  if (buttons & XB_RIGHT) mask |= 0x01;
  if (buttons & XB_LEFT)  mask |= 0x02;
  if (buttons & XB_UP)    mask |= 0x04;
  if (buttons & XB_DOWN)  mask |= 0x08;
  if (buttons & XB_A)     mask |= 0x10;
  if (buttons & XB_B)     mask |= 0x20;
  if (buttons & XB_VIEW)  mask |= 0x40;   // Select
  if (buttons & XB_MENU)  mask |= 0x80;   // Start
  return mask;
}

// Keystroke mode: the pad drives the DS shell/menus using keys the rest of the
// firmware already understands, so nothing downstream needs to know a controller
// exists. Directions auto-repeat when held (see serviceGamepadRepeat) because the
// keystroke path is edge-only -- without it, scrolling a file list would mean
// tapping the stick once per row. (PadKeyBinding is defined at the top of the
// file; see the note there.)
static const PadKeyBinding padKeyBindings[] = {
  {XB_UP,    "\x1B[A",  true},
  {XB_DOWN,  "\x1B[B",  true},
  {XB_RIGHT, "\x1B[C",  true},
  {XB_LEFT,  "\x1B[D",  true},
  {XB_A,     "\r",      false},   // Enter
  {XB_B,     "\x1B",    false},   // Escape
  {XB_X,     "\x08",    false},   // Backspace
  {XB_Y,     "\t",      false},   // Tab
  {XB_MENU,  "\r",      false},
  {XB_VIEW,  "\x1B",    false},
  {XB_LB,    "\x1B[5~", true},    // Page Up
  {XB_RB,    "\x1B[6~", true},    // Page Down
  {XB_RT,    " ",       false},
};

static const PadKeyBinding *padBindingFor(uint32_t bit) {
  for (const PadKeyBinding &binding : padKeyBindings) {
    if (binding.bit == bit) {
      return &binding;
    }
  }
  return nullptr;
}

static void padEmitKeystrokes(Peer &peer, uint32_t buttons) {
  const uint32_t pressed = buttons & ~peer.padPrevKeyButtons;
  const uint32_t released = peer.padPrevKeyButtons & ~buttons;
  peer.padPrevKeyButtons = buttons;

  for (const PadKeyBinding &binding : padKeyBindings) {
    if (pressed & binding.bit) {
      linkWrite(binding.sequence);
      if (binding.repeats) {
        // Newest direction wins, so rolling the stick from up to left starts
        // repeating left instead of arguing with the key that is no longer held.
        peer.padRepeatBit = binding.bit;
        peer.padRepeatAt = millis() + PAD_REPEAT_DELAY_MS;
      }
    }
  }

  if (peer.padRepeatBit != 0 && (released & peer.padRepeatBit)) {
    peer.padRepeatBit = 0;
  }
}

// One input's share of the keystroke auto-repeat. Split out from the loop below
// because the analog joystick needs the same treatment but is not in peers[].
static void servicePeerRepeat(Peer &peer) {
  if (peer.padRepeatBit == 0) {
    return;
  }

  if ((peer.padButtons & peer.padRepeatBit) == 0) {   // released without us noticing
    peer.padRepeatBit = 0;
    return;
  }

  if (int32_t(millis() - peer.padRepeatAt) < 0) {
    return;
  }

  const PadKeyBinding *binding = padBindingFor(peer.padRepeatBit);
  if (binding == nullptr) {
    peer.padRepeatBit = 0;
    return;
  }

  linkWrite(binding->sequence);
  peer.padRepeatAt = millis() + PAD_REPEAT_INTERVAL_MS;
}

static void serviceGamepadRepeat() {
  if (gameMode) {
    return;
  }

  for (Peer &peer : peers) {
    if (!peer.connected) {
      continue;
    }
    servicePeerRepeat(peer);
  }
}

// Runs after every report, in whichever mode is active.
//
// Guide toggles game mode, the same escape hatch F12 is on the keyboard, so the
// pad can switch itself between driving the shell and driving a ROM without the
// DS link being involved at all.
//
// In game mode LB opens the emulator menu and LB+RB together quit. Neither
// bumper maps to a Game Boy button, so the chord cannot collide with gameplay,
// and needing two shoulders at once gives quitting the same "no single stray
// press can do this" property Ctrl+T has on the keyboard. Pressing LB before RB
// does open the menu on the way to quitting; the emulator is closing anyway.
//
// Outside game mode the button bar is the one input that does not send keystrokes:
// its three switches exist to drive playback and launch apps, which is a job no
// key sequence expresses, so they get their own private bytes (LINK_PAD_*). A
// paired controller keeps sending Enter/Escape from its A/B, because a controller
// is a general navigation device and losing those would cost more than it gains.
static void padApplyState(Peer &peer) {
  const bool guide = (peer.padButtons & XB_GUIDE) != 0;
  if (guide && !peer.guidePrev) {
    peer.guidePrev = guide;   // applyGameMode resets the other per-peer state
    applyGameMode(!gameMode, "Guide button toggle");
    return;
  }
  peer.guidePrev = guide;

  if (gameMode) {
    const bool bumperL = (peer.padButtons & XB_LB) != 0;
    const bool bumperR = (peer.padButtons & XB_RB) != 0;
    peer.gbMask = padToGameBoy(peer.padButtons);
    peer.gbQuit = bumperL && bumperR;
    peer.gbMenu = bumperL && !bumperR;
    emitMergedGamepadState();
    return;
  }

  if (&peer == &buttonBarPeer) {
    buttonBarEmitLinkBytes(peer, peer.padButtons);
    return;
  }

  padEmitKeystrokes(peer, peer.padButtons);
}

//   ---- Analog thumb joystick ----------------------------------------------
//
// A KY-023-style module: two potentiometers wired as dividers across 3V3 and a
// momentary switch to ground. Its axes are decoded into the same XB_UP/DOWN/
// LEFT/RIGHT bits an Xbox stick produces and handed to padApplyState(), so it
// sends exactly what the arrow keys send -- CSI arrow sequences in keystroke
// mode, Game Boy D-pad events in game mode -- with the same hysteresis and
// auto-repeat. The stick's push-button rides along as XB_VIEW, the pad's Select:
// the Game Boy's Select button in game mode, Escape in keystroke mode, which is
// where the pad's View button already goes.
#if SLAVE_JOYSTICK_ENABLED

static constexpr uint32_t JOYSTICK_SAMPLE_INTERVAL_MS = 10;
static constexpr uint32_t JOYSTICK_BUTTON_DEBOUNCE_MS = 25;
static constexpr uint8_t JOYSTICK_CALIBRATION_SAMPLES = 32;
// ADC counts (12-bit) scaled onto the +/-32768 range padAxisHeld() thresholds
// against, so both sticks share one set of on/off thresholds. Half of the ADC's
// span is 2048 counts, and 2048 * 16 = 32768.
static constexpr int32_t JOYSTICK_SCALE = 16;
// A wired-up module rests near mid-scale and is quiet. An unconnected ADC pin
// floats: it reads at one of the rails, or wanders. Either would look like a
// direction being held forever, so an implausible rest position means "no
// joystick" rather than a stuck arrow key. "JOY 1" overrides the verdict.
static constexpr uint16_t JOYSTICK_CENTER_MIN = 1000;
static constexpr uint16_t JOYSTICK_CENTER_MAX = 3100;
static constexpr uint16_t JOYSTICK_MAX_REST_JITTER = 220;

static uint16_t joystickCenterX = 2048;
static uint16_t joystickCenterY = 2048;
static uint16_t joystickRawX = 0;
static uint16_t joystickRawY = 0;
static uint16_t joystickRestJitter = 0;
static bool joystickForced = false;   // "JOY 1": trust the wiring over the probe
static uint32_t nextJoystickSampleAt = 0;
static bool joystickButtonRawPressed = false;
static bool joystickButtonPressed = false;
static uint32_t joystickButtonChangedAt = 0;

// Average of a short burst, with the spread reported alongside: the mean is the
// centre to measure deflection from, the spread is how we tell a resting stick
// from a floating pin.
static uint16_t joystickSampleAxis(int pin, uint16_t &spread) {
  uint32_t total = 0;
  uint16_t lowest = 4095;
  uint16_t highest = 0;
  for (uint8_t i = 0; i < JOYSTICK_CALIBRATION_SAMPLES; i++) {
    const uint16_t sample = analogRead(pin);
    total += sample;
    if (sample < lowest) lowest = sample;
    if (sample > highest) highest = sample;
    delay(1);
  }
  spread = highest - lowest;
  return uint16_t(total / JOYSTICK_CALIBRATION_SAMPLES);
}

// Re-reads the resting position. Nothing may be touching the stick while this
// runs -- wherever it is held becomes the new centre.
static bool joystickCalibrate() {
  uint16_t spreadX = 0;
  uint16_t spreadY = 0;
  joystickCenterX = joystickSampleAxis(SLAVE_JOYSTICK_X_PIN, spreadX);
  joystickCenterY = joystickSampleAxis(SLAVE_JOYSTICK_Y_PIN, spreadY);
  joystickRawX = joystickCenterX;
  joystickRawY = joystickCenterY;
  joystickRestJitter = spreadX > spreadY ? spreadX : spreadY;

  const bool plausible =
    joystickCenterX >= JOYSTICK_CENTER_MIN && joystickCenterX <= JOYSTICK_CENTER_MAX &&
    joystickCenterY >= JOYSTICK_CENTER_MIN && joystickCenterY <= JOYSTICK_CENTER_MAX &&
    joystickRestJitter <= JOYSTICK_MAX_REST_JITTER;
  return plausible || joystickForced;
}

// Drops whatever the joystick was holding. Used when it is switched off, so the
// DS is not left with a button down that nothing will ever release.
static void joystickReleaseAll() {
  joystickPeer.padButtons = 0;
  joystickPeer.padRepeatBit = 0;
  if (gameMode) {
    joystickPeer.gbMask = 0;
    emitMergedGamepadState();
  } else {
    joystickPeer.padPrevKeyButtons = 0;
  }
}

static void joystickBegin() {
  analogReadResolution(12);
  analogSetPinAttenuation(SLAVE_JOYSTICK_X_PIN, ADC_11db);   // full 0-3V3 swing
  analogSetPinAttenuation(SLAVE_JOYSTICK_Y_PIN, ADC_11db);
  pinMode(SLAVE_JOYSTICK_BUTTON_PIN, INPUT_PULLUP);
  joystickButtonRawPressed = digitalRead(SLAVE_JOYSTICK_BUTTON_PIN) == LOW;
  joystickButtonPressed = joystickButtonRawPressed;
  joystickButtonChangedAt = millis();

  joystickPresent = joystickCalibrate();
  Serial.printf("Joystick X=%d Y=%d SW=%d: %s (centre %u/%u jitter %u)\r\n",
                SLAVE_JOYSTICK_X_PIN, SLAVE_JOYSTICK_Y_PIN, SLAVE_JOYSTICK_BUTTON_PIN,
                joystickPresent ? "detected" : "not detected (pins float; use JOY 1 to force)",
                joystickCenterX, joystickCenterY, joystickRestJitter);
}

static uint32_t joystickReadDirections() {
  joystickRawX = analogRead(SLAVE_JOYSTICK_X_PIN);
  joystickRawY = analogRead(SLAVE_JOYSTICK_Y_PIN);

  int32_t x = (int32_t(joystickRawX) - int32_t(joystickCenterX)) * JOYSTICK_SCALE;
  int32_t y = (int32_t(joystickRawY) - int32_t(joystickCenterY)) * JOYSTICK_SCALE;
  if (SLAVE_JOYSTICK_INVERT_X) x = -x;
  if (SLAVE_JOYSTICK_INVERT_Y) y = -y;

  // Rising X is right and rising Y is up, before the invert flags above. Which
  // way a given module is soldered and which way round it is glued into the case
  // both change that, so the flags live in BoardVariant.h next to the encoder's.
  const uint32_t previous = joystickPeer.padButtons;
  uint32_t next = 0;
  if (padAxisHeld(-x, previous & XB_LEFT))  next |= XB_LEFT;
  if (padAxisHeld(x,  previous & XB_RIGHT)) next |= XB_RIGHT;
  if (padAxisHeld(y,  previous & XB_UP))    next |= XB_UP;
  if (padAxisHeld(-y, previous & XB_DOWN))  next |= XB_DOWN;
  return next;
}

// Debounced, and reported as a level rather than an edge: padApplyState wants
// the whole "what is held now" bitmap, and game mode needs the release too.
static bool joystickButtonHeld() {
  const bool rawPressed = digitalRead(SLAVE_JOYSTICK_BUTTON_PIN) == LOW;
  const uint32_t now = millis();
  if (rawPressed != joystickButtonRawPressed) {
    joystickButtonRawPressed = rawPressed;
    joystickButtonChangedAt = now;
  }
  if (rawPressed != joystickButtonPressed &&
      uint32_t(now - joystickButtonChangedAt) >= JOYSTICK_BUTTON_DEBOUNCE_MS) {
    joystickButtonPressed = rawPressed;
  }
  return joystickButtonPressed;
}

static void serviceJoystick() {
  if (!joystickPresent) {
    return;
  }

  // The ADC is polled on a fixed tick rather than every loop pass: at 10 ms a
  // held direction still repeats on time, and the reads stay out of the way of
  // the BLE work loop() is really here to do.
  const uint32_t now = millis();
  if (int32_t(now - nextJoystickSampleAt) >= 0) {
    nextJoystickSampleAt = now + JOYSTICK_SAMPLE_INTERVAL_MS;

    uint32_t buttons = joystickReadDirections();
    if (joystickButtonHeld()) {
      buttons |= XB_VIEW;
    }

    if (buttons != joystickPeer.padButtons) {
      joystickPeer.padButtons = buttons;
      padApplyState(joystickPeer);
    }
  }

  if (!gameMode) {
    servicePeerRepeat(joystickPeer);
  }
}

static const char *joystickStateName() {
  return joystickPresent ? (joystickForced ? "forced" : "ready") : "absent";
}

// "JOY", "JOY CAL", "JOY 0|1" -- report, recentre, or overrule the probe.
static void handleJoystickCommand(const String &args) {
  if (args == "0") {
    joystickForced = false;
    if (joystickPresent) {
      joystickPresent = false;
      joystickReleaseAll();
    }
    Serial.println("Joystick disabled");
    return;
  }

  if (args == "1" || args == "CAL" || args == "CALIBRATE" || args.length() == 0) {
    if (args == "1") {
      joystickForced = true;
    }
    if (args.length() > 0) {
      joystickPresent = joystickCalibrate();
    }
  } else {
    Serial.println("Usage: JOY [CAL|0|1]");
    return;
  }

  Serial.printf("Joystick %s: x=%u y=%u centre=%u/%u jitter=%u button=%s\r\n",
                joystickStateName(), joystickRawX, joystickRawY,
                joystickCenterX, joystickCenterY, joystickRestJitter,
                joystickButtonPressed ? "down" : "up");
}

#else
static void joystickBegin() {}
static void serviceJoystick() {}
static const char *joystickStateName() {
  return "disabled";
}
static void handleJoystickCommand(const String &args) {
  (void)args;
  Serial.println("Joystick support is compiled out (SLAVE_JOYSTICK_ENABLED 0)");
}
#endif

//   ---- Three-button bar ----------------------------------------------------
//
// Three momentary switches on a common rail -- Start, B, A from left to right --
// the Game Boy buttons the joystick does not already cover. Like the joystick they
// are decoded into pad bits and pushed through padApplyState(), so in game mode
// they are the real Start/B/A. Outside game mode they part company with every other
// input: instead of keystrokes they send one private LINK_PAD_* byte per press, and
// DOLL-OS turns that into transport or a launcher depending on what is running.
//
// The common rail is 3V3 and each switch closes onto its GPIO, so a pressed
// button reads high and the pins idle low on the internal pulldowns. Wired the
// other way -- common to GND, switches pulling the pin down -- set
// SLAVE_BUTTON_BAR_ACTIVE_LOW to 1 and the pins idle high on pullups instead.
#if SLAVE_BUTTON_BAR_ENABLED

static constexpr uint32_t BUTTON_BAR_DEBOUNCE_MS = 25;

struct ButtonBarKey {
  uint8_t pin;
  uint32_t bit;
  const char *label;
};

//listed left to right, which is also the order "BTN" prints them in
static const ButtonBarKey buttonBarKeys[] = {
  {SLAVE_BUTTON_C_PIN, XB_MENU, "START"},
  {SLAVE_BUTTON_B_PIN, XB_B,    "B"},
  {SLAVE_BUTTON_A_PIN, XB_A,    "A"},
};
static constexpr uint8_t BUTTON_BAR_COUNT =
  sizeof(buttonBarKeys) / sizeof(buttonBarKeys[0]);

static bool buttonBarRawPressed[BUTTON_BAR_COUNT];
static bool buttonBarPressed[BUTTON_BAR_COUNT];
static uint32_t buttonBarChangedAt[BUTTON_BAR_COUNT];

// Keystroke mode: one private byte per press edge (see LINK_PAD_* above), which is
// the whole reason the bar does not go through padEmitKeystrokes(). Nothing is sent
// on release and nothing auto-repeats -- every action on the DS side is a one-shot,
// so a held button is the same event as a tapped one. XB_VIEW is listed for the
// build that adds a fourth switch as Select; the bar as wired never sets that bit
// (the joystick's click carries Select, and keeps sending Escape in the shell).
struct ButtonBarLinkByte {
  uint32_t bit;
  uint8_t byte;
};

static const ButtonBarLinkByte buttonBarLinkBytes[] = {
  {XB_MENU, LINK_PAD_START},
  {XB_VIEW, LINK_PAD_SELECT},
  {XB_B,    LINK_PAD_B},
  {XB_A,    LINK_PAD_A},
};

static void buttonBarEmitLinkBytes(Peer &peer, uint32_t buttons) {
  const uint32_t pressed = buttons & ~peer.padPrevKeyButtons;
  peer.padPrevKeyButtons = buttons;

  for (const ButtonBarLinkByte &entry : buttonBarLinkBytes) {
    if (pressed & entry.bit) {
      linkWrite(entry.byte);
    }
  }
}

static bool buttonBarRead(uint8_t index) {
  const bool high = digitalRead(buttonBarKeys[index].pin) == HIGH;
  return SLAVE_BUTTON_BAR_ACTIVE_LOW ? !high : high;
}

static void buttonBarBegin() {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < BUTTON_BAR_COUNT; i++) {
    pinMode(buttonBarKeys[i].pin,
            SLAVE_BUTTON_BAR_ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);
    buttonBarRawPressed[i] = buttonBarRead(i);
    buttonBarPressed[i] = buttonBarRawPressed[i];
    buttonBarChangedAt[i] = now;
  }
  Serial.printf("Button bar enabled: START=%d B=%d A=%d active-%s\r\n",
                SLAVE_BUTTON_C_PIN, SLAVE_BUTTON_B_PIN, SLAVE_BUTTON_A_PIN,
                SLAVE_BUTTON_BAR_ACTIVE_LOW ? "low" : "high");
}

static void serviceButtonBar() {
  const uint32_t now = millis();
  uint32_t buttons = 0;

  for (uint8_t i = 0; i < BUTTON_BAR_COUNT; i++) {
    const bool rawPressed = buttonBarRead(i);
    if (rawPressed != buttonBarRawPressed[i]) {
      buttonBarRawPressed[i] = rawPressed;
      buttonBarChangedAt[i] = now;
    }
    if (rawPressed != buttonBarPressed[i] &&
        uint32_t(now - buttonBarChangedAt[i]) >= BUTTON_BAR_DEBOUNCE_MS) {
      buttonBarPressed[i] = rawPressed;
    }
    if (buttonBarPressed[i]) {
      buttons |= buttonBarKeys[i].bit;
    }
  }

  // None of A/B/Start auto-repeats, so unlike the joystick there is no repeat to
  // service here -- an edge either way is the whole of the state change.
  if (buttons != buttonBarPeer.padButtons) {
    buttonBarPeer.padButtons = buttons;
    padApplyState(buttonBarPeer);
  }
}

// "BTN" -- raw pin levels and the decoded press state, for checking which way
// round the bar is wired without having to guess from behaviour.
static void handleButtonBarCommand() {
  Serial.print("Button bar active-");
  Serial.print(SLAVE_BUTTON_BAR_ACTIVE_LOW ? "low:" : "high:");
  for (uint8_t i = 0; i < BUTTON_BAR_COUNT; i++) {
    Serial.printf(" %s(GPIO%u)=%s/%s", buttonBarKeys[i].label,
                  unsigned(buttonBarKeys[i].pin),
                  digitalRead(buttonBarKeys[i].pin) == HIGH ? "high" : "low",
                  buttonBarPressed[i] ? "down" : "up");
  }
  Serial.println();
}

#else
static void buttonBarBegin() {}
static void serviceButtonBar() {}
static void buttonBarEmitLinkBytes(Peer &peer, uint32_t buttons) {
  (void)peer;
  (void)buttons;   // padApplyState still names it; buttonBarPeer just never gets state
}
static void handleButtonBarCommand() {
  Serial.println("Button bar support is compiled out (SLAVE_BUTTON_BAR_ENABLED 0)");
}
#endif

static void handleGamepadInputReport(Peer &peer, uint8_t reportId, const uint8_t *data, size_t length) {
  if (reportId == 2) {
    // Guide button, reported on its own so it can be grabbed by a host without
    // disturbing the main report.
    if (length >= 1) {
      peer.padButtons = (data[0] & 0x01) ? (peer.padButtons | XB_GUIDE)
                                         : (peer.padButtons & ~XB_GUIDE);
    }
  } else if (reportId <= 1 && length >= 14) {
    // reportId 0 covers pads whose Report Reference descriptor we could not read;
    // the length check keeps a short consumer/battery report out of the parser.
    padParseMainReport(peer, data, length);
  } else {
    return;   // some other report (battery, consumer keys) -- nothing to do
  }

  padApplyState(peer);
}

static void dumpInputReport(const Peer &peer, uint8_t reportId,
                            const uint8_t *data, size_t length);

static void handleKeyboardReport(Peer &peer, const uint8_t *data, size_t length) {
  if (length < 8) {
    return;
  }

  const uint8_t *report = data;
  if (length >= 9 && data[1] != 0x00 && data[2] == 0x00) {
    report = data + 1;
  }

  // F12 flips gameMode directly from the keyboard, independent of the "GAME 0|1"
  // command that normally arrives over DS's bit-banged SlaveLink -- an escape
  // hatch for when that link is unreliable/unavailable, so gamepad mode can
  // still be toggled by hand. Checked on every report regardless of the mode
  // we're currently in, using the same report layout (report+2, 6 key slots)
  // both branches below already read.
  {
    bool f12Down = false;
    const uint8_t *reportKeys = report + 2;
    for (uint8_t i = 0; i < 6; i++) {
      if (reportKeys[i] == 0x45) {   // F12 usage code
        f12Down = true;
        break;
      }
    }
    if (f12Down && !peer.f12Prev) {
      peer.f12Prev = f12Down;   // applyGameMode resets the other per-peer state
      applyGameMode(!gameMode, "F12 toggle");
      return;
    }
    peer.f12Prev = f12Down;
  }

  if (gameMode) {
    handleGamepadReport(peer, report);
    return;
  }

  const uint8_t modifiers = report[0];
  const uint8_t *keys = report + 2;

  for (uint8_t i = 0; i < 6; i++) {
    uint8_t key = keys[i];
    if (key != 0 && !keyWasDown(peer, key)) {
      emitKey(key, modifiers);
    }
  }

  peer.lastModifiers = modifiers;
  rememberKeys(peer, keys);
}

//   ---- USB boot-keyboard host ---------------------------------------------
//
// This is deliberately a small class driver rather than a second key mapper.
// USB boot keyboards and BLE boot keyboards use the same eight-byte report, so
// once the USB interface is claimed every report goes through
// handleKeyboardReport() above. That keeps terminal modifiers, F12 game mode,
// lock state, and the Game Boy mapping identical on both transports.
#if USB_KEYBOARD_HOST_ENABLED

enum UsbKeyboardControlStage : uint8_t {
  USB_KB_CONTROL_IDLE = 0,
  USB_KB_CONTROL_SET_PROTOCOL,
  USB_KB_CONTROL_SET_IDLE,
  USB_KB_CONTROL_SET_LED,
};

struct UsbKeyboardHostState {
  usb_host_client_handle_t client;
  usb_device_handle_t device;
  usb_transfer_t *inputTransfer;
  usb_transfer_t *controlTransfer;
  uint8_t interfaceNumber;
  uint8_t alternateSetting;
  uint8_t inputEndpoint;
  uint16_t inputMps;
  volatile bool interfaceClaimed;
  volatile bool inputInFlight;
  volatile bool controlInFlight;
  volatile bool disconnectRequested;
  volatile UsbKeyboardControlStage controlStage;
};

static UsbKeyboardHostState usbKeyboardHost = {};
static volatile bool usbHostLibraryReady = false;
static volatile bool usbKeyboardLedPending = false;
static volatile uint8_t usbKeyboardPendingLedMask = 0;
static portMUX_TYPE usbKeyboardLedMux = portMUX_INITIALIZER_UNLOCKED;

static void usbKeyboardTryCleanup();
static bool usbKeyboardSubmitInput();

static void usbKeyboardResetPeerState() {
  clearPeerGameState(usbKeyboardPeer);
  memset(usbKeyboardPeer.lastKeys, 0, sizeof(usbKeyboardPeer.lastKeys));
  usbKeyboardPeer.lastModifiers = 0;
  usbKeyboardPeer.f12Prev = false;
}

static void usbKeyboardMarkDisconnected() {
  const bool wasConnected = usbKeyboardConnected;
  usbKeyboardConnected = false;
  usbKeyboardResetPeerState();
  if (wasConnected && gameMode) {
    emitMergedGamepadState();
  }
}

static bool usbKeyboardSubmitControl(UsbKeyboardControlStage stage, uint8_t ledValue = 0) {
  UsbKeyboardHostState &host = usbKeyboardHost;
  if (host.device == nullptr || host.controlTransfer == nullptr ||
      host.controlInFlight || host.disconnectRequested) {
    return false;
  }

  usb_setup_packet_t *setup = (usb_setup_packet_t *)host.controlTransfer->data_buffer;
  setup->bmRequestType = USB_BM_REQUEST_TYPE_DIR_OUT |
                         USB_BM_REQUEST_TYPE_TYPE_CLASS |
                         USB_BM_REQUEST_TYPE_RECIP_INTERFACE;
  setup->wIndex = host.interfaceNumber;
  setup->wLength = 0;

  if (stage == USB_KB_CONTROL_SET_PROTOCOL) {
    setup->bRequest = 0x0B;   // HID SET_PROTOCOL
    setup->wValue = 0;        // boot protocol
  } else if (stage == USB_KB_CONTROL_SET_IDLE) {
    setup->bRequest = 0x0A;   // HID SET_IDLE
    setup->wValue = 0;
  } else if (stage == USB_KB_CONTROL_SET_LED) {
    setup->bRequest = 0x09;   // HID SET_REPORT
    setup->wValue = 0x0200;   // output report, ID 0
    setup->wLength = 1;
    host.controlTransfer->data_buffer[USB_SETUP_PACKET_SIZE] = ledValue & 0x1F;
  } else {
    return false;
  }

  host.controlTransfer->device_handle = host.device;
  host.controlTransfer->bEndpointAddress = 0;
  host.controlTransfer->num_bytes = USB_SETUP_PACKET_SIZE + setup->wLength;
  host.controlStage = stage;
  host.controlInFlight = true;
  esp_err_t error = usb_host_transfer_submit_control(host.client, host.controlTransfer);
  if (error != ESP_OK) {
    host.controlInFlight = false;
    host.controlStage = USB_KB_CONTROL_IDLE;
    Serial.printf("USB keyboard control submit failed: %s\r\n", esp_err_to_name(error));
    return false;
  }
  return true;
}

static void usbKeyboardControlComplete(usb_transfer_t *transfer) {
  UsbKeyboardHostState &host = usbKeyboardHost;
  const UsbKeyboardControlStage completedStage = host.controlStage;
  host.controlInFlight = false;
  host.controlStage = USB_KB_CONTROL_IDLE;

  if (host.disconnectRequested || transfer->status == USB_TRANSFER_STATUS_NO_DEVICE) {
    return;
  }

  if (transfer->status != USB_TRANSFER_STATUS_COMPLETED) {
    // Some otherwise valid keyboards stall SET_IDLE or SET_PROTOCOL. Boot
    // reports are still worth trying, so configuration continues.
    Serial.printf("USB keyboard control request status=%d; continuing\r\n",
                  int(transfer->status));
  }

  if (completedStage == USB_KB_CONTROL_SET_PROTOCOL) {
    if (!usbKeyboardSubmitControl(USB_KB_CONTROL_SET_IDLE)) {
      usbKeyboardSubmitInput();
    }
  } else if (completedStage == USB_KB_CONTROL_SET_IDLE) {
    usbKeyboardSubmitInput();
  }
}

static void usbKeyboardInputComplete(usb_transfer_t *transfer) {
  UsbKeyboardHostState &host = usbKeyboardHost;
  host.inputInFlight = false;

  if (host.disconnectRequested || transfer->status == USB_TRANSFER_STATUS_NO_DEVICE) {
    return;
  }

  if (transfer->status == USB_TRANSFER_STATUS_COMPLETED) {
    if (transfer->actual_num_bytes >= 8) {
      if (reportDump) {
        dumpInputReport(usbKeyboardPeer, 0, transfer->data_buffer,
                        transfer->actual_num_bytes);
      }
      handleKeyboardReport(usbKeyboardPeer, transfer->data_buffer,
                           transfer->actual_num_bytes);
    }
  } else {
    Serial.printf("USB keyboard input status=%d; retrying\r\n", int(transfer->status));
    usb_host_endpoint_clear(host.device, host.inputEndpoint);
  }

  usbKeyboardSubmitInput();
}

static bool usbKeyboardSubmitInput() {
  UsbKeyboardHostState &host = usbKeyboardHost;
  if (host.device == nullptr || host.inputTransfer == nullptr ||
      host.inputInFlight || host.disconnectRequested) {
    return false;
  }

  host.inputTransfer->device_handle = host.device;
  host.inputTransfer->bEndpointAddress = host.inputEndpoint;
  host.inputTransfer->num_bytes = host.inputMps;
  host.inputInFlight = true;
  esp_err_t error = usb_host_transfer_submit(host.inputTransfer);
  if (error != ESP_OK) {
    host.inputInFlight = false;
    Serial.printf("USB keyboard input submit failed: %s\r\n", esp_err_to_name(error));
    return false;
  }

  if (!usbKeyboardConnected) {
    usbKeyboardResetPeerState();
    usbKeyboardPeer.name = "USB keyboard";
    usbKeyboardPeer.kind = PEER_KEYBOARD;
    usbKeyboardConnected = true;
    Serial.println("USB keyboard ready (boot protocol)");
    usbKeyboardQueueLedMask(ledMask);
  }
  return true;
}

static bool usbKeyboardFindInterface(const usb_config_desc_t *config,
                                     uint8_t &interfaceNumber,
                                     uint8_t &alternateSetting,
                                     uint8_t &inputEndpoint,
                                     uint16_t &inputMps) {
  const uint8_t *bytes = config->val;
  const size_t totalLength = config->wTotalLength;
  bool bootKeyboardInterface = false;

  for (size_t offset = 0; offset + USB_STANDARD_DESC_SIZE <= totalLength;) {
    const usb_standard_desc_t *descriptor =
      (const usb_standard_desc_t *)(bytes + offset);
    if (descriptor->bLength < USB_STANDARD_DESC_SIZE ||
        offset + descriptor->bLength > totalLength) {
      break;
    }

    if (descriptor->bDescriptorType == USB_B_DESCRIPTOR_TYPE_INTERFACE &&
        descriptor->bLength >= USB_INTF_DESC_SIZE) {
      const usb_intf_desc_t *interfaceDescriptor =
        (const usb_intf_desc_t *)descriptor;
      bootKeyboardInterface =
        interfaceDescriptor->bInterfaceClass == 0x03 &&
        interfaceDescriptor->bInterfaceSubClass == 0x01 &&
        interfaceDescriptor->bInterfaceProtocol == 0x01;
      if (bootKeyboardInterface) {
        interfaceNumber = interfaceDescriptor->bInterfaceNumber;
        alternateSetting = interfaceDescriptor->bAlternateSetting;
      }
    } else if (bootKeyboardInterface &&
               descriptor->bDescriptorType == USB_B_DESCRIPTOR_TYPE_ENDPOINT &&
               descriptor->bLength >= USB_EP_DESC_SIZE) {
      const usb_ep_desc_t *endpointDescriptor = (const usb_ep_desc_t *)descriptor;
      const bool input = (endpointDescriptor->bEndpointAddress &
                          USB_B_ENDPOINT_ADDRESS_EP_DIR_MASK) != 0;
      const bool interrupt =
        (endpointDescriptor->bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK) ==
        USB_BM_ATTRIBUTES_XFER_INT;
      if (input && interrupt) {
        inputEndpoint = endpointDescriptor->bEndpointAddress;
        inputMps = USB_EP_DESC_GET_MPS(endpointDescriptor);
        return inputMps >= 8;
      }
    }

    offset += descriptor->bLength;
  }
  return false;
}

static void usbKeyboardTryCleanup() {
  UsbKeyboardHostState &host = usbKeyboardHost;
  if (!host.disconnectRequested || host.inputInFlight || host.controlInFlight) {
    return;
  }

  usbKeyboardMarkDisconnected();
  if (host.interfaceClaimed && host.device != nullptr) {
    esp_err_t error = usb_host_interface_release(
      host.client, host.device, host.interfaceNumber);
    if (error != ESP_OK) {
      Serial.printf("USB keyboard interface release failed: %s\r\n",
                    esp_err_to_name(error));
    }
    host.interfaceClaimed = false;
  }
  if (host.inputTransfer != nullptr) {
    usb_host_transfer_free(host.inputTransfer);
    host.inputTransfer = nullptr;
  }
  if (host.controlTransfer != nullptr) {
    usb_host_transfer_free(host.controlTransfer);
    host.controlTransfer = nullptr;
  }
  if (host.device != nullptr) {
    esp_err_t error = usb_host_device_close(host.client, host.device);
    if (error != ESP_OK) {
      Serial.printf("USB keyboard device close failed: %s\r\n",
                    esp_err_to_name(error));
    }
    host.device = nullptr;
  }

  host.disconnectRequested = false;
  host.inputEndpoint = 0;
  host.inputMps = 0;
  Serial.println("USB keyboard disconnected");
}

static void usbKeyboardOpenDevice(uint8_t address) {
  UsbKeyboardHostState &host = usbKeyboardHost;
  if (host.device != nullptr) {
    return;   // one native root port, one keyboard at a time
  }

  usb_device_handle_t device = nullptr;
  esp_err_t error = usb_host_device_open(host.client, address, &device);
  if (error != ESP_OK) {
    Serial.printf("USB device open failed: %s\r\n", esp_err_to_name(error));
    return;
  }

  const usb_config_desc_t *config = nullptr;
  uint8_t interfaceNumber = 0;
  uint8_t alternateSetting = 0;
  uint8_t inputEndpoint = 0;
  uint16_t inputMps = 0;
  error = usb_host_get_active_config_descriptor(device, &config);
  if (error != ESP_OK || config == nullptr ||
      !usbKeyboardFindInterface(config, interfaceNumber, alternateSetting,
                                inputEndpoint, inputMps)) {
    usb_host_device_close(host.client, device);
    Serial.println("USB device has no boot-keyboard interface; ignored");
    return;
  }

  error = usb_host_interface_claim(host.client, device, interfaceNumber,
                                   alternateSetting);
  if (error != ESP_OK) {
    usb_host_device_close(host.client, device);
    Serial.printf("USB keyboard interface claim failed: %s\r\n", esp_err_to_name(error));
    return;
  }

  host.device = device;
  host.interfaceNumber = interfaceNumber;
  host.alternateSetting = alternateSetting;
  host.inputEndpoint = inputEndpoint;
  host.inputMps = inputMps;
  host.interfaceClaimed = true;
  host.disconnectRequested = false;

  error = usb_host_transfer_alloc(inputMps, 0, &host.inputTransfer);
  if (error == ESP_OK) {
    error = usb_host_transfer_alloc(USB_SETUP_PACKET_SIZE + 1, 0,
                                    &host.controlTransfer);
  }
  if (error != ESP_OK) {
    Serial.printf("USB keyboard transfer allocation failed: %s\r\n", esp_err_to_name(error));
    host.disconnectRequested = true;
    return;
  }

  host.inputTransfer->callback = usbKeyboardInputComplete;
  host.inputTransfer->context = &host;
  host.controlTransfer->callback = usbKeyboardControlComplete;
  host.controlTransfer->context = &host;

  const usb_device_desc_t *deviceDescriptor = nullptr;
  if (usb_host_get_device_descriptor(device, &deviceDescriptor) == ESP_OK &&
      deviceDescriptor != nullptr) {
    Serial.printf("USB keyboard found: VID=%04X PID=%04X interface=%u endpoint=0x%02X\r\n",
                  deviceDescriptor->idVendor, deviceDescriptor->idProduct,
                  interfaceNumber, inputEndpoint);
  } else {
    Serial.printf("USB keyboard found: interface=%u endpoint=0x%02X\r\n",
                  interfaceNumber, inputEndpoint);
  }

  if (!usbKeyboardSubmitControl(USB_KB_CONTROL_SET_PROTOCOL)) {
    usbKeyboardSubmitInput();
  }
}

static void usbKeyboardClientEvent(const usb_host_client_event_msg_t *event, void *arg) {
  (void)arg;
  if (event->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
    usbKeyboardOpenDevice(event->new_dev.address);
  } else if (event->event == USB_HOST_CLIENT_EVENT_DEV_GONE &&
             event->dev_gone.dev_hdl == usbKeyboardHost.device) {
    usbKeyboardHost.disconnectRequested = true;
    usbKeyboardMarkDisconnected();
  }
}

static void usbKeyboardServiceLedRequest() {
  UsbKeyboardHostState &host = usbKeyboardHost;
  if (!usbKeyboardConnected || host.controlInFlight || host.disconnectRequested) {
    return;
  }

  bool pending;
  uint8_t mask;
  portENTER_CRITICAL(&usbKeyboardLedMux);
  pending = usbKeyboardLedPending;
  mask = usbKeyboardPendingLedMask;
  portEXIT_CRITICAL(&usbKeyboardLedMux);
  if (!pending || !usbKeyboardSubmitControl(USB_KB_CONTROL_SET_LED, mask)) {
    return;
  }

  portENTER_CRITICAL(&usbKeyboardLedMux);
  if (usbKeyboardPendingLedMask == mask) {
    usbKeyboardLedPending = false;
  }
  portEXIT_CRITICAL(&usbKeyboardLedMux);
}

static void usbHostLibraryTask(void *notifyTask) {
  usb_host_config_t config = {};
  config.skip_phy_setup = false;
  config.root_port_unpowered = false;
  config.intr_flags = ESP_INTR_FLAG_LEVEL1;
  esp_err_t error = usb_host_install(&config);
  usbHostLibraryReady = (error == ESP_OK);
  xTaskNotifyGive((TaskHandle_t)notifyTask);
  if (error != ESP_OK) {
    Serial.printf("USB Host install failed: %s\r\n", esp_err_to_name(error));
    vTaskDelete(nullptr);
    return;
  }

  for (;;) {
    uint32_t eventFlags = 0;
    usb_host_lib_handle_events(portMAX_DELAY, &eventFlags);
  }
}

static void usbKeyboardClientTask(void *arg) {
  (void)arg;
  usb_host_client_config_t config = {};
  config.is_synchronous = false;
  config.max_num_event_msg = 5;
  config.async.client_event_callback = usbKeyboardClientEvent;
  config.async.callback_arg = nullptr;
  esp_err_t error = usb_host_client_register(&config, &usbKeyboardHost.client);
  if (error != ESP_OK) {
    Serial.printf("USB keyboard client registration failed: %s\r\n", esp_err_to_name(error));
    vTaskDelete(nullptr);
    return;
  }

  Serial.println("USB keyboard host ready; waiting for a boot keyboard");
  for (;;) {
    usb_host_client_handle_events(usbKeyboardHost.client, pdMS_TO_TICKS(20));
    usbKeyboardTryCleanup();
    usbKeyboardServiceLedRequest();
    if (usbKeyboardConnected && !usbKeyboardHost.inputInFlight &&
        !usbKeyboardHost.disconnectRequested) {
      usbKeyboardSubmitInput();
    }
  }
}

static void usbKeyboardHostBegin() {
  TaskHandle_t currentTask = xTaskGetCurrentTaskHandle();
  BaseType_t created = xTaskCreatePinnedToCore(
    usbHostLibraryTask, "usb-host", 4096, currentTask, 2, nullptr, 0);
  if (created != pdPASS) {
    Serial.println("Could not create USB Host library task");
    return;
  }

  ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000));
  if (!usbHostLibraryReady) {
    Serial.println("USB keyboard host unavailable");
    return;
  }

  created = xTaskCreatePinnedToCore(
    usbKeyboardClientTask, "usb-keyboard", 6144, nullptr, 3, nullptr, 0);
  if (created != pdPASS) {
    Serial.println("Could not create USB keyboard client task");
  }
}

static void usbKeyboardQueueLedMask(uint8_t mask) {
  portENTER_CRITICAL(&usbKeyboardLedMux);
  usbKeyboardPendingLedMask = mask & 0x1F;
  usbKeyboardLedPending = true;
  portEXIT_CRITICAL(&usbKeyboardLedMux);
}

#else

static void usbKeyboardHostBegin() {
  Serial.println("USB keyboard host disabled: select USB-OTG mode with USB CDC On Boot disabled");
}

static void usbKeyboardQueueLedMask(uint8_t mask) {
  (void)mask;
}

#endif

static uint8_t reportIdFor(const Peer &peer, NimBLERemoteCharacteristic *characteristic) {
  for (uint8_t i = 0; i < peer.reportCount; i++) {
    if (peer.reports[i].characteristic == characteristic) {
      return peer.reports[i].reportId;
    }
  }
  return 0;
}

static void dumpInputReport(const Peer &peer, uint8_t reportId, const uint8_t *data, size_t length) {
  Serial.print("report from ");
  Serial.print(peer.name);
  Serial.print(" id=");
  Serial.print(reportId);
  Serial.print(" len=");
  Serial.print(length);
  Serial.print(" :");
  for (size_t i = 0; i < length; i++) {
    Serial.printf(" %02X", data[i]);
  }
  Serial.println();
}

static Peer *findPeerByClient(const NimBLEClient *lookup) {
  if (lookup == nullptr) {
    return nullptr;
  }
  for (Peer &peer : peers) {
    if (peer.inUse && peer.client == lookup) {
      return &peer;
    }
  }
  return nullptr;
}

// Decides how to read this peer's reports. An explicit "PAD" override wins;
// otherwise whatever the advertisement said at connect time; failing both, the
// report's own length settles it -- a keyboard report is 8 bytes (9 with a
// leading report ID), an Xbox pad's main report is 14-16. The verdict is latched
// on the peer so the rest of its session parses consistently.
static PeerKind resolvePeerKind(Peer &peer, size_t length) {
  if (peer.kind != PEER_UNKNOWN) {
    return peer.kind;
  }

  if (length >= 14) {
    peer.kind = PEER_GAMEPAD;
    Serial.print(peer.name);
    Serial.println(": input reports look like a gamepad (>=14 bytes); parsing as one");
  } else if (length >= 8) {
    peer.kind = PEER_KEYBOARD;
    Serial.print(peer.name);
    Serial.println(": input reports look like a keyboard (8 bytes); parsing as one");
  }
  return peer.kind;
}

static void inputNotifyCallback(
  NimBLERemoteCharacteristic *characteristic,
  uint8_t *data,
  size_t length,
  bool isNotify
) {
  (void)isNotify;

  // Which device sent this. Two peers' notifications land in the same callback,
  // and every scrap of decode state below is per-peer, so an unattributable
  // report is dropped rather than folded into the wrong device's state.
  Peer *peer = findPeerByClient(characteristic->getClient());
  if (peer == nullptr) {
    return;
  }

  const uint8_t reportId = reportIdFor(*peer, characteristic);
  if (reportDump) {
    dumpInputReport(*peer, reportId, data, length);
  }

  if (resolvePeerKind(*peer, length) == PEER_GAMEPAD) {
    handleGamepadInputReport(*peer, reportId, data, length);
    return;
  }

  handleKeyboardReport(*peer, data, length);
}

// Frees a slot and wipes its decode state, so the next device to land in it
// cannot inherit half a session from the last one.
static void releasePeer(Peer &peer) {
  peer.inUse = false;
  peer.connected = false;
  peer.needsConfigure = false;
  peer.kind = PEER_UNKNOWN;
  peer.name = "";
  peer.address = NimBLEAddress();
  peer.hidOutputReport = nullptr;
  peer.bootOutputReport = nullptr;
  peer.reportCount = 0;
  memset(peer.lastKeys, 0, sizeof(peer.lastKeys));
  peer.lastModifiers = 0;
  peer.f12Prev = false;
  peer.padButtons = 0;
  peer.padPrevKeyButtons = 0;
  peer.padRepeatBit = 0;
  peer.padRepeatAt = 0;
  peer.guidePrev = false;
  clearPeerGameState(peer);
}

class ClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient *pClient) override {
    Peer *peer = findPeerByClient(pClient);
    if (peer == nullptr) {
      return;
    }
    // HID setup (encryption, service discovery, subscribing) happens in loop():
    // it blocks on pairing, and this callback runs on the NimBLE host task.
    peer->needsConfigure = true;
    connectPending = false;
    Serial.print("BLE link connected: ");
    Serial.println(peer->name);
  }

  void onConnectFail(NimBLEClient *pClient, int reason) override {
    Peer *peer = findPeerByClient(pClient);
    if (peer == nullptr) {
      return;
    }
    Serial.print("BLE connect failed for ");
    Serial.print(peer->name);
    Serial.print(", reason=");
    Serial.print(reason);
    Serial.print(" (");
    Serial.print(bleErrorText(reason));
    Serial.println(")");

    releasePeer(*peer);
    connectPending = false;
    shouldScan = true;
    nextScanAt = millis() + CONNECT_RETRY_DELAY_MS;
  }

  void onDisconnect(NimBLEClient *pClient, int reason) override {
    Peer *peer = findPeerByClient(pClient);
    if (peer == nullptr) {
      return;
    }

    Serial.printf("BLE disconnected (%s), reason=%d\r\n", peer->name.c_str(), reason);

    // Release anything the DS still thinks this device was holding. Its bitmap
    // is only ever cleared by our 0xF1 events, so a controller that walks out of
    // range mid-press would otherwise leave the character running into a wall.
    // Done before the slot is freed, and merged with whatever the *other* peer
    // is still holding rather than blanket-releasing everything.
    const bool wasContributing = peer->gbMask != 0 || peer->gbQuit || peer->gbMenu;
    clearPeerGameState(*peer);
    if (gameMode && wasContributing) {
      emitMergedGamepadState();
    }

    releasePeer(*peer);
    connectPending = false;
    shouldScan = true;
    nextScanAt = millis() + SCAN_RESTART_DELAY_MS;
  }

  void onAuthenticationComplete(NimBLEConnInfo &connInfo) override {
    if (!connInfo.isEncrypted()) {
      Serial.println("Pairing/encryption failed");
      NimBLEDevice::getClientByHandle(connInfo.getConnHandle())->disconnect();
      return;
    }

    Serial.print("Authentication complete: bonded=");
    Serial.print(connInfo.isBonded() ? "yes" : "no");
    Serial.print(" encrypted=");
    Serial.print(connInfo.isEncrypted() ? "yes" : "no");
    Serial.print(" id=");
    Serial.println(connInfo.getIdAddress().toString().c_str());
  }
};

static ClientCallbacks clientCallbacks;

// Reads the Report Reference descriptor (0x2908): byte 0 is the report ID, byte
// 1 the type (1 = input, 2 = output, 3 = feature). Returns false when the
// descriptor is missing, in which case reportId is left alone.
static bool readReportReference(
  NimBLERemoteCharacteristic *characteristic,
  uint8_t *reportId,
  uint8_t *reportType
) {
  NimBLERemoteDescriptor *reportRef = characteristic->getDescriptor(REPORT_REF_UUID);
  if (reportRef == nullptr) {
    return false;
  }

  NimBLEAttValue value = reportRef->readValue();
  if (value.size() < 2) {
    return false;
  }

  *reportId = value[0];
  *reportType = value[1];
  return true;
}

static bool reportReferenceIs(NimBLERemoteCharacteristic *characteristic, uint8_t reportType) {
  uint8_t id = 0;
  uint8_t type = 0;
  return readReportReference(characteristic, &id, &type) && type == reportType;
}

static void subscribeIfInputReport(Peer &peer, NimBLERemoteCharacteristic *characteristic, uint8_t *count) {
  if (!characteristic->canNotify()) {
    return;
  }

  const NimBLEUUID uuid = characteristic->getUUID();
  uint8_t reportId = 0;
  uint8_t reportType = 0;
  const bool haveRef = readReportReference(characteristic, &reportId, &reportType);

  const bool isBootInput = uuid == BOOT_KEYBOARD_INPUT_UUID;
  const bool isInputReport = uuid == REPORT_UUID && haveRef && reportType == 0x01;
  if (!isBootInput && !isInputReport) {
    return;
  }

  if (!characteristic->subscribe(true, inputNotifyCallback)) {
    return;
  }
  (*count)++;

  // Remember which report ID this characteristic speaks. A keyboard only ever
  // has one input report so this never mattered before; a gamepad splits its
  // state across several and the notify callback has no other way to tell them
  // apart (see InputReportRef).
  if (peer.reportCount < MAX_INPUT_REPORTS) {
    peer.reports[peer.reportCount].characteristic = characteristic;
    peer.reports[peer.reportCount].reportId = isBootInput ? 1 : reportId;
    peer.reportCount++;
  }
  Serial.printf("Subscribed to input report id=%u\r\n", isBootInput ? 1 : reportId);
}

static void rememberIfKeyboardOutput(Peer &peer, NimBLERemoteCharacteristic *characteristic) {
  const NimBLEUUID uuid = characteristic->getUUID();
  if (uuid == BOOT_KEYBOARD_OUTPUT_UUID) {
    peer.bootOutputReport = characteristic;
    return;
  }

  if (uuid == REPORT_UUID && reportReferenceIs(characteristic, 0x02)) {
    peer.hidOutputReport = characteristic;
  }
}

// Writes one peer's output report. Returns false if it has nowhere to write to,
// which is the normal case for a gamepad.
static bool sendPeerOutputReport(Peer &peer, const uint8_t *payload, size_t length) {
  NimBLERemoteCharacteristic *target =
    peer.hidOutputReport != nullptr ? peer.hidOutputReport : peer.bootOutputReport;
  if (!peer.connected || target == nullptr) {
    return false;
  }
  return target->writeValue(payload, length, true);
}

// Writes the supplied snapshot without changing the desired bridge-wide mask.
// Keeping those separate matters when a notification changes Caps Lock while
// loop() is still waiting for an earlier LED write to finish.
static bool writeLedMask(uint8_t mask) {
  mask &= 0x1F;
  uint8_t written = 0;
  uint8_t attempted = 0;
  const uint8_t payload[] = {mask};
  for (Peer &peer : peers) {
    if (!peer.connected || peer.kind == PEER_GAMEPAD) {
      continue;
    }
    attempted++;
    if (sendPeerOutputReport(peer, payload, sizeof(payload))) {
      written++;
    }
  }
  if (usbKeyboardConnected) {
    attempted++;
    usbKeyboardQueueLedMask(mask);
    written++;
  }

  if (attempted == 0) {
    Serial.println("LED command saved; no keyboard connected");
    return false;
  }

  Serial.printf("LED report written to %u of %u keyboard(s)\r\n", written, attempted);
  return written > 0;
}

// Lock LEDs go to every connected keyboard -- with two of them the mask is a
// bridge-wide setting, not one device's, so they should agree.
static bool sendLedMask(uint8_t mask) {
  ledMask = mask & 0x1F;
  return writeLedMask(ledMask);
}

static const char *bleErrorText(int error) {
  switch (error) {
    case BLE_HS_ETIMEOUT:
      return "host connection timeout";
    case BLE_HS_EBUSY:
      return "BLE stack busy";
    case BLE_HS_EAUTHEN:
      return "authentication failed";
    case BLE_HS_HCI_ERR(BLE_ERR_PINKEY_MISSING):
      return "PIN or bond key missing";
    case BLE_HS_HCI_ERR(BLE_ERR_CONN_SPVN_TMO):
      return "connection supervision timeout";
    case BLE_HS_HCI_ERR(BLE_ERR_CONN_ESTABLISHMENT):
      return "connection failed to establish";
    default:
      return "unknown";
  }
}

// True for advertisements that look like a game controller rather than a
// keyboard. The BLE appearance value is the reliable signal -- an Xbox Wireless
// Controller advertises 0x03C4 ("HID Gamepad") -- and the name is a fallback for
// pads that leave appearance unset.
static bool deviceLooksLikeGamepad(const NimBLEAdvertisedDevice *device) {
  if (device == nullptr) {
    return false;
  }

  if (device->haveAppearance() && device->getAppearance() == APPEARANCE_GAMEPAD) {
    return true;
  }

  if (!device->haveName()) {
    return false;
  }

  String name = device->getName().c_str();
  name.toLowerCase();
  return name.indexOf("xbox") >= 0 || name.indexOf("controller") >= 0 ||
         name.indexOf("gamepad") >= 0;
}

static Peer *findFreePeerSlot() {
  for (Peer &peer : peers) {
    if (!peer.inUse) {
      return &peer;
    }
  }
  return nullptr;
}

static Peer *findPeerByAddress(const NimBLEAddress &address) {
  for (Peer &peer : peers) {
    if (peer.inUse && peer.address == address) {
      return &peer;
    }
  }
  return nullptr;
}

// Starts connecting to whatever the scan callback found. Always asynchronous:
// the reconnect case needs it (a directed advertisement can be gone before a
// blocking connect returns), and using one path for both means there is a single
// place where a slot is claimed and a single place -- onConnect -> needsConfigure
// -- where HID setup begins.
static bool beginConnect(const NimBLEAddress &address, const String &name, bool isGamepad) {
  if (findPeerByAddress(address) != nullptr) {
    return false;   // already connected or connecting to this one
  }

  Peer *peer = findFreePeerSlot();
  if (peer == nullptr) {
    return false;
  }

  NimBLEClient *newClient = NimBLEDevice::getClientByPeerAddress(address);
  if (newClient == nullptr) {
    newClient = NimBLEDevice::getDisconnectedClient();
  }
  if (newClient == nullptr) {
    newClient = NimBLEDevice::createClient();
  }
  if (newClient == nullptr) {
    Serial.println("Could not allocate a NimBLE client");
    return false;
  }

  // A recycled client may still be pointed at whichever slot used it last.
  if (Peer *stale = findPeerByClient(newClient)) {
    if (stale != peer) {
      releasePeer(*stale);
    }
  }

  releasePeer(*peer);
  peer->inUse = true;
  peer->client = newClient;
  peer->address = address;
  peer->name = name.length() > 0 ? name : String("(unnamed)");
  // Only a positive gamepad signal is trusted; anything else stays UNKNOWN so
  // resolvePeerKind() can settle it from the first report's length rather than
  // mis-parsing a pad that advertised nothing useful as a keyboard.
  peer->kind = peerKindForced != PEER_UNKNOWN ? PeerKind(peerKindForced)
               : isGamepad                    ? PEER_GAMEPAD
                                              : PEER_UNKNOWN;

  newClient->setClientCallbacks(&clientCallbacks, false);
  newClient->setConnectionParams(12, 24, 0, 150);
  newClient->setConnectTimeout(10000);
  newClient->setConnectRetries(3);

  // The scanner and the initiator share one radio: connecting while the scan is
  // still running starves connection establishment and fails with HCI 0x3E
  // (reason 574). Stop the scan first; loop() restarts it once this settles.
  NimBLEScan *scan = NimBLEDevice::getScan();
  if (scan->isScanning()) {
    scan->stop();
  }

  connectPending = true;
  shouldScan = false;

  Serial.print("Connecting to ");
  Serial.print(peer->name);
  Serial.print(" ");
  Serial.println(address.toString().c_str());

  if (!newClient->connect(address, true, true, false)) {
    int error = newClient->getLastError();
    Serial.print("BLE connect could not start, reason=");
    Serial.print(error);
    Serial.print(" (");
    Serial.print(bleErrorText(error));
    Serial.println(")");
    releasePeer(*peer);
    connectPending = false;
    shouldScan = true;
    nextScanAt = millis() + CONNECT_RETRY_DELAY_MS;
    return false;
  }

  return true;
}

static bool configurePeer(Peer &peer) {
  NimBLEClient *peerClient = peer.client;
  if (peerClient == nullptr || !peerClient->isConnected()) {
    return false;
  }

  // Encrypt the link before touching the HID service. Keyboards get away without
  // this: their report characteristics demand authentication, so the first read
  // or CCCD write makes NimBLE start pairing on its own. An Xbox pad answers
  // those requests unencrypted and then simply never notifies -- the subscribe
  // "succeeds", the console says bonded=no encrypted=no, and no button does
  // anything. Asking for security explicitly is the only way to get it, and on a
  // link that is already encrypted (a bonded device reconnecting, where the keys
  // are restored during connection setup) it is skipped.
  if (!peerClient->getConnInfo().isEncrypted()) {
    Serial.println("Link is not encrypted; starting pairing");
    if (!peerClient->secureConnection()) {
      Serial.println("Pairing failed; disconnecting");
      peerClient->disconnect();
      return false;
    }
    Serial.println("Link encrypted");
  }

  NimBLERemoteService *hidService = peerClient->getService(HID_SERVICE_UUID);
  if (hidService == nullptr) {
    Serial.println("HID service missing");
    peerClient->disconnect();
    return false;
  }

  NimBLERemoteCharacteristic *protocolMode = hidService->getCharacteristic(PROTOCOL_MODE_UUID);
  if (protocolMode != nullptr && protocolMode->canWrite()) {
    uint8_t reportMode = 1;
    protocolMode->writeValue(&reportMode, 1, true);
  }

  uint8_t inputReportCount = 0;
  peer.reportCount = 0;
  const std::vector<NimBLERemoteCharacteristic *> &chars = hidService->getCharacteristics(true);
  for (NimBLERemoteCharacteristic *characteristic : chars) {
    subscribeIfInputReport(peer, characteristic, &inputReportCount);
    rememberIfKeyboardOutput(peer, characteristic);
  }

  if (inputReportCount == 0) {
    Serial.println("No HID input reports found");
    peerClient->disconnect();
    return false;
  }

  peer.connected = true;
  Serial.print("Subscribed to input reports: ");
  Serial.println(inputReportCount);
  rememberPeer(peer);

  // Stop hunting for new devices once a slot is filled by pairing -- otherwise
  // the next unrelated HID device to advertise would be grabbed as well.
  if (pairingMode) {
    pairingMode = false;
  }

  // Lock LEDs are a keyboard thing; a pad has no output report to write them to,
  // and asking for one just logs a failure on every connect.
  if (peer.kind != PEER_GAMEPAD) {
    writeLedMask(ledMask);
  }
  return true;
}

class ScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice *advertisedDevice) override {
    if (!advertisedDevice->isConnectable() || connectPending) {
      return;
    }

    // Skip anything already in a slot. Scanning now continues while devices are
    // connected (that is how the second one gets found), so a peer's own
    // advertisements come back around and would otherwise be re-connected.
    if (findPeerByAddress(advertisedDevice->getAddress()) != nullptr) {
      return;
    }

    // Not every pad puts 0x1812 in its advertisement, so a gamepad appearance or
    // name is accepted as a second way in; the HID service still has to be there
    // once connected (configurePeer bails otherwise).
    const bool hidDevice = advertisedDevice->isAdvertisingService(HID_SERVICE_UUID) ||
                           deviceLooksLikeGamepad(advertisedDevice);
    const bool savedDevice = keyboardMatchesSavedIdentity(advertisedDevice);
    const bool bondedDevice = NimBLEDevice::isBonded(advertisedDevice->getAddress());

    if (pairingMode) {
      if (!hidDevice) {
        return;
      }
    } else if (!savedDevice && !bondedDevice) {
      return;
    }

    Serial.print(pairingMode ? "Found BLE HID device for pairing: " : "Found saved BLE device: ");
    Serial.println(advertisedDevice->toString().c_str());

    // Copy what we need out of the advertisement here: the NimBLEAdvertisedDevice
    // belongs to the scan results and does not outlive the next scan.
    beginConnect(advertisedDevice->getAddress(),
                 advertisedDevice->haveName() ? String(advertisedDevice->getName().c_str()) : String(""),
                 deviceLooksLikeGamepad(advertisedDevice));
  }

  void onScanEnd(const NimBLEScanResults &results, int reason) override {
    (void)results;
    Serial.printf("BLE scan ended, reason=%d\r\n", reason);
    if (!connectPending && wantMorePeers()) {
      shouldScan = true;
      nextScanAt = millis() + (connectedPeerCount() > 0 ? SCAN_IDLE_DELAY_MS
                                                        : SCAN_RESTART_DELAY_MS);
    }
  }
};

static ScanCallbacks scanCallbacks;

static void startScan() {
  // Scanning continues while a slot is free *and* there is something worth
  // finding -- that is what lets a keyboard and a pad both be picked up without
  // re-pairing, without leaving the radio scanning forever behind a controller
  // that is already connected. See wantMorePeers().
  if (connectPending || !wantMorePeers()) {
    return;
  }

  Serial.println(pairingMode
    ? "Scanning for a new BLE HID device..."
    : "Scanning for saved BLE devices...");
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&scanCallbacks, false);
  scan->setInterval(100);
  scan->setWindow(100);
  scan->setActiveScan(true);
  scan->start(SCAN_TIME_MS, false, true);
}

static int parseBinaryArg(const String &arg) {
  if (arg == "1" || arg.equalsIgnoreCase("ON")) {
    return 1;
  }
  if (arg == "0" || arg.equalsIgnoreCase("OFF")) {
    return 0;
  }
  return -1;
}

static void setCapsLockState(bool on) {
  capsLockOn = on;
  ledMask = (ledMask & ~uint8_t(0x02)) | (on ? uint8_t(0x02) : uint8_t(0x00));

  // emitKey() runs inside NimBLE's notification callback. A responded GATT
  // write waits for the NimBLE host task, so doing it there deadlocks the task
  // that must deliver the response. Defer only the LED transaction to loop();
  // capsLockOn already changes immediately for the next key report.
  ledMaskWritePending = true;
}

static void servicePendingLedMaskWrite() {
  if (!ledMaskWritePending) {
    return;
  }

  const uint8_t mask = ledMask;
  ledMaskWritePending = false;
  writeLedMask(mask);
}

static void setLedBit(uint8_t bit, const String &arg) {
  int value = parseBinaryArg(arg);
  if (value < 0) {
    Serial.println("Use 0/1 or ON/OFF");
    return;
  }

  if (bit == 0x02) {
    setCapsLockState(value == 1);
    return;
  }

  if (value) {
    ledMask |= bit;
  } else {
    ledMask &= ~bit;
  }
  sendLedMask(ledMask);
}

static bool parseHexByte(const char *text, uint8_t *value) {
  char *end = nullptr;
  unsigned long parsed = strtoul(text, &end, 16);
  if (end == text || parsed > 0xFF) {
    return false;
  }
  *value = uint8_t(parsed);
  return true;
}

static void sendRawOutputReport(String args) {
  uint8_t payload[16];
  uint8_t count = 0;
  args.trim();

  char buffer[64];
  args.toCharArray(buffer, sizeof(buffer));
  char *token = strtok(buffer, " ,");
  while (token != nullptr && count < sizeof(payload)) {
    if (!parseHexByte(token, &payload[count])) {
      Serial.println("Bad hex byte in OUT command");
      return;
    }
    count++;
    token = strtok(nullptr, " ,");
  }

  if (count == 0) {
    Serial.println("OUT needs at least one hex byte");
    return;
  }

  uint8_t written = 0;
  for (Peer &peer : peers) {
    if (peer.connected && sendPeerOutputReport(peer, payload, count)) {
      written++;
    }
  }

  if (written == 0) {
    Serial.println("No peer has a writable output report");
    return;
  }
  Serial.printf("Raw output report written to %u peer(s)\r\n", written);
}

static const char *peerKindName(PeerKind kind) {
  return kind == PEER_GAMEPAD ? "gamepad" : kind == PEER_KEYBOARD ? "keyboard" : "unknown";
}

static void listPeers() {
  Serial.print("peers: ");
  Serial.print(connectedPeerCount());
  Serial.print(" connected of ");
  Serial.println(MAX_PEERS);

  for (uint8_t i = 0; i < MAX_PEERS; i++) {
    Peer &peer = peers[i];
    Serial.print("  ");
    Serial.print(i + 1);
    Serial.print(": ");
    if (!peer.inUse) {
      Serial.println("(free)");
      continue;
    }
    Serial.print(peer.connected ? "connected" : "connecting");
    Serial.print(" kind=");
    Serial.print(peerKindName(peer.kind));
    Serial.print(" reports=");
    Serial.print(peer.reportCount);
    Serial.print(" ");
    Serial.print(peer.address.toString().c_str());
    Serial.print(" ");
    Serial.println(peer.name);
  }
}

// "PAD [slot] AUTO|KEYBOARD|GAMEPAD". The slot number comes from STATUS. It may
// be omitted when exactly one peer is connected, which is the common case; with
// two connected there is no sensible default, so it is required.
static void handlePadCommand(String args) {
  args.trim();
  args.toUpperCase();

  int slot = -1;
  int split = args.indexOf(' ');
  if (split > 0 && isdigit(args[0])) {
    slot = args.substring(0, split).toInt() - 1;
    args = args.substring(split + 1);
    args.trim();
  } else if (args.length() > 0 && isdigit(args[0]) && args.length() <= 2) {
    Serial.println("Use PAD [slot] AUTO|KEYBOARD|GAMEPAD");
    return;
  }

  PeerKind kind;
  if (args == "AUTO" || args.length() == 0) {
    kind = PEER_UNKNOWN;
  } else if (args == "KEYBOARD" || args == "KB") {
    kind = PEER_KEYBOARD;
  } else if (args == "GAMEPAD" || args == "PAD" || args == "XBOX") {
    kind = PEER_GAMEPAD;
  } else {
    Serial.println("Use PAD [slot] AUTO|KEYBOARD|GAMEPAD");
    return;
  }

  // No slot given: fall back to the only connected peer, or to setting the
  // default applied to whatever connects next if nothing is connected yet.
  if (slot < 0) {
    if (connectedPeerCount() > 1) {
      Serial.println("Two peers connected; say which: PAD <slot> AUTO|KEYBOARD|GAMEPAD (see STATUS)");
      return;
    }
    for (uint8_t i = 0; i < MAX_PEERS; i++) {
      if (peers[i].connected) {
        slot = i;
        break;
      }
    }
    if (slot < 0) {
      peerKindForced = kind;
      Serial.print("Nothing connected; peers will default to ");
      Serial.println(kind == PEER_UNKNOWN ? "auto-detect" : peerKindName(kind));
      return;
    }
  }

  if (slot < 0 || slot >= MAX_PEERS || !peers[slot].inUse) {
    Serial.println("No such peer slot; see STATUS");
    return;
  }

  peers[slot].kind = kind;
  Serial.print("Peer ");
  Serial.print(slot + 1);
  Serial.print(" (");
  Serial.print(peers[slot].name);
  Serial.print(") parsed as ");
  Serial.println(kind == PEER_UNKNOWN ? "auto-detect (next report decides)" : peerKindName(kind));
}

static void handleCommand(String line, const char *source) {
  line.trim();
  if (line.length() == 0) {
    return;
  }

  int split = line.indexOf(' ');
  String command = split < 0 ? line : line.substring(0, split);
  String args = split < 0 ? "" : line.substring(split + 1);
  command.toUpperCase();
  args.trim();

  Serial.print(source);
  Serial.print("> ");
  Serial.println(line);

  if (command == "HELP") {
    printCommandHelp();
  } else if (command == "STATUS") {
    listPeers();
    Serial.print("game=");
    Serial.print(gameMode ? "on" : "off");
    Serial.print(" ledMask=");
    Serial.print(ledMask);
    Serial.print(" savedKeyboards=");
    Serial.print(savedKeyboardCount);
    Serial.print(" spiffs=");
    Serial.print(spiffsReady ? "mounted" : "not-mounted");
    Serial.print(" bonds=");
    Serial.print(NimBLEDevice::getNumBonds());
    Serial.print(" pairing=");
    Serial.print(pairingMode ? "enabled" : "off");
    Serial.print(" connecting=");
    Serial.print(connectPending ? "yes" : "no");
    Serial.print(" scan=");
    Serial.print(NimBLEDevice::getScan()->isScanning() ? "active" : "idle");
    Serial.print(" usbKeyboard=");
    Serial.print(usbKeyboardConnected ? "connected" : "idle");
    Serial.print(" oled=");
    Serial.print(statusDisplayStateName());
    Serial.print(" screen=");
    Serial.print(rotarySettingsActive ? "settings" : "volume");
    Serial.print(" encoder=");
#if SLAVE_ROTARY_ENCODER_ENABLED
    Serial.print(SLAVE_ROTARY_A_PIN);
    Serial.print("/");
    Serial.print(SLAVE_ROTARY_B_PIN);
    Serial.print(" button=");
    Serial.print(SLAVE_ROTARY_BUTTON_PIN);
#else
    Serial.print("disabled");
#endif
    Serial.print(" joystick=");
    Serial.print(joystickStateName());
#if SLAVE_BUTTON_BAR_ENABLED
    //left to right, matching the boot banner and "BTN"
    Serial.printf(" buttons=%d/%d/%d", SLAVE_BUTTON_C_PIN, SLAVE_BUTTON_B_PIN, SLAVE_BUTTON_A_PIN);
#else
    Serial.print(" buttons=disabled");
#endif
    Serial.print(" wifi=");
    Serial.print(WiFi.status() == WL_CONNECTED ? "connected" : "disconnected");
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print(" ip=");
      Serial.print(WiFi.localIP());
    }
    Serial.print(" telnet=");
    Serial.println(telnetClient && telnetClient.connected() ? "connected" : "idle");
  } else if (command == "KEYBOARDS" || command == "PAIRED") {
    listSavedKeyboards();
  } else if (command == "FORGET") {
    clearSavedKeyboards();
  } else if (command == "PAIR") {
    // Deliberately does *not* drop existing connections: pairing a second device
    // while the first stays live is the whole point of having two slots. If both
    // are taken, one has to go first -- say so rather than silently doing
    // nothing, which is what a full scan would amount to.
    NimBLEScan *scan = NimBLEDevice::getScan();
    if (scan->isScanning()) {
      scan->stop();
    }
    if (claimedPeerCount() >= MAX_PEERS) {
      Serial.println("All peer slots are in use; FORGET or power one off first");
    } else {
      pairingMode = true;
      shouldScan = true;
      nextScanAt = millis() + SCAN_RESTART_DELAY_MS;
      Serial.println("Pairing mode enabled; put the keyboard or controller in pairing mode");
    }
  } else if (command == "SCAN" || command == "RECONNECT") {
    NimBLEScan *scan = NimBLEDevice::getScan();
    if (scan->isScanning()) {
      scan->stop();
    }
    pairingMode = false;
    shouldScan = true;
    nextScanAt = millis() + SCAN_RESTART_DELAY_MS;
    Serial.println("Reconnect scan requested (existing connections kept)");
  } else if (command == "DROP") {
    disconnectAllPeers();
    shouldScan = true;
    nextScanAt = millis() + SCAN_RESTART_DELAY_MS;
    Serial.println("Disconnected all peers");
  } else if (command == "LED") {
    sendLedMask(uint8_t(args.toInt()));
  } else if (command == "NUM") {
    setLedBit(0x01, args);
  } else if (command == "CAPS") {
    setLedBit(0x02, args);
  } else if (command == "SCROLL") {
    setLedBit(0x04, args);
  } else if (command == "OUT") {
    sendRawOutputReport(args);
  } else if (command == "GAME") {
    int value = parseBinaryArg(args);
    if (value < 0) {
      Serial.println("Use GAME 0|1");
    } else {
      applyGameMode(value == 1, "GAME command");
    }
  } else if (command == "PAD") {
    handlePadCommand(args);
  } else if (command == "DUMP") {
    int value = parseBinaryArg(args);
    if (value < 0) {
      Serial.println("Use DUMP 0|1");
    } else {
      reportDump = (value == 1);
      Serial.print("Input report hex dump ");
      Serial.println(reportDump ? "on" : "off");
    }
  } else if (command == "JOY") {
    String joyArgs = args;
    joyArgs.toUpperCase();
    handleJoystickCommand(joyArgs);
  } else if (command == "BTN" || command == "BUTTONS") {
    handleButtonBarCommand();
  } else {
    Serial.print("Unknown command: ");
    Serial.println(command);
  }
}

static void clearTelnetOutputQueue() {
  portENTER_CRITICAL(&telnetOutputMux);
  telnetOutputHead = 0;
  telnetOutputTail = 0;
  portEXIT_CRITICAL(&telnetOutputMux);
}

static void drainTelnetOutput() {
  if (!telnetOutputEnabled || !telnetClient || !telnetClient.connected()) {
    return;
  }

  uint8_t chunk[256];
  size_t count = 0;

  portENTER_CRITICAL(&telnetOutputMux);
  while (telnetOutputTail != telnetOutputHead && count < sizeof(chunk)) {
    chunk[count++] = telnetOutputBuffer[telnetOutputTail];
    telnetOutputTail = (telnetOutputTail + 1) % TELNET_OUTPUT_BUFFER_SIZE;
  }
  portEXIT_CRITICAL(&telnetOutputMux);

  if (count > 0) {
    telnetClient.write(chunk, count);
  }
}

static void sendTelnetOption(uint8_t command, uint8_t option) {
  const uint8_t response[] = {255, command, option};
  telnetClient.write(response, sizeof(response));
}

static void resetTelnetParser() {
  telnetLine = "";
  telnetParserState = 0;
  telnetPendingCommand = 0;
  telnetSawCarriageReturn = false;
}

static void finishTelnetCommand() {
  telnetClient.print("\r\n");
  handleCommand(telnetLine, "TELNET");
  telnetLine = "";
  Serial.print("ds> ");
}

static void processTelnetDataByte(uint8_t value) {
  requestStatusFlash(2);

  if (value == '\r') {
    finishTelnetCommand();
    telnetSawCarriageReturn = true;
    return;
  }

  if (value == '\n') {
    if (!telnetSawCarriageReturn) {
      finishTelnetCommand();
    }
    telnetSawCarriageReturn = false;
    return;
  }

  telnetSawCarriageReturn = false;
  if (value == 8 || value == 127) {
    if (telnetLine.length() > 0) {
      telnetLine.remove(telnetLine.length() - 1);
      telnetClient.print("\b \b");
    }
    return;
  }

  if (value == 3) {
    telnetLine = "";
    telnetClient.print("^C\r\n");
    Serial.print("ds> ");
    return;
  }

  if (value >= 32 && value < 127) {
    if (telnetLine.length() < 80) {
      telnetLine += char(value);
      telnetClient.write(value);
    } else {
      telnetClient.write(uint8_t('\a'));
    }
  }
}

static void processTelnetByte(uint8_t value) {
  static constexpr uint8_t TELNET_DATA = 0;
  static constexpr uint8_t TELNET_IAC = 1;
  static constexpr uint8_t TELNET_OPTION = 2;
  static constexpr uint8_t TELNET_SUBNEGOTIATION = 3;
  static constexpr uint8_t TELNET_SUBNEGOTIATION_IAC = 4;
  static constexpr uint8_t IAC = 255;
  static constexpr uint8_t DONT = 254;
  static constexpr uint8_t DO = 253;
  static constexpr uint8_t WONT = 252;
  static constexpr uint8_t WILL = 251;
  static constexpr uint8_t SB = 250;
  static constexpr uint8_t SE = 240;
  static constexpr uint8_t ECHO = 1;
  static constexpr uint8_t SUPPRESS_GO_AHEAD = 3;

  switch (telnetParserState) {
    case TELNET_DATA:
      if (value == IAC) {
        telnetParserState = TELNET_IAC;
      } else {
        processTelnetDataByte(value);
      }
      break;

    case TELNET_IAC:
      if (value == WILL || value == WONT || value == DO || value == DONT) {
        telnetPendingCommand = value;
        telnetParserState = TELNET_OPTION;
      } else if (value == SB) {
        telnetParserState = TELNET_SUBNEGOTIATION;
      } else {
        telnetParserState = TELNET_DATA;
      }
      break;

    case TELNET_OPTION:
      if (telnetPendingCommand == WILL && value != SUPPRESS_GO_AHEAD) {
        sendTelnetOption(DONT, value);
      } else if (telnetPendingCommand == DO &&
                 value != ECHO && value != SUPPRESS_GO_AHEAD) {
        sendTelnetOption(WONT, value);
      }
      telnetParserState = TELNET_DATA;
      break;

    case TELNET_SUBNEGOTIATION:
      if (value == IAC) {
        telnetParserState = TELNET_SUBNEGOTIATION_IAC;
      }
      break;

    case TELNET_SUBNEGOTIATION_IAC:
      telnetParserState = value == SE ? TELNET_DATA : TELNET_SUBNEGOTIATION;
      break;
  }
}

static void acceptTelnetClient() {
  WiFiClient incoming = telnetServer.accept();
  if (!incoming) {
    return;
  }

  telnetOutputEnabled = false;
  clearTelnetOutputQueue();
  if (telnetClient) {
    telnetClient.stop();
  }

  telnetClient = incoming;
  telnetClient.setNoDelay(true);
  resetTelnetParser();

  // Server-side echo, suppress-go-ahead in both directions.
  sendTelnetOption(251, 1);
  sendTelnetOption(251, 3);
  sendTelnetOption(253, 3);

  telnetOutputEnabled = true;
  Serial.println();
  Serial.println("DS-Slave Telnet console connected");
  printCommandHelp();
  Serial.print("ds> ");
}

static void serviceTelnetClient() {
  if (!telnetClient) {
    return;
  }

  if (!telnetClient.connected()) {
    telnetOutputEnabled = false;
    clearTelnetOutputQueue();
    telnetClient.stop();
    resetTelnetParser();
    Serial.println("Telnet client disconnected");
    return;
  }

  while (telnetClient.available()) {
    int value = telnetClient.read();
    if (value >= 0) {
      processTelnetByte(uint8_t(value));
    }
  }

  drainTelnetOutput();
}

static void beginWiFi() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  nextWiFiAttemptAt = millis() + WIFI_RETRY_INTERVAL_MS;
  Serial.print("Connecting to WiFi SSID ");
  Serial.println(WIFI_SSID);
}

static void serviceWiFiAndTelnet() {
  wl_status_t status = WiFi.status();
  if (status != previousWiFiStatus) {
    previousWiFiStatus = status;
    if (status == WL_CONNECTED) {
      telnetServer.begin();
      telnetServer.setNoDelay(true);
      telnetServerRunning = true;
      Serial.print("WiFi connected: ");
      Serial.print(WiFi.localIP());
      Serial.print(" telnet port ");
      Serial.println(TELNET_PORT);
    } else {
      telnetOutputEnabled = false;
      clearTelnetOutputQueue();
      if (telnetClient) {
        telnetClient.stop();
      }
      if (telnetServerRunning) {
        telnetServer.end();
        telnetServerRunning = false;
      }
      Serial.print("WiFi disconnected, status=");
      Serial.println(int(status));
    }
  }

  if (status != WL_CONNECTED) {
    if (int32_t(millis() - nextWiFiAttemptAt) >= 0) {
      nextWiFiAttemptAt = millis() + WIFI_RETRY_INTERVAL_MS;
      Serial.println("Retrying WiFi connection");
      WiFi.reconnect();
    }
    return;
  }

  acceptTelnetClient();
  serviceTelnetClient();
}

static void pumpCommandStream(Stream &stream, String &line, const char *source) {
  while (stream.available()) {
    char c = char(stream.read());
    if (strcmp(source, "UART1") == 0) {
      Serial0.write(uint8_t(c));
    }
    requestStatusFlash(2);
    if (c == '\n' || c == '\r') {
      handleCommand(line, source);
      line = "";
    } else if (line.length() < 80) {
      line += c;
    } else {
      line = "";
      Serial.print(source);
      Serial.println(" command too long; discarded");
    }
  }
}

static void pumpUartCommands() {
  pumpCommandStream(LinkSerial, uartLine, "UART1");
}

static void pumpConsoleCommands() {
  pumpCommandStream(Serial, consoleLine, "USB");
}

static void printCommandHelp() {
  Serial.println("Commands from the debug console, Telnet, or UART1 RX:");
  Serial.println("  HELP");
  Serial.println("  STATUS");
  Serial.println("  KEYBOARDS");
  Serial.println("  FORGET [ALL]");
  Serial.println("  PAIR              add a device; existing connections are kept");
  Serial.println("  SCAN");
  Serial.println("  RECONNECT");
  Serial.println("  DROP              disconnect every peer (bonds are kept)");
  Serial.println("  LED <0-31>");
  Serial.println("  NUM 0|1");
  Serial.println("  CAPS 0|1");
  Serial.println("  SCROLL 0|1");
  Serial.println("  OUT <hex bytes>");
  Serial.println("  GAME 0|1");
  Serial.println("  PAD [slot] AUTO|KEYBOARD|GAMEPAD");
  Serial.println("  DUMP 0|1");
  Serial.println("  JOY [CAL|0|1]     analog joystick: report, recentre, or force off/on");
  Serial.println("  BTN               button bar: raw pin levels and press state");
  Serial.printf("Up to %u devices at once (a keyboard and a controller); STATUS lists the slots.\r\n",
                MAX_PEERS);
  Serial.println("Modifiers: Ctrl sends control bytes where possible, Alt prefixes ESC, Cmd uses CSI-u.");
  Serial.println("GAME 1: send Game Boy button events (0xF0 down/0xF1 up/0xF2 quit) instead of keystrokes.");
  Serial.println("F12 (keyboard) or Guide (gamepad) also toggles gameMode, regardless of this command link.");
  Serial.println("Gamepad, GAME 0: dpad/stick=arrows A=Enter B=Esc X=Bksp Y=Tab LB/RB=PgUp/PgDn RT=Space.");
  Serial.println("Gamepad, GAME 1: dpad/stick, A/B, Menu=Start, View=Select, LB=menu, LB+RB=quit.");
  Serial.println("DUMP 1 hex-logs raw input reports here -- use it to check an unfamiliar pad's layout.");
  Serial.println("Joystick: axes send arrow keys (D-pad in GAME 1), stick click sends Select (Esc).");
  Serial.println("Button bar, left to right: Start, B, A (GAME 1) / 0xF8, 0xFA, 0xFB private bytes (GAME 0).");
  Serial.println("Those bytes are context controls on the DS: prev/pause/next for music+radio, else gb/radio/music.");
}

void setup() {
  Serial.begin(USB_BAUD);
  Serial0.begin(SERIAL0_MIRROR_BAUD);
  gpio_deep_sleep_hold_dis();             // Release the UART idle-high hold left by deep sleep.
  gpio_hold_dis(gpio_num_t(UART_TX_PIN));
  LinkSerial.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  sendMainWakeBeacon();                    // Wake DOLL-OS after any slave reset, including rotary wake.
  FastLED.addLeds<WS2812, STATUS_LED_PIN, GRB>(statusLed, STATUS_LED_COUNT);
  FastLED.setBrightness(STATUS_LED_BRIGHTNESS);
  setStatusColor(baseStatusColor());

  delay(200);
  Serial.println();
  Serial.println("DS-Slave BLE/USB keyboard UART bridge");
  Serial.printf("UART1 TX=%d RX=%d baud=%lu\r\n", UART_TX_PIN, UART_RX_PIN, (unsigned long)UART_BAUD);
  Serial.printf("Serial0 mirror baud=%lu\r\n", (unsigned long)SERIAL0_MIRROR_BAUD);
  Serial.printf("Status WS2812 pin=%d brightness=%u\r\n", STATUS_LED_PIN, STATUS_LED_BRIGHTNESS);
  rotaryEncoderBegin();
  joystickBegin();
  buttonBarBegin();
  statusDisplayBegin();

  usbKeyboardHostBegin();

  spiffsReady = SPIFFS.begin(true);
  if (spiffsReady) {
    loadKeyboardStore();
    Serial.print("SPIFFS mounted; saved keyboards=");
    Serial.println(savedKeyboardCount);
  } else {
    Serial.println("SPIFFS mount failed; keyboard registry disabled");
  }

  printCommandHelp();
  beginWiFi();

  NimBLEDevice::init("DS-Slave");
  NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_PUBLIC);
  NimBLEDevice::setPower(3);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  // Bond, no MITM: the status OLED is read-only and there is no passkey/confirm
  // input path. Offer LE Secure Connections; Xbox controllers pair with SC and
  // refuse to hand over input reports without it. Keyboards that only do legacy
  // pairing negotiate down to it, so asking for SC costs them nothing.
  NimBLEDevice::setSecurityAuth(BLE_SM_PAIR_AUTHREQ_BOND | BLE_SM_PAIR_AUTHREQ_SC);
  NimBLEDevice::setSecurityInitKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
  NimBLEDevice::setSecurityRespKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);

  pairingMode = NimBLEDevice::getNumBonds() == 0;
  Serial.print("NimBLE bonds=");
  Serial.print(NimBLEDevice::getNumBonds());
  Serial.print(" pairingMode=");
  Serial.println(pairingMode ? "enabled" : "off");
  nextScanAt = millis();
  startScan();
}

void loop() {
  serviceWiFiAndTelnet();
  pumpUartCommands();
  pumpConsoleCommands();
  servicePendingLedMaskWrite();
  serviceGamepadRepeat();
  serviceRotaryEncoder();
  serviceJoystick();
  serviceButtonBar();
  updateStatusLed();
  updateStatusDisplay(false);

  if (connectedPeerCount() == 0 && !usbKeyboardConnected &&
      millis() - lastDisconnectedLogAt >= DISCONNECTED_LOG_INTERVAL_MS) {
    lastDisconnectedLogAt = millis();
    Serial.print("Disconnected: scan=");
    Serial.print(NimBLEDevice::getScan()->isScanning() ? "active" : "idle");
    Serial.print(" pairing=");
    Serial.print(pairingMode ? "enabled" : "off");
    Serial.print(" connecting=");
    Serial.print(connectPending ? "yes" : "no");
    Serial.print(" savedKeyboards=");
    Serial.println(savedKeyboardCount);
  }

  // HID setup for anything that finished connecting since the last pass. Done
  // here rather than in onConnect because it blocks on pairing and on GATT
  // reads, and onConnect runs on the NimBLE host task.
  for (Peer &peer : peers) {
    if (!peer.needsConfigure) {
      continue;
    }
    peer.needsConfigure = false;
    if (configurePeer(peer)) {
      Serial.print("Ready: ");
      Serial.print(peer.name);
      Serial.print(" as ");
      Serial.println(peerKindName(peer.kind));

      // beginConnect stopped the scan to free the radio. Re-arm it here, or the
      // second device would never be looked for again -- wantMorePeers() decides
      // whether there is any point.
      if (wantMorePeers()) {
        shouldScan = true;
        nextScanAt = millis() + SCAN_IDLE_DELAY_MS;
      }
    } else {
      // configurePeer disconnects on failure, which frees the slot via
      // onDisconnect; release here too in case it never got that far.
      releasePeer(peer);
      shouldScan = true;
      nextScanAt = millis() + CONNECT_RETRY_DELAY_MS;
    }
  }

  if (shouldScan && !connectPending && wantMorePeers() &&
      int32_t(millis() - nextScanAt) >= 0 &&
      !NimBLEDevice::getScan()->isScanning()) {
    shouldScan = false;
    startScan();
  }

  delay(10);
}
