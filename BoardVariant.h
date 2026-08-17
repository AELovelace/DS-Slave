#pragma once

// DS-Slave physical build options. Keep case/wiring direction quirks here so
// the main sketch only describes behavior.

// Set to 1 if clockwise turns the volume down or moves settings backward.
#define SLAVE_ROTARY_REVERSED 1

// Set to 1 if the joystick sends left for right, or down for up. Which way a
// module reads depends on how it is soldered and which way it is glued into the
// case, so it is a build option rather than something the sketch can know.
// "JOY" on the console prints the live axis values while you push the stick.
#define SLAVE_JOYSTICK_INVERT_X 0
#define SLAVE_JOYSTICK_INVERT_Y 0

// Button bar wiring. 0 = the bar's common rail is 3V3 and each switch closes
// onto its GPIO, so a press reads high. Set to 1 if the common rail is GND and
// the switches pull their pin down instead. "BTN" on the console prints the raw
// pin levels, which is the quickest way to tell which one you have.
#define SLAVE_BUTTON_BAR_ACTIVE_LOW 0
