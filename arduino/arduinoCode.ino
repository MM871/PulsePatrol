#include <Joystick.h>
#include <Servo.h>
#include <stdint.h>

// ─────────────────────────────────────────────────────────────────────────────
// PulsePatrol — Arduino Side
//
// This sketch does three things in a loop:
//   1. Reads the joystick position and converts it to a servo angle (0–180°)
//   2. Fires the ultrasonic sensor and measures how far away the nearest
//      object is
//   3. Packs both values into a 16-bit frame and clocks it out to the FPGA
//      over a simple 3-wire bit-bang protocol (SYNC, CLOCK, DATA)
//
// The FPGA side (main.c) listens on JP1 GPIO pins and reconstructs each
// packet on rising clock edges.
// ─────────────────────────────────────────────────────────────────────────────

// ─── Ultrasonic sensor pins ──────────────────────────────────────────────────
const int trig = 5;  // TRIG output — sends the ultrasonic pulse
const int echo = 6;  // ECHO input  — goes HIGH for as long as the echo takes

// Speed of sound: 0.0343 cm/µs, divided by 2 because the pulse travels
// to the object AND back before we measure it
const float speed = 0.0343 / 2.0;

// ─── GPIO bit-bang pins (connect to JP1 header on DE1-SoC) ──────────────────
//
//   DATA  → JP1 bit 0  — the actual data bit being sent
//   CLOCK → JP1 bit 1  — FPGA samples DATA on every rising edge
//   SYNC  → JP1 bit 2  — held HIGH for the duration of one full packet
//
// CLOCK is on pin 2 specifically because pin 2 is INT0 on the Uno —
// useful if you ever want interrupt-driven transmission instead of polling.
const int DATA_PIN = 11;
const int CLOCK_PIN = 2;
const int SYNC_PIN = 10;

// ─── Servo and joystick objects
// ───────────────────────────────────────────────
Servo myServo;
Joystick joy(A0, A1, 7);  // X axis on A0, Y axis on A1, button on pin 7

// ─────────────────────────────────────────────────────────────────────────────
// sendByte — clock out one byte, MSB first
//
// For each of the 8 bits (starting from the most significant):
//   - Put the bit on DATA_PIN
//   - Pulse CLOCK high for 20 µs then low for 20 µs
//   - FPGA latches DATA on the rising edge of CLOCK
// ─────────────────────────────────────────────────────────────────────────────
void sendByte(uint8_t data) {
  for (int i = 7; i >= 0; i--) {
    // Extract bit i and drive it onto the DATA line
    digitalWrite(DATA_PIN, (data >> i) & 0x01);

    // Rising edge — FPGA samples here
    digitalWrite(CLOCK_PIN, HIGH);
    delayMicroseconds(20);

    // Falling edge — get ready for the next bit
    digitalWrite(CLOCK_PIN, LOW);
    delayMicroseconds(20);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// sendPacket — wrap two bytes in a SYNC frame and send them
//
// Frame structure:
//   SYNC HIGH  ──┐
//   [20 ms gap]  │  tells the FPGA "a packet is starting"
//   byte 0: distance (8 bits)
//   byte 1: angle    (8 bits)
//   SYNC LOW   ──┘  tells the FPGA "packet is done"
//
// The FPGA polls SYNC first, then waits for CLOCK to go LOW (idle),
// then shifts in exactly 16 bits on rising edges.
// ─────────────────────────────────────────────────────────────────────────────
void sendPacket(uint8_t distance, uint8_t angle) {
  // Assert SYNC to signal start of frame
  digitalWrite(SYNC_PIN, HIGH);
  delay(20);  // give the FPGA time to notice SYNC before clocking starts

  sendByte(distance);  // high byte — distance in cm
  sendByte(angle);     // low byte  — angle in degrees

  // De-assert SYNC to signal end of frame
  digitalWrite(SYNC_PIN, LOW);
}

// ─────────────────────────────────────────────────────────────────────────────
// readDistance — fire the HC-SR04 and return distance in cm
//
// The HC-SR04 works like this:
//   1. Pull TRIG LOW briefly to reset it
//   2. Pulse TRIG HIGH for 10 µs — this fires the ultrasonic burst
//   3. Measure how long ECHO stays HIGH — that's the round-trip travel time
//   4. Multiply by speed of sound (halved) to get distance in cm
//
// pulseIn times out after 30 ms (≈ 5 m range — well beyond our radar's scope).
// If it times out, we return 255 as a sentinel meaning "no return / too far".
// ─────────────────────────────────────────────────────────────────────────────
uint8_t readDistance() {
  // Reset the trigger line
  digitalWrite(trig, LOW);
  delayMicroseconds(2);

  // Fire the ultrasonic pulse
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  // Measure echo duration — timeout after 30 ms
  long duration = pulseIn(echo, HIGH, 30000);

  // Timeout means nothing came back — signal this with 255
  if (duration == 0) return 255;

  float raw = duration * speed;

  // Clamp to uint8_t range (254 max, 255 reserved as sentinel)
  return (raw > 254) ? 255 : (uint8_t)raw;
}

// ─────────────────────────────────────────────────────────────────────────────
// readAngle — read joystick X axis and drive the servo to match
//
// The joystick X axis is mapped to 0–180°.
// We flip it (180 - raw) so pushing the joystick right sweeps the radar
// to the right on screen too — without the flip it would be mirrored.
// The servo physically moves to match, pointing the HC-SR04 at the right angle.
// ─────────────────────────────────────────────────────────────────────────────
uint8_t readAngle() {
  int raw = joy.readX(0, 180);  // map joystick X to 0–180
  int disp = 180 - raw;         // flip so screen direction matches physical

  myServo.write(disp);   // physically rotate the servo
  return (uint8_t)disp;  // return angle to encode in packet
}

// ─────────────────────────────────────────────────────────────────────────────
// setup — runs once on power-up
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);  // UART debug output — view in Serial Monitor

  // Ultrasonic sensor
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  // GPIO bit-bang lines — all start LOW (idle)
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(SYNC_PIN, OUTPUT);
  digitalWrite(DATA_PIN, LOW);
  digitalWrite(CLOCK_PIN, LOW);
  digitalWrite(SYNC_PIN, LOW);

  // Attach servo to PWM pin 9
  myServo.attach(9);
}

// ─────────────────────────────────────────────────────────────────────────────
// loop — runs forever
//
// Every 10 ms:
//   1. Read the joystick and move the servo
//   2. Fire the ultrasonic sensor
//   3. Send both values to the FPGA as a 16-bit packet
//   4. Print to Serial for debugging
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  uint8_t angle = readAngle();        // read joystick + move servo
  uint8_t distance = readDistance();  // fire sensor + measure return

  sendPacket(distance, angle);  // transmit to FPGA over GPIO

  // Debug output — useful when tuning sensor or protocol timing
  Serial.print("Angle: ");
  Serial.print(angle);
  Serial.print(" | Distance: ");
  Serial.println(distance);

  delay(10);  // ~100 packets per second — fast enough for smooth radar
}