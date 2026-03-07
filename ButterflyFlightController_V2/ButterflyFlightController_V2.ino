/*
 * RC Butterfly Flight Controller V2 (ESP32-C3 Mini + FrSky RXS-R)
 *
 * Smooth sinusoidal wing motion with advanced controls,
 * ported from the Ctorque MicroPython ornithopter code.
 *
 * Uses F.Port protocol from the uninverted S.Port pad on RXS-R.
 * Sends battery telemetry back to the transmitter via F.Port.
 *
 * Features:
 *   - Sinusoidal flap motion (cosine wave, 40 steps per cycle)
 *   - Variable flap amplitude via stick/knob
 *   - Pitch control (shifts wing center for climb/dive)
 *   - Per-wing trim correction via aux channels
 *   - Differential turning
 *   - Battery voltage monitoring with low-pass filter
 *   - F.Port telemetry: battery voltage + percentage on transmitter
 *   - WS2812 LED effects on wings (synced to flap motion)
 *   - Non-blocking: F.Port is read every loop, not blocked during flap
 *
 * Channel Mapping (configure on your transmitter):
 *   Ch 0  Throttle   -> Flap speed (low=slow, high=fast)
 *   Ch 1  Aileron    -> Turning (differential wing amplitude)
 *   Ch 2  Elevator   -> Pitch control (wing center offset)
 *   Ch 3  Rudder     -> Flap amplitude (small to large)
 *   Ch 4  Aux 1      -> Flap enable/disable (switch, >center = on)
 *   Ch 5  Aux 2      -> Left wing trim
 *   Ch 6  Aux 3      -> Right wing trim
 *   Ch 7  Aux 4      -> LED on/off (switch, >center = on)
 *   Ch 8  Aux 5      -> LED mode (3-pos: solid / rainbow / flap-sync)
 *   Ch 9  Aux 6      -> LED brightness (pot/slider)
 *   Ch 10 Aux 7      -> LED color hue (pot/slider)
 *
 * Telemetry (appears on transmitter automatically):
 *   VFAS  -> Battery voltage
 *   Fuel  -> Battery percentage
 *
 * Wiring:
 *   F.Port (receiver P pad) -> GPIO 7
 *   Left wing servo         -> GPIO 4
 *   Right wing servo        -> GPIO 5
 *   Battery voltage divider  -> GPIO 0
 *   WS2812 LED data line    -> GPIO 3
 *
 * Battery circuit:
 *   VBAT ──[ R1 ]──┬──[ R2 ]── GND
 *                  └── GPIO 0
 *   Divider ratio = (R1 + R2) / R2
 *   Ctorque board uses ratio of 4 (e.g. R1=30k, R2=10k)
 *   Adjust VDIV_RATIO below to match your resistors.
 */

#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>
#include "driver/gpio.h"

// ── Pin Configuration ───────────────────────────────────────────────
#define FPORT_RX_PIN    7
#define SERVO_LEFT_PIN  4
#define SERVO_RIGHT_PIN 5
#define BATTERY_ADC_PIN 0   // Must be ADC-capable (GPIO 0-4 on ESP32-C3)
#define LED_PIN         3   // WS2812 data line
#define LED_COUNT       10  // Total LEDs on the strip (adjust to your setup)

// ── UART for F.Port (half-duplex) ──────────────────────────────────
HardwareSerial fportSerial(1);

// ── Servos ──────────────────────────────────────────────────────────
Servo servoLeft;
Servo servoRight;

// ── WS2812 LEDs ─────────────────────────────────────────────────────
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// ── Channel Mapping ─────────────────────────────────────────────────
#define CH_THROTTLE     0  // Flap speed
#define CH_AILERON      1  // Turning / differential
#define CH_ELEVATOR     2  // Pitch offset
#define CH_RUDDER       3  // Flap amplitude
#define CH_FLAP_ENABLE  4  // On/off switch
#define CH_TRIM_LEFT    5  // Left wing trim
#define CH_TRIM_RIGHT   6  // Right wing trim
#define CH_LED_ENABLE   7  // LED on/off switch
#define CH_LED_MODE     8  // LED mode (3-pos switch)
#define CH_LED_BRIGHT   9  // LED brightness (pot/slider)
#define CH_LED_COLOR    10 // LED color hue (pot/slider)

// ── F.Port Constants ────────────────────────────────────────────────
#define FPORT_HEADER     0x7E
#define FPORT_ESCAPE     0x7D
#define FPORT_ESCAPE_XOR 0x20

// Frame types
#define FPORT_TYPE_CONTROL  0x00   // RC channel data
#define FPORT_TYPE_DOWNLINK 0x01   // Telemetry poll from receiver
#define FPORT_TYPE_UPLINK   0x81   // Telemetry response from FC

// Parser result codes
#define FRAME_NONE 0
#define FRAME_RC   1
#define FRAME_POLL 2

// SmartPort sensor app IDs (show up on transmitter)
#define APPID_VFAS 0x0210  // Battery voltage (centivolts)
#define APPID_FUEL 0x0600  // Fuel level (percentage)

// Our telemetry physical ID (0x00-0x1B, receiver auto-discovers)
#define TELEM_PHYS_ID 0x00

// ── Channel Value Range (S.Bus encoding) ────────────────────────────
const uint16_t CH_MIN    = 172;
const uint16_t CH_MAX    = 1811;
const uint16_t CH_CENTER = 992;

// ── Servo Limits ────────────────────────────────────────────────────
const int SERVO_CENTER    = 90;
const int SERVO_LIMIT_MIN = 0;
const int SERVO_LIMIT_MAX = 180;

// ── Sinusoidal Flap Parameters ──────────────────────────────────────
// 40 steps per full cosine cycle (matches Ctorque's 20 up + 20 down)
const int   FLAP_STEPS      = 40;
const float PHASE_INCREMENT = TWO_PI / FLAP_STEPS;

// Step delay range (ms) — controls flap speed
// Full cycle time = FLAP_STEPS * delay = 320ms (fast) to 640ms (slow)
const int DELAY_MIN_MS = 8;   // Fastest (full throttle)
const int DELAY_MAX_MS = 16;  // Slowest (zero throttle)

// Amplitude range (degrees from center)
const float AMP_MIN = 20.0;
const float AMP_MAX = 70.0;

// Differential range for turning (degrees)
const float DIFF_MAX = 20.0;

// Pitch offset range (degrees)
const float PITCH_MAX = 20.0;

// Per-wing trim range (degrees)
const float TRIM_MAX = 20.0;

// ── Battery Monitoring ──────────────────────────────────────────────
// Voltage divider ratio: (R1+R2)/R2. Ctorque board uses 4.0.
// Set this to match your resistor divider so the ADC reads within range.
const float VDIV_RATIO = 4.0;

// Battery voltage thresholds — adjust for your cell count
// 1S LiPo: min=3.3  max=4.2
// 2S LiPo: min=6.6  max=8.4
const float BAT_VOLT_MIN = 6.6;
const float BAT_VOLT_MAX = 8.4;

// Low-pass filter coefficient (0.0–1.0, higher = smoother but slower)
const float BAT_FILTER_ALPHA = 0.98;

// How often to read / print battery info (ms)
const unsigned long BAT_READ_INTERVAL_MS  = 100;
const unsigned long BAT_PRINT_INTERVAL_MS = 5000;

// Voltage at which to warn on serial
const float BAT_WARN_VOLTAGE = 7.0;

// ── WS2812 LED Settings ─────────────────────────────────────────────
// How often to update LED effects (ms) — ~40fps
const unsigned long LED_UPDATE_INTERVAL_MS = 25;

// LED modes (selected by 3-position switch on CH_LED_MODE)
#define LED_MODE_SOLID     0
#define LED_MODE_RAINBOW   1
#define LED_MODE_FLAPSYNC  2

// Rainbow cycle speed (higher = faster rotation)
const float RAINBOW_SPEED = 300.0;

// ── Failsafe ────────────────────────────────────────────────────────
const unsigned long FAILSAFE_TIMEOUT_MS = 500;

// ── Runtime State ───────────────────────────────────────────────────

// Flap state
float         flapPhase       = 0.0;
unsigned long lastStepTime    = 0;
bool          flappingEnabled = false;

// Control values (updated each F.Port frame)
int   servoDelayMs  = DELAY_MAX_MS;
float amplitude     = 45.0;
float differential  = 0.0;
float pitchOffset   = 0.0;
float leftTrim      = 0.0;
float rightTrim     = 0.0;

// Battery state
float         batteryVoltage     = 0.0;
int           batteryPercent     = 0;
float         smoothedAdcMv      = 0.0;
unsigned long lastBatReadTime    = 0;
unsigned long lastBatPrintTime   = 0;

// Telemetry state — cycles through sensors each poll
uint8_t telemSensorIndex = 0;

// LED state (updated from RC channels)
unsigned long lastLedUpdateTime = 0;
bool          ledEnabled        = true;
uint8_t       ledMode           = LED_MODE_RAINBOW;
uint8_t       ledBrightness     = 60;
uint16_t      ledColorHue       = 0;     // 0-65535 (NeoPixel HSV hue)
uint16_t      rainbowOffset     = 0;     // Auto-advancing rainbow position

// F.Port parsing state
uint16_t      channels[16];
uint8_t       fportBuffer[32];
int           bufferIndex        = 0;
bool          inFrame            = false;
bool          failsafeActive     = false;
unsigned long lastValidFrameTime = 0;
uint8_t       pollPhysId         = 0;


// ═══════════════════════════════════════════════════════════════════
//  F.Port Parser
// ═══════════════════════════════════════════════════════════════════

// Parse channel data from an RC control frame
bool parseFportChannels() {
  if (fportBuffer[0] != FPORT_HEADER)  return false;
  if (fportBuffer[1] != 0x19)          return false;  // len = 25
  if (fportBuffer[2] != FPORT_TYPE_CONTROL) return false;

  uint8_t* p = &fportBuffer[3];

  // Decode 16 channels from 22 bytes (11 bits each, same as S.Bus)
  channels[0]  = (p[0]       | p[1]  << 8)                & 0x07FF;
  channels[1]  = (p[1]  >> 3 | p[2]  << 5)                & 0x07FF;
  channels[2]  = (p[2]  >> 6 | p[3]  << 2 | p[4]  << 10)  & 0x07FF;
  channels[3]  = (p[4]  >> 1 | p[5]  << 7)                & 0x07FF;
  channels[4]  = (p[5]  >> 4 | p[6]  << 4)                & 0x07FF;
  channels[5]  = (p[6]  >> 7 | p[7]  << 1 | p[8]  << 9)   & 0x07FF;
  channels[6]  = (p[8]  >> 2 | p[9]  << 6)                & 0x07FF;
  channels[7]  = (p[9]  >> 5 | p[10] << 3)                & 0x07FF;
  channels[8]  = (p[11]      | p[12] << 8)                & 0x07FF;
  channels[9]  = (p[12] >> 3 | p[13] << 5)                & 0x07FF;
  channels[10] = (p[13] >> 6 | p[14] << 2 | p[15] << 10)  & 0x07FF;
  channels[11] = (p[15] >> 1 | p[16] << 7)                & 0x07FF;
  channels[12] = (p[16] >> 4 | p[17] << 4)                & 0x07FF;
  channels[13] = (p[17] >> 7 | p[18] << 1 | p[19] << 9)   & 0x07FF;
  channels[14] = (p[19] >> 2 | p[20] << 6)                & 0x07FF;
  channels[15] = (p[20] >> 5 | p[21] << 3)                & 0x07FF;

  return true;
}

// Read F.Port data — returns FRAME_NONE, FRAME_RC, or FRAME_POLL
int readFport() {
  while (fportSerial.available()) {
    uint8_t b = fportSerial.read();

    if (b == FPORT_HEADER) {
      if (inFrame && bufferIndex >= 4) {
        // End-of-frame marker found — identify the frame type
        fportBuffer[bufferIndex] = b;
        inFrame = false;

        uint8_t frameType = fportBuffer[2];

        if (frameType == FPORT_TYPE_CONTROL && bufferIndex > 25) {
          if (parseFportChannels()) {
            bufferIndex = 0;
            return FRAME_RC;
          }
        } else if (frameType == FPORT_TYPE_DOWNLINK) {
          // Telemetry poll — physical ID is at byte 3
          pollPhysId = fportBuffer[3];
          bufferIndex = 0;
          return FRAME_POLL;
        }
      }
      // Start of a new frame
      bufferIndex = 0;
      fportBuffer[bufferIndex++] = b;
      inFrame = true;
    } else if (inFrame) {
      if (bufferIndex < 32) {
        fportBuffer[bufferIndex++] = b;
      } else {
        inFrame = false;
        bufferIndex = 0;
      }
    }
  }
  return FRAME_NONE;
}


// ═══════════════════════════════════════════════════════════════════
//  F.Port Telemetry Response
// ═══════════════════════════════════════════════════════════════════

// FrSky CRC: sum with carry folding
uint8_t fportCrc(uint8_t *data, uint8_t len) {
  uint16_t sum = 0;
  for (uint8_t i = 0; i < len; i++) {
    sum += data[i];
    sum += sum >> 8;
    sum &= 0x00FF;
  }
  return 0xFF - (uint8_t)sum;
}

// Write a single byte with F.Port byte-stuffing
void fportWriteByte(uint8_t b) {
  if (b == FPORT_HEADER || b == FPORT_ESCAPE) {
    fportSerial.write(FPORT_ESCAPE);
    fportSerial.write(b ^ FPORT_ESCAPE_XOR);
  } else {
    fportSerial.write(b);
  }
}

// Send a telemetry uplink frame (response to a poll)
void sendTelemetry(uint16_t appId, uint32_t value) {
  // Build the raw frame (before byte-stuffing)
  // Format: [len] [type] [phys_id] [prim_lo] [prim_hi]
  //         [appid_lo] [appid_hi] [data x4]
  uint8_t frame[11];
  frame[0]  = 0x08;                      // len: 8 payload bytes follow
  frame[1]  = FPORT_TYPE_UPLINK;         // 0x81
  frame[2]  = pollPhysId;                // echo back the polled ID
  frame[3]  = 0x10;                      // prim low (data frame)
  frame[4]  = 0x00;                      // prim high
  frame[5]  = appId & 0xFF;              // app ID low
  frame[6]  = (appId >> 8) & 0xFF;       // app ID high
  frame[7]  = value & 0xFF;              // data byte 0
  frame[8]  = (value >> 8) & 0xFF;       // data byte 1
  frame[9]  = (value >> 16) & 0xFF;      // data byte 2
  frame[10] = (value >> 24) & 0xFF;      // data byte 3

  // CRC covers type through last data byte (frame[1]..frame[10])
  uint8_t crc = fportCrc(&frame[1], 10);

  // Brief pause to let the receiver release the bus
  delayMicroseconds(100);

  // Send frame bytes with byte-stuffing (no 7E delimiters)
  for (int i = 0; i < 11; i++) {
    fportWriteByte(frame[i]);
  }
  fportWriteByte(crc);

  // Wait for TX to finish, then flush our own echo from RX buffer
  fportSerial.flush();
  while (fportSerial.available()) fportSerial.read();
}

// Handle a telemetry poll — respond if it's our physical ID
void handleTelemetryPoll() {
  if (pollPhysId != TELEM_PHYS_ID) return;

  // Cycle through sensor values each poll
  switch (telemSensorIndex) {
    case 0:
      // VFAS: battery voltage in centivolts (e.g. 3.85V → 385)
      sendTelemetry(APPID_VFAS, (uint32_t)(batteryVoltage * 100.0));
      break;
    case 1:
      // Fuel: battery percentage (0-100)
      sendTelemetry(APPID_FUEL, (uint32_t)batteryPercent);
      break;
  }

  telemSensorIndex++;
  if (telemSensorIndex > 1) telemSensorIndex = 0;
}


// ═══════════════════════════════════════════════════════════════════
//  Battery Monitoring
// ═══════════════════════════════════════════════════════════════════

void initBattery() {
  analogReadResolution(12);

  // Prime the low-pass filter with initial readings
  float sum = 0;
  for (int i = 0; i < 50; i++) {
    sum += analogReadMilliVolts(BATTERY_ADC_PIN);
    delay(2);
  }
  smoothedAdcMv = sum / 50.0;

  // Calculate initial voltage
  batteryVoltage = (smoothedAdcMv / 1000.0) * VDIV_RATIO;
  batteryPercent = constrain(
    (int)(((batteryVoltage - BAT_VOLT_MIN) / (BAT_VOLT_MAX - BAT_VOLT_MIN)) * 100.0),
    0, 100
  );
}

void updateBattery() {
  if (millis() - lastBatReadTime < BAT_READ_INTERVAL_MS) return;
  lastBatReadTime = millis();

  // Read and smooth
  float rawMv = (float)analogReadMilliVolts(BATTERY_ADC_PIN);
  smoothedAdcMv = BAT_FILTER_ALPHA * smoothedAdcMv
                + (1.0 - BAT_FILTER_ALPHA) * rawMv;

  // Convert through voltage divider
  batteryVoltage = (smoothedAdcMv / 1000.0) * VDIV_RATIO;

  // Calculate percentage
  batteryPercent = constrain(
    (int)(((batteryVoltage - BAT_VOLT_MIN) / (BAT_VOLT_MAX - BAT_VOLT_MIN)) * 100.0),
    0, 100
  );
}

void printBattery() {
  if (millis() - lastBatPrintTime < BAT_PRINT_INTERVAL_MS) return;
  lastBatPrintTime = millis();

  Serial.print("Battery: ");
  Serial.print(batteryVoltage, 2);
  Serial.print("V  ");
  Serial.print(batteryPercent);
  Serial.print("%");

  if (batteryVoltage > 0.5 && batteryVoltage < BAT_WARN_VOLTAGE) {
    Serial.print("  ** LOW BATTERY **");
  }
  Serial.println();
}


// ═══════════════════════════════════════════════════════════════════
//  WS2812 LED Effects
// ═══════════════════════════════════════════════════════════════════

void initLeds() {
  strip.begin();
  strip.setBrightness(ledBrightness);
  strip.clear();
  strip.show();
}

// ── Mode 0: Solid color ─────────────────────────────────────────────
// All LEDs set to the hue selected by the color pot.
void ledEffectSolid() {
  for (int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, strip.ColorHSV(ledColorHue, 255, 255));
  }
}

// ── Mode 1: Rainbow ─────────────────────────────────────────────────
// Classic auto-cycling rainbow spread across the strip.
void ledEffectRainbow() {
  rainbowOffset += (uint16_t)RAINBOW_SPEED;
  for (int i = 0; i < LED_COUNT; i++) {
    uint16_t hue = rainbowOffset + (i * 65535 / LED_COUNT);
    strip.setPixelColor(i, strip.ColorHSV(hue, 255, 255));
  }
}

// ── Mode 2: Flap-synced dual-color ──────────────────────────────────
// Two complementary colors (set by color pot) alternate across the
// strip. As wings flap, the two colors swap brightness in opposition —
// at wing-up one color dominates, at wing-down the other takes over.
void ledEffectFlapSync() {
  float cosVal = cos(flapPhase);

  // Two complementary hues: base + 180 degrees on the color wheel
  uint16_t hueA = ledColorHue;
  uint16_t hueB = ledColorHue + 32768;  // +180 deg in NeoPixel HSV

  // Opposite brightness for each color group:
  // Color A glows when cosVal > 0 (wings in one direction)
  // Color B glows when cosVal < 0 (wings in the other direction)
  float blendA = (cosVal + 1.0) / 2.0;   // 0.0 - 1.0
  float blendB = 1.0 - blendA;

  uint8_t brightA = (uint8_t)(40 + blendA * 215);  // 40-255
  uint8_t brightB = (uint8_t)(40 + blendB * 215);

  for (int i = 0; i < LED_COUNT; i++) {
    if (i % 2 == 0) {
      strip.setPixelColor(i, strip.ColorHSV(hueA, 255, brightA));
    } else {
      strip.setPixelColor(i, strip.ColorHSV(hueB, 255, brightB));
    }
  }
}

// ── LED update (called every loop) ──────────────────────────────────
void updateLeds() {
  if (millis() - lastLedUpdateTime < LED_UPDATE_INTERVAL_MS) return;
  lastLedUpdateTime = millis();

  // Master on/off from RC switch
  if (!ledEnabled) {
    strip.clear();
    strip.show();
    return;
  }

  // Brightness from RC pot
  strip.setBrightness(ledBrightness);

  // Run the selected mode
  switch (ledMode) {
    case LED_MODE_SOLID:    ledEffectSolid();    break;
    case LED_MODE_RAINBOW:  ledEffectRainbow();  break;
    case LED_MODE_FLAPSYNC: ledEffectFlapSync(); break;
    default:                ledEffectRainbow();  break;
  }

  strip.show();
}


// ═══════════════════════════════════════════════════════════════════
//  Helpers
// ═══════════════════════════════════════════════════════════════════

float mapFloat(uint16_t val, uint16_t inMin, uint16_t inMax,
               float outMin, float outMax) {
  return (float)(val - inMin) * (outMax - outMin) /
         (float)(inMax - inMin) + outMin;
}


// ═══════════════════════════════════════════════════════════════════
//  Setup
// ═══════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  Serial.println("Butterfly Flight Controller V2 - Sinusoidal + Telemetry");
  Serial.println("Initializing...");

  // ── F.Port UART (RX only) ─────────────────────────────────────────
  // RX on GPIO 7, no TX (telemetry disabled)
  fportSerial.begin(115200, SERIAL_8N1, FPORT_RX_PIN, -1, false);
  while (fportSerial.available()) fportSerial.read();
  Serial.println("F.Port initialized on GPIO 7 (RX only)");

  // ── Servos ────────────────────────────────────────────────────────
  servoLeft.attach(SERVO_LEFT_PIN);
  servoRight.attach(SERVO_RIGHT_PIN);
  servoLeft.write(SERVO_CENTER);
  servoRight.write(SERVO_CENTER);
  Serial.print("Servos: Left GPIO ");
  Serial.print(SERVO_LEFT_PIN);
  Serial.print(" | Right GPIO ");
  Serial.println(SERVO_RIGHT_PIN);

  // ── Battery ───────────────────────────────────────────────────────
  initBattery();
  Serial.print("Battery ADC on GPIO ");
  Serial.print(BATTERY_ADC_PIN);
  Serial.print(" | Divider ratio: ");
  Serial.print(VDIV_RATIO, 1);
  Serial.print(" | Range: ");
  Serial.print(BAT_VOLT_MIN, 1);
  Serial.print("-");
  Serial.print(BAT_VOLT_MAX, 1);
  Serial.println("V");
  Serial.print("Initial reading: ");
  Serial.print(batteryVoltage, 2);
  Serial.print("V (");
  Serial.print(batteryPercent);
  Serial.println("%)");

  // ── WS2812 LEDs ──────────────────────────────────────────────────
  initLeds();
  Serial.print("WS2812 LEDs: ");
  Serial.print(LED_COUNT);
  Serial.print(" on GPIO ");
  Serial.println(LED_PIN);

  // ── Ready ─────────────────────────────────────────────────────────
  lastValidFrameTime = millis();
  lastStepTime = millis();
  lastBatPrintTime = millis();

  Serial.println();
  Serial.println("Channel map:");
  Serial.println("  Ch0 Throttle  -> Flap speed");
  Serial.println("  Ch1 Aileron   -> Turn (differential)");
  Serial.println("  Ch2 Elevator  -> Pitch offset");
  Serial.println("  Ch3 Rudder    -> Amplitude");
  Serial.println("  Ch4 Aux1      -> Flap enable (switch)");
  Serial.println("  Ch5 Aux2      -> Left trim");
  Serial.println("  Ch6 Aux3      -> Right trim");
  Serial.println("  Ch7 Aux4      -> LED on/off (switch)");
  Serial.println("  Ch8 Aux5      -> LED mode (3-pos: solid/rainbow/flap)");
  Serial.println("  Ch9 Aux6      -> LED brightness (pot)");
  Serial.println("  Ch10 Aux7     -> LED color hue (pot)");
  Serial.println();
  Serial.println("Telemetry -> VFAS (voltage), Fuel (percentage)");
  Serial.println("Ready - waiting for F.Port signal...");
}


// ═══════════════════════════════════════════════════════════════════
//  Main Loop
// ═══════════════════════════════════════════════════════════════════

void loop() {
  // ── 1. Read F.Port ────────────────────────────────────────────────
  int frameResult = readFport();

  if (frameResult == FRAME_RC) {
    lastValidFrameTime = millis();
    failsafeActive = false;

    // Flap enable: switch channel, above center = on
    flappingEnabled = (channels[CH_FLAP_ENABLE] > CH_CENTER);

    // Flap speed: throttle -> step delay (high throttle = short delay = fast)
    servoDelayMs = (int)mapFloat(channels[CH_THROTTLE],
                                 CH_MIN, CH_MAX,
                                 DELAY_MAX_MS, DELAY_MIN_MS);
    servoDelayMs = constrain(servoDelayMs, DELAY_MIN_MS, DELAY_MAX_MS);

    // Amplitude: rudder/knob -> wing sweep angle
    amplitude = mapFloat(channels[CH_RUDDER],
                         CH_MIN, CH_MAX,
                         AMP_MIN, AMP_MAX);
    amplitude = constrain(amplitude, AMP_MIN, AMP_MAX);

    // Turning: aileron -> differential (positive = turn right)
    differential = mapFloat(channels[CH_AILERON],
                            CH_MIN, CH_MAX,
                            -DIFF_MAX, DIFF_MAX);
    differential = constrain(differential, -DIFF_MAX, DIFF_MAX);

    // Pitch: elevator -> wing center offset (positive = nose up)
    pitchOffset = mapFloat(channels[CH_ELEVATOR],
                           CH_MIN, CH_MAX,
                           -PITCH_MAX, PITCH_MAX);
    pitchOffset = constrain(pitchOffset, -PITCH_MAX, PITCH_MAX);

    // Per-wing trims
    leftTrim = mapFloat(channels[CH_TRIM_LEFT],
                        CH_MIN, CH_MAX,
                        -TRIM_MAX, TRIM_MAX);
    leftTrim = constrain(leftTrim, -TRIM_MAX, TRIM_MAX);

    rightTrim = mapFloat(channels[CH_TRIM_RIGHT],
                         CH_MIN, CH_MAX,
                         -TRIM_MAX, TRIM_MAX);
    rightTrim = constrain(rightTrim, -TRIM_MAX, TRIM_MAX);

    // ── LED controls ──────────────────────────────────────────────
    // On/off switch
    ledEnabled = (channels[CH_LED_ENABLE] > CH_CENTER);

    // Mode: 3-position switch → low/mid/high thirds of channel range
    uint16_t modeVal = channels[CH_LED_MODE];
    if (modeVal < 718) {
      ledMode = LED_MODE_SOLID;
    } else if (modeVal < 1265) {
      ledMode = LED_MODE_RAINBOW;
    } else {
      ledMode = LED_MODE_FLAPSYNC;
    }

    // Brightness pot (0-255)
    ledBrightness = (uint8_t)mapFloat(channels[CH_LED_BRIGHT],
                                      CH_MIN, CH_MAX, 0.0, 255.0);

    // Color hue pot (0-65535, full color wheel)
    ledColorHue = (uint16_t)mapFloat(channels[CH_LED_COLOR],
                                     CH_MIN, CH_MAX, 0.0, 65535.0);

  } else if (frameResult == FRAME_POLL) {
    // Telemetry disabled (no TX pin) — ignore polls
  }

  // ── 2. Failsafe ──────────────────────────────────────────────────
  if (millis() - lastValidFrameTime > FAILSAFE_TIMEOUT_MS) {
    if (!failsafeActive) {
      failsafeActive = true;
      flappingEnabled = false;
      Serial.println("FAILSAFE - no F.Port signal");
    }
  }

  // ── 3. Battery monitoring ─────────────────────────────────────────
  updateBattery();
  printBattery();

  // ── 4. LED effects ────────────────────────────────────────────────
  updateLeds();

  // ── 5. Wing motion ───────────────────────────────────────────────
  if (flappingEnabled && !failsafeActive) {
    if (millis() - lastStepTime >= (unsigned long)servoDelayMs) {
      lastStepTime = millis();

      // Sinusoidal position from cosine wave
      // cos sweeps 1 -> -1 -> 1 over one full cycle
      float cosVal = cos(flapPhase);

      // Left wing:  center + trim + pitch, modulated by (amp - diff)
      // Right wing: center - trim - pitch, modulated by -(amp + diff)
      // Mirrors the Ctorque formula — servos flap in opposition,
      // differential makes one wing sweep more than the other
      float leftAngle  = (SERVO_CENTER + leftTrim + pitchOffset)
                         + (amplitude - differential) * cosVal;
      float rightAngle = (SERVO_CENTER - rightTrim - pitchOffset)
                         - (amplitude + differential) * cosVal;

      int leftPos  = constrain((int)leftAngle,  SERVO_LIMIT_MIN, SERVO_LIMIT_MAX);
      int rightPos = constrain((int)rightAngle, SERVO_LIMIT_MIN, SERVO_LIMIT_MAX);

      servoLeft.write(leftPos);
      servoRight.write(rightPos);

      // Advance phase
      flapPhase += PHASE_INCREMENT;
      if (flapPhase >= TWO_PI) flapPhase -= TWO_PI;
    }
  } else {
    // Not flapping — hold wings at center with trims applied
    int leftRest  = constrain((int)(SERVO_CENTER + leftTrim),
                              SERVO_LIMIT_MIN, SERVO_LIMIT_MAX);
    int rightRest = constrain((int)(SERVO_CENTER - rightTrim),
                              SERVO_LIMIT_MIN, SERVO_LIMIT_MAX);
    servoLeft.write(leftRest);
    servoRight.write(rightRest);

    // Reset phase so flapping always starts from the same position
    flapPhase = 0.0;
    lastStepTime = millis();
  }
}
