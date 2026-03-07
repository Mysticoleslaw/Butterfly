/*
 * RC Butterfly Flight Controller (ESP32-C3 Mini + FrSky RXS-R)
 *
 * Uses F.Port protocol from the uninverted S.Port pad on RXS-R
 *
 * Controls an RC butterfly with two wing servos
 * - Throttle channel controls flap rate (how fast wings flap)
 * - Aileron channel controls turning (differential wing movement)
 * - Flap Enable channel controls on/off (enable/disable flapping)
 *
 * Hardware Requirements:
 * - ESP32-C3 Mini Development Board
 * - FrSky RXS-R receiver with uninverted S.Port output (P pad)
 * - Two servos for wing control
 *
 * Wiring:
 * - F.Port from receiver P pad -> GPIO 7
 * - Servo 1 (Left wing) -> GPIO 4
 * - Servo 2 (Right wing) -> GPIO 5
 */

#include <ESP32Servo.h>

// Hardware configuration
#define FPORT_RX_PIN 7
#define SERVO_LEFT_PIN 4
#define SERVO_RIGHT_PIN 5

// UART for F.Port
HardwareSerial fportSerial(1);

// Servo objects
Servo servoLeft;
Servo servoRight;

// Channel mapping
#define THROTTLE_CHANNEL 0
#define AILERON_CHANNEL 1
#define FLAP_ENABLE_CHANNEL 2

// F.Port constants
#define FPORT_HEADER 0x7E
#define FPORT_RC_FRAME_LENGTH 0x19  // 25 bytes
#define FPORT_RC_FRAME_TYPE 0x00

// Channel value range (same as S.bus)
const uint16_t CHANNEL_MIN = 172;
const uint16_t CHANNEL_MAX = 1811;
const uint16_t CHANNEL_CENTER = 992;

// Flap control parameters
const unsigned long MIN_FLAP_PERIOD = 50;
const unsigned long MAX_FLAP_PERIOD = 500;
const int FLAP_AMPLITUDE_FULL = 45;
const int FLAP_AMPLITUDE_MIN = 22;
const int SERVO_CENTER = 90;
const int SERVO_MIN = 30;
const int SERVO_MAX = 150;
const int MAX_TURN_REDUCTION = 23;

// Wing state
int leftWingAmplitude = FLAP_AMPLITUDE_FULL;
int rightWingAmplitude = FLAP_AMPLITUDE_FULL;
int leftWingPos = SERVO_CENTER;
int rightWingPos = SERVO_CENTER;

// Flap timing
unsigned long lastFlapTime = 0;
unsigned long flapPeriod = 200;
bool flapDirection = true;
bool flappingEnabled = false;

// F.Port parsing
uint16_t channels[16];
uint8_t fportBuffer[32];
int bufferIndex = 0;
bool inFrame = false;
bool failsafeActive = false;
unsigned long lastValidFrameTime = 0;
const unsigned long FAILSAFE_TIMEOUT = 500;

// Parse F.Port RC frame and extract channels
bool parseFportFrame() {
  // Frame structure: 7E [len] [type] [22 bytes channel data] [flags] [crc] 7E
  // Channel data is same encoding as S.bus (11 bits per channel)

  if (fportBuffer[0] != FPORT_HEADER) return false;
  if (fportBuffer[1] != FPORT_RC_FRAME_LENGTH) return false;
  if (fportBuffer[2] != FPORT_RC_FRAME_TYPE) return false;

  // Extract channel data starting at byte 3
  uint8_t* payload = &fportBuffer[3];

  // Decode 16 channels from 22 bytes (11 bits each)
  channels[0]  = (payload[0]       | payload[1]  << 8) & 0x07FF;
  channels[1]  = (payload[1]  >> 3 | payload[2]  << 5) & 0x07FF;
  channels[2]  = (payload[2]  >> 6 | payload[3]  << 2 | payload[4] << 10) & 0x07FF;
  channels[3]  = (payload[4]  >> 1 | payload[5]  << 7) & 0x07FF;
  channels[4]  = (payload[5]  >> 4 | payload[6]  << 4) & 0x07FF;
  channels[5]  = (payload[6]  >> 7 | payload[7]  << 1 | payload[8] << 9) & 0x07FF;
  channels[6]  = (payload[8]  >> 2 | payload[9]  << 6) & 0x07FF;
  channels[7]  = (payload[9]  >> 5 | payload[10] << 3) & 0x07FF;
  channels[8]  = (payload[11]      | payload[12] << 8) & 0x07FF;
  channels[9]  = (payload[12] >> 3 | payload[13] << 5) & 0x07FF;
  channels[10] = (payload[13] >> 6 | payload[14] << 2 | payload[15] << 10) & 0x07FF;
  channels[11] = (payload[15] >> 1 | payload[16] << 7) & 0x07FF;
  channels[12] = (payload[16] >> 4 | payload[17] << 4) & 0x07FF;
  channels[13] = (payload[17] >> 7 | payload[18] << 1 | payload[19] << 9) & 0x07FF;
  channels[14] = (payload[19] >> 2 | payload[20] << 6) & 0x07FF;
  channels[15] = (payload[20] >> 5 | payload[21] << 3) & 0x07FF;

  return true;
}

// Read and parse F.Port data
bool readFport() {
  while (fportSerial.available()) {
    uint8_t b = fportSerial.read();

    if (b == FPORT_HEADER) {
      if (inFrame && bufferIndex > 25) {
        // End of frame - try to parse
        fportBuffer[bufferIndex] = b;
        inFrame = false;
        if (parseFportFrame()) {
          bufferIndex = 0;
          return true;
        }
      }
      // Start of new frame
      bufferIndex = 0;
      fportBuffer[bufferIndex++] = b;
      inFrame = true;
    } else if (inFrame) {
      if (bufferIndex < 32) {
        fportBuffer[bufferIndex++] = b;
      } else {
        // Buffer overflow, reset
        inFrame = false;
        bufferIndex = 0;
      }
    }
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    ;
  }
  Serial.println("RC Butterfly Flight Controller - F.Port");
  Serial.println("Initializing...");

  // F.Port: 115200 baud, 8N1, no inversion
  fportSerial.begin(115200, SERIAL_8N1, FPORT_RX_PIN, -1, false);

  while (fportSerial.available()) {
    fportSerial.read();
  }
  Serial.println("F.Port initialized on GPIO 7");

  // Attach servos
  servoLeft.attach(SERVO_LEFT_PIN);
  servoRight.attach(SERVO_RIGHT_PIN);
  Serial.print("Servos attached - Left: GPIO ");
  Serial.print(SERVO_LEFT_PIN);
  Serial.print(" | Right: GPIO ");
  Serial.println(SERVO_RIGHT_PIN);

  // Center servos
  servoLeft.write(SERVO_CENTER);
  servoRight.write(SERVO_CENTER);
  Serial.println("Servos centered at 90 degrees");

  lastValidFrameTime = millis();
  Serial.println("Ready! Waiting for F.Port signal...");
}

void loop() {
  // Check for F.Port data
  if (readFport()) {
    lastValidFrameTime = millis();
    failsafeActive = false;

    // Read channel values
    uint16_t throttle = channels[THROTTLE_CHANNEL];
    uint16_t aileron = channels[AILERON_CHANNEL];
    uint16_t flapEnable = channels[FLAP_ENABLE_CHANNEL];

    // Check if flapping is enabled (channel 3 above center)
    flappingEnabled = (flapEnable > CHANNEL_CENTER);

    if (flappingEnabled) {
      // Map throttle to flap rate
      flapPeriod = map(throttle, CHANNEL_MIN, CHANNEL_MAX, MAX_FLAP_PERIOD, MIN_FLAP_PERIOD);
      flapPeriod = constrain(flapPeriod, MIN_FLAP_PERIOD, MAX_FLAP_PERIOD);

      // Map aileron to turn adjustment
      int aileronValue = map(aileron, CHANNEL_MIN, CHANNEL_MAX, -100, 100);
      int amplitudeChange = map(abs(aileronValue), 0, 100, 0, MAX_TURN_REDUCTION);

      if (aileronValue < 0) {
        leftWingAmplitude = FLAP_AMPLITUDE_FULL - amplitudeChange;
        rightWingAmplitude = FLAP_AMPLITUDE_FULL + amplitudeChange;
      } else if (aileronValue > 0) {
        rightWingAmplitude = FLAP_AMPLITUDE_FULL - amplitudeChange;
        leftWingAmplitude = FLAP_AMPLITUDE_FULL + amplitudeChange;
      } else {
        leftWingAmplitude = FLAP_AMPLITUDE_FULL;
        rightWingAmplitude = FLAP_AMPLITUDE_FULL;
      }

      int maxAmp = FLAP_AMPLITUDE_FULL + MAX_TURN_REDUCTION;
      leftWingAmplitude = constrain(leftWingAmplitude, FLAP_AMPLITUDE_MIN, maxAmp);
      rightWingAmplitude = constrain(rightWingAmplitude, FLAP_AMPLITUDE_MIN, maxAmp);
    }
  }

  // Check for failsafe
  if (millis() - lastValidFrameTime > FAILSAFE_TIMEOUT) {
    if (!failsafeActive) {
      failsafeActive = true;
      leftWingAmplitude = FLAP_AMPLITUDE_FULL;
      rightWingAmplitude = FLAP_AMPLITUDE_FULL;
      flapPeriod = MAX_FLAP_PERIOD;
    }

    static unsigned long lastFailsafeMsg = 0;
    if (millis() - lastFailsafeMsg > 2000) {
      Serial.print("FAILSAFE - No F.Port signal | Last: ");
      Serial.print(millis() - lastValidFrameTime);
      Serial.println(" ms ago");
      lastFailsafeMsg = millis();
    }
  }

  // Flap the wings
  if (flappingEnabled && !failsafeActive) {
    if (millis() - lastFlapTime >= flapPeriod) {
      lastFlapTime = millis();
      flapDirection = !flapDirection;

      if (flapDirection) {
        leftWingPos = SERVO_CENTER + leftWingAmplitude;
        rightWingPos = SERVO_CENTER - rightWingAmplitude;
      } else {
        leftWingPos = SERVO_CENTER - leftWingAmplitude;
        rightWingPos = SERVO_CENTER + rightWingAmplitude;
      }

      leftWingPos = constrain(leftWingPos, SERVO_MIN, SERVO_MAX);
      rightWingPos = constrain(rightWingPos, SERVO_MIN, SERVO_MAX);

      servoLeft.write(leftWingPos);
      servoRight.write(rightWingPos);
    }
  } else {
    servoLeft.write(SERVO_CENTER);
    servoRight.write(SERVO_CENTER);
    lastFlapTime = millis();
  }
}
