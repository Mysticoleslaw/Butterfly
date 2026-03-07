/*
 * RC Butterfly Flight Controller (ESP32 Version - No Inverter Needed!)
 * 
 * This version uses ESP32's hardware UART inversion capability
 * to handle inverted S.bus signals without a hardware inverter!
 * 
 * Controls an RC butterfly with two wing servos using FrSky S.bus input
 * - Throttle channel controls flap rate (how fast wings flap)
 * - Aileron channel controls turning (differential wing movement)
 * - Flap Enable channel controls on/off (enable/disable flapping)
 * 
 * Hardware Requirements:
 * - ESP32 board
 * - FrSky 4XR receiver with S.bus output
 * - NO INVERTER NEEDED! ESP32 handles inversion in hardware
 * - Two servos for wing control
 * 
 * Wiring:
 * - S.bus output from receiver (DIRECT, no inverter!) -> GPIO 16 (UART2 RX)
 * - Servo 1 (Left wing) -> GPIO 18
 * - Servo 2 (Right wing) -> GPIO 19
 * 
 * Channel Mapping (default):
 * - Channel 1: Throttle (flap rate)
 * - Channel 2: Aileron (turning)
 * - Channel 3: Flap Enable (on/off switch)
 * 
 * NOTE: ESP32 has hardware support for inverted UART signals,
 * eliminating the need for a hardware inverter!
 */

#include "FrskySbus.h"
#include <ESP32Servo.h>

// Hardware configuration
#define SBUS_RX_PIN 16   // GPIO 16 for S.bus RX (UART2 on ESP32)
#define SERVO_LEFT_PIN 18
#define SERVO_RIGHT_PIN 19

// ESP32 HardwareSerial with inverted signal support
HardwareSerial sbusSerial(2); // Use UART2 on ESP32
FrskySbus frsky_(sbusSerial);

// Servo objects
Servo servoLeft;
Servo servoRight;

// Channel mapping (adjust based on your transmitter setup)
#define THROTTLE_CHANNEL 0  // Channel 1 (0-indexed)
#define AILERON_CHANNEL 1   // Channel 2 (0-indexed)
#define FLAP_ENABLE_CHANNEL 2  // Channel 3 (0-indexed) - on/off switch

// Control parameters
const uint16_t SBUS_MIN = 172;   // Minimum S.bus value
const uint16_t SBUS_MAX = 1811;  // Maximum S.bus value
const uint16_t SBUS_CENTER = 992; // Center S.bus value

// Flap control parameters
const unsigned long MIN_FLAP_PERIOD = 50;   // Minimum time between flaps (ms) - faster flapping
const unsigned long MAX_FLAP_PERIOD = 500;  // Maximum time between flaps (ms) - slower flapping
const int FLAP_AMPLITUDE_FULL = 45;        // Full flap amplitude (degrees) - both wings when straight
const int FLAP_AMPLITUDE_MIN = 22;         // Minimum flap amplitude when turning (degrees)
const int SERVO_CENTER = 90;               // Center position for servos (degrees)
const int SERVO_MIN = 30;                  // Minimum servo position (degrees) - increased range for turning
const int SERVO_MAX = 150;                 // Maximum servo position (degrees) - increased range for turning

// Turn control parameters
const int MAX_TURN_REDUCTION = 23;  // Maximum reduction in flap amplitude for turning (degrees)

// Wing state
int leftWingAmplitude = FLAP_AMPLITUDE_FULL;   // Current flap amplitude for left wing
int rightWingAmplitude = FLAP_AMPLITUDE_FULL;  // Current flap amplitude for right wing
int leftWingPos = SERVO_CENTER;
int rightWingPos = SERVO_CENTER;

// Flap timing
unsigned long lastFlapTime = 0;
unsigned long flapPeriod = 200;  // Default flap period (ms)
bool flapDirection = true;       // true = up, false = down
bool flappingEnabled = false;   // Enable/disable flapping based on channel 3

// Safety
bool failsafeActive = false;
bool lostFrame = false;
unsigned long lastValidFrameTime = 0;
const unsigned long FAILSAFE_TIMEOUT = 500; // ms without valid frame = failsafe

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    ; // Wait for serial port or timeout after 3 seconds
  }
  Serial.println("RC Butterfly Flight Controller (ESP32 - No Inverter!)");
  Serial.println("Initializing...");

  // Initialize ESP32 UART with inverted signal support
  // ESP32 can invert the signal in hardware - no external inverter needed!
  sbusSerial.begin(100000, SERIAL_8E2, SBUS_RX_PIN, -1, true); // baud, config, RX, TX, invert
  // Note: The 'invert' parameter (true) tells ESP32 to invert the signal in hardware
  
  // Clear buffer
  while (sbusSerial.available() > 0) {
    sbusSerial.read();
  }
  Serial.println("S.bus initialized (ESP32 hardware inversion)");
  Serial.print("S.bus RX pin: GPIO ");
  Serial.println(SBUS_RX_PIN);

  // Attach servos
  servoLeft.attach(SERVO_LEFT_PIN);
  servoRight.attach(SERVO_RIGHT_PIN);
  
  // Set servos to center position
  servoLeft.write(SERVO_CENTER);
  servoRight.write(SERVO_CENTER);
  Serial.print("Servos attached - Left: GPIO ");
  Serial.print(SERVO_LEFT_PIN);
  Serial.print(" | Right: GPIO ");
  Serial.println(SERVO_RIGHT_PIN);
  Serial.println("Servos initialized and centered at 90 degrees");

  // Initialize timing
  lastFlapTime = millis();
  lastValidFrameTime = millis();

  Serial.println("Ready!");
  Serial.println("Waiting for S.bus signal...");
}

void loop() {
  // Check for new S.bus message
  if (frsky_.checkForNewMessage()) {
    lastValidFrameTime = millis();
    failsafeActive = frsky_.isFailsafeActive();
    lostFrame = frsky_.lostFrame();

    if (!failsafeActive && !lostFrame) {
      // Read channel values
      uint16_t throttle = frsky_.getChannelValue(THROTTLE_CHANNEL);
      uint16_t aileron = frsky_.getChannelValue(AILERON_CHANNEL);
      uint16_t flapEnable = frsky_.getChannelValue(FLAP_ENABLE_CHANNEL);

      // Check if flapping is enabled (channel 3 above center = enabled)
      flappingEnabled = (flapEnable > SBUS_CENTER);

      // Only process throttle and aileron inputs if flapping is enabled
      if (flappingEnabled) {
        // Map throttle to flap rate
        // Higher throttle = faster flapping (shorter period)
        flapPeriod = map(throttle, SBUS_MIN, SBUS_MAX, MAX_FLAP_PERIOD, MIN_FLAP_PERIOD);
        flapPeriod = constrain(flapPeriod, MIN_FLAP_PERIOD, MAX_FLAP_PERIOD);

        // Map aileron to turn adjustment
        // Negative aileron (left) = reduce left wing, increase right wing by same amount
        // Positive aileron (right) = reduce right wing, increase left wing by same amount
        int aileronValue = map(aileron, SBUS_MIN, SBUS_MAX, -100, 100);  // -100 to +100
        
        // Calculate amplitude changes for each wing
        // When turning, decrease amplitude on one side and increase on the other by the SAME amount
        // This creates balanced asymmetric lift for better turning
        int amplitudeChange = map(abs(aileronValue), 0, 100, 0, MAX_TURN_REDUCTION);
        
        if (aileronValue < 0) {
          // Turning left - reduce left wing amplitude, increase right wing amplitude by same amount
          leftWingAmplitude = FLAP_AMPLITUDE_FULL - amplitudeChange;
          rightWingAmplitude = FLAP_AMPLITUDE_FULL + amplitudeChange;  // Increase by same amount
        } else if (aileronValue > 0) {
          // Turning right - reduce right wing amplitude, increase left wing amplitude by same amount
          rightWingAmplitude = FLAP_AMPLITUDE_FULL - amplitudeChange;
          leftWingAmplitude = FLAP_AMPLITUDE_FULL + amplitudeChange;  // Increase by same amount
        } else {
          // Flying straight - both wings full amplitude
          leftWingAmplitude = FLAP_AMPLITUDE_FULL;
          rightWingAmplitude = FLAP_AMPLITUDE_FULL;
        }
        
        // Constrain amplitudes to safe range
        // Allow increased amplitude up to full + MAX_TURN_REDUCTION for better turning
        int maxAmplitude = FLAP_AMPLITUDE_FULL + MAX_TURN_REDUCTION;
        leftWingAmplitude = constrain(leftWingAmplitude, FLAP_AMPLITUDE_MIN, maxAmplitude);
        rightWingAmplitude = constrain(rightWingAmplitude, FLAP_AMPLITUDE_MIN, maxAmplitude);
      } else {
        // Flapping disabled - ignore all inputs and return to neutral
        leftWingAmplitude = FLAP_AMPLITUDE_FULL;
        rightWingAmplitude = FLAP_AMPLITUDE_FULL;
        flapPeriod = MAX_FLAP_PERIOD;
      }

      // Debug output (can be disabled for production)
      static unsigned long lastDebugTime = 0;
      if (millis() - lastDebugTime > 100) {  // Print every 100ms
        Serial.print("Throttle: ");
        Serial.print(throttle);
        Serial.print(" | Aileron: ");
        Serial.print(aileron);
        Serial.print(" | Flap Enable: ");
        Serial.print(flapEnable);
        Serial.print(" | Flapping: ");
        Serial.print(flappingEnabled ? "ON" : "OFF");
        Serial.print(" | Flap Period: ");
        Serial.print(flapPeriod);
        Serial.print(" | Left Amp: ");
        Serial.print(leftWingAmplitude);
        Serial.print(" | Right Amp: ");
        Serial.println(rightWingAmplitude);
        lastDebugTime = millis();
      }
    }
  }

  // Check for failsafe timeout
  if (millis() - lastValidFrameTime > FAILSAFE_TIMEOUT) {
    failsafeActive = true;
  }

  // Handle failsafe - return wings to neutral position
  if (failsafeActive) {
    leftWingAmplitude = FLAP_AMPLITUDE_FULL;
    rightWingAmplitude = FLAP_AMPLITUDE_FULL;
    flapPeriod = MAX_FLAP_PERIOD;  // Slow down flapping
    Serial.println("FAILSAFE ACTIVE - Returning to neutral");
  }

  // Flap the wings (only if enabled)
  unsigned long currentTime = millis();
  
  if (flappingEnabled) {
    if (currentTime - lastFlapTime >= flapPeriod) {
      lastFlapTime = currentTime;
      
      // Toggle flap direction
      flapDirection = !flapDirection;
      
      // Calculate wing positions
      // Both wings flap around center (90 degrees)
      // Left wing: normal motion (up when flapDirection is true)
      // Right wing: INVERTED motion (down when flapDirection is true) for opposite flapping
      // Turning reduces amplitude on one side, creating asymmetric lift
      if (flapDirection) {
        // Left wing up, right wing down (opposite motion)
        leftWingPos = SERVO_CENTER + leftWingAmplitude;
        rightWingPos = SERVO_CENTER - rightWingAmplitude;
      } else {
        // Left wing down, right wing up (opposite motion)
        leftWingPos = SERVO_CENTER - leftWingAmplitude;
        rightWingPos = SERVO_CENTER + rightWingAmplitude;
      }
      
      // Constrain to safe servo range
      leftWingPos = constrain(leftWingPos, SERVO_MIN, SERVO_MAX);
      rightWingPos = constrain(rightWingPos, SERVO_MIN, SERVO_MAX);
      
      // Update servos
      servoLeft.write(leftWingPos);
      servoRight.write(rightWingPos);
    }
  } else {
    // Flapping disabled - return wings to center position
    servoLeft.write(SERVO_CENTER);
    servoRight.write(SERVO_CENTER);
    lastFlapTime = currentTime; // Reset timer so it starts immediately when enabled
  }
}

