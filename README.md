# RC Butterfly Flight Controller

A flight controller for an RC butterfly that uses an ESP32 to control two wing servos based on FrSky S.bus input from a 4XR receiver.

## Features

- **Throttle Control**: Controls the flap rate (speed) of the wings
- **Aileron Control**: Controls turning by reducing flap amplitude on one wing
- **Flap Enable Switch**: On/off control for flapping
- **Opposite Wing Motion**: Wings flap in opposite directions for realistic butterfly flight
- **Failsafe Protection**: Automatically returns to neutral position if signal is lost
- **No Inverter Needed**: ESP32 handles inverted S.bus signals in hardware

## Hardware Requirements

### Required Components
- **Waveshare ESP32-C3 Mini Development Board**
- **FrSky 4XR 3/16 channel receiver** with S.bus output
- **Two standard servos** (9g or similar) for wing control
- Power supply for servos (if not powered through ESP32-C3)

### Optional Components
- USB cable for programming and debugging
- Breadboard and jumper wires for prototyping

## Wiring

```
FrSky 4XR Receiver
    |
    | S.bus OUT (direct connection, no inverter!)
    |
ESP32-C3 Mini GPIO 4 (UART1 RX)
    |
    | Control signals
    |
Servo 1 (Left Wing)  -> GPIO 5
Servo 2 (Right Wing) -> GPIO 6
```

### Pin Connections (Waveshare ESP32-C3 Mini)

| Component | ESP32-C3 Pin | Notes |
|-----------|--------------|-------|
| S.bus Signal | GPIO 4 | UART1 RX |
| Left Wing Servo | GPIO 5 | PWM pin |
| Right Wing Servo | GPIO 6 | PWM pin |

**Note**: Pins are configurable. You can change `SBUS_RX_PIN`, `SERVO_LEFT_PIN`, and `SERVO_RIGHT_PIN` in the code to use different GPIO pins if needed.

## Software Setup

### 1. Install Required Libraries

1. **FrSky S.bus Library**: 
   - Copy the `frsky-sbus-main/src` folder to your Arduino `libraries` folder
   - Rename it to `FrskySbus`
   - Location: `~/Documents/Arduino/libraries/FrskySbus/`

2. **ESP32Servo Library**:
   - Install via Arduino Library Manager: Sketch → Include Library → Manage Libraries → Search "ESP32Servo"
   - Or via command line: `arduino-cli lib install "ESP32Servo"`

### 2. Install ESP32 Board Support

1. In Arduino IDE: File → Preferences → Additional Board Manager URLs
2. Add: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
3. Tools → Board → Boards Manager → Search "ESP32" → Install "esp32 by Espressif Systems"

### 3. Upload the Code

1. Open `ButterflyFlightController_ESP32.ino` in Arduino IDE
2. Select your board: Tools → Board → ESP32 Arduino → ESP32C3 Dev Module
3. Select the correct port: Tools → Port → [Your ESP32-C3 port]
4. Upload the sketch

## How It Works

### Flap Rate Control (Throttle)
- **Low Throttle**: Slow, gentle flapping (500ms period)
- **High Throttle**: Fast, aggressive flapping (50ms period)
- The throttle stick position directly controls the speed of wing movement

### Turning Control (Aileron)
- **Neutral**: Both wings flap with full amplitude (45° range: 45° to 135°)
- **Left Stick**: Left wing amplitude reduces (e.g., 68° to 112°), right wing keeps full amplitude → turns left
- **Right Stick**: Right wing amplitude reduces, left wing keeps full amplitude → turns right
- The differential amplitude creates asymmetric lift for turning

### Flap Enable (Channel 3)
- **ON** (above center): Flapping enabled, responds to throttle and aileron
- **OFF** (below center): Flapping disabled, wings return to center (90°), ignores throttle and aileron

### Wing Motion
- Wings flap in **opposite directions** (left up = right down, and vice versa)
- This creates realistic butterfly flapping motion
- Both wings always flap around center (90°), but with different amplitudes when turning

### Failsafe Protection
- If no S.bus signal is received for 500ms, failsafe activates
- Wings automatically return to center position
- Flapping stops
- Normal operation resumes when signal is restored

## Calibration and Tuning

### Adjusting Flap Speed Range
Modify these constants in the code:
```cpp
const unsigned long MIN_FLAP_PERIOD = 50;   // Fastest flapping (ms)
const unsigned long MAX_FLAP_PERIOD = 500;  // Slowest flapping (ms)
```

### Adjusting Flap Amplitude
Change the wing movement range:
```cpp
const int FLAP_AMPLITUDE_FULL = 45;  // Full amplitude when straight (degrees)
const int FLAP_AMPLITUDE_MIN = 22;  // Minimum amplitude when turning (degrees)
```

### Adjusting Turn Sensitivity
Modify the maximum turn reduction:
```cpp
const int MAX_TURN_REDUCTION = 23;  // Maximum amplitude reduction for turning (degrees)
```

### Adjusting Servo Range
If your servos need different limits:
```cpp
const int SERVO_MIN = 45;   // Minimum servo position (degrees)
const int SERVO_MAX = 135;  // Maximum servo position (degrees)
const int SERVO_CENTER = 90; // Center position (degrees)
```

### Channel Mapping
If your transmitter channels are different, modify:
```cpp
#define THROTTLE_CHANNEL 0  // Change to your throttle channel (0-15)
#define AILERON_CHANNEL 1   // Change to your aileron channel (0-15)
#define FLAP_ENABLE_CHANNEL 2  // Change to your enable channel (0-15)
```

## Transmitter Setup

Configure your transmitter with:
- **Channel 1**: Throttle stick (controls flap rate)
- **Channel 2**: Aileron stick (controls turning)
- **Channel 3**: 2-position or 3-position switch (enables/disables flapping)

## Troubleshooting

### No Wing Movement
1. Check that servos are connected to GPIO 18 and 19
2. Verify servo power supply (servos may need external power)
3. Check Serial Monitor for error messages
4. Verify S.bus signal is being received (check debug output)

### Erratic Movement
1. Ensure S.bus signal is reaching the ESP32
2. Check GPIO 16 connection
3. Verify transmitter is bound to receiver
4. Check that channel mapping matches your transmitter

### Failsafe Always Active
1. Verify S.bus signal is reaching GPIO 16
2. Check receiver is powered and bound
3. Ensure S.bus is enabled on receiver (some receivers need configuration)
4. Check Serial Monitor for connection status

### Wings Not Centered
1. Adjust `SERVO_CENTER` value if needed
2. Physically adjust servo horns if needed
3. Use transmitter trim if available

## Debugging

The code includes Serial output for debugging. Open Serial Monitor at 115200 baud to see:
- Throttle, aileron, and flap enable channel values
- Current flap period
- Wing amplitudes for each side
- Failsafe status

To disable debug output for production, comment out or remove the Serial.print statements in the loop.

## Why ESP32-C3 Mini?

The Waveshare ESP32-C3 Mini is ideal for this project because:
- **Hardware UART**: Dedicated hardware serial (not software emulation)
- **Native Inverted Signal Support**: Handles inverted S.bus signals in hardware - no inverter circuit needed!
- **Reliable at 100000 Baud**: Handles S.bus speed easily without jitter
- **Better Performance**: Smooth, responsive control
- **Compact Size**: Small form factor perfect for RC applications
- **Low Power**: Efficient operation for battery-powered projects

## Safety Notes

- Always test with the butterfly secured before free flight
- Start with low throttle settings to verify control
- Ensure failsafe is working before flight
- Check all connections before powering on
- Use appropriate power supply for servos (may need external supply)
- Verify all controls respond correctly before first flight

## License

This project uses the FrSky S.bus library which is based on work by Brian R Taylor and maintained by COPPERPUNK.

## Credits

- FrSky S.bus Library: [COPPERPUNK](https://github.com/copperpunk-arduino/frsky-sbus)
- Original SBUS.h library: [Brian R Taylor](https://github.com/bolderflight/SBUS)
- ESP32Servo Library: [Kevin Harrington, John K. Bennett](https://github.com/madhephaestus/ESP32Servo)

