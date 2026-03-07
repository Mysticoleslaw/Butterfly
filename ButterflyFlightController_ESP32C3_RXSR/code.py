"""
RC Butterfly Flight Controller (ESP32-C3 Mini + FrSky RXS-R) - CircuitPython Version

This version uses CircuitPython to control an RC butterfly with two wing servos
using FrSky S.bus input from an RXS-R receiver.

Controls:
- Throttle channel controls flap rate (how fast wings flap)
- Aileron channel controls turning (differential wing movement)
- Flap Enable channel controls on/off (enable/disable flapping)

Hardware Requirements:
- ESP32-C3 Mini Development Board
- FrSky RXS-R receiver with S.bus output
- NO INVERTER NEEDED! ESP32-C3 handles inversion in hardware
- Two servos for wing control

Wiring:
- S.bus output from receiver (DIRECT, no inverter!) -> GPIO 6 (UART RX)
- Servo 1 (Left wing) -> GPIO 4
- Servo 2 (Right wing) -> GPIO 5

Channel Mapping (default):
- Channel 1: Throttle (flap rate)
- Channel 2: Aileron (turning)
- Channel 3: Flap Enable (on/off switch)

NOTE: ESP32-C3 has hardware support for inverted UART signals,
eliminating the need for a hardware inverter!
"""

import board
import busio
import time
from adafruit_motor import servo
from pwmio import PWMOut
import supervisor

# Hardware configuration (ESP32-C3 Mini)
SBUS_RX_PIN = board.GP6   # GPIO 6 for S.bus RX (UART RX)
SERVO_LEFT_PIN = board.GP4
SERVO_RIGHT_PIN = board.GP5

# Initialize UART for S.bus (100000 baud, 8E2)
# IMPORTANT: CircuitPython UART doesn't support hardware inversion natively
# You may need:
# 1. A hardware inverter circuit, OR
# 2. Check if your ESP32-C3 board firmware supports inversion via board-specific methods
# For ESP32-C3, you might need to use machine.UART if available, or invert in software
uart = busio.UART(SBUS_RX_PIN, None, baudrate=100000, bits=8, parity=busio.UART.Parity.EVEN, stop=2)

# Initialize servos
pwm_left = PWMOut(SERVO_LEFT_PIN, frequency=50)
pwm_right = PWMOut(SERVO_RIGHT_PIN, frequency=50)
servo_left = servo.Servo(pwm_left, min_pulse=500, max_pulse=2500)
servo_right = servo.Servo(pwm_right, min_pulse=500, max_pulse=2500)

# Channel mapping (adjust based on your transmitter setup)
THROTTLE_CHANNEL = 0  # Channel 1 (0-indexed)
AILERON_CHANNEL = 1   # Channel 2 (0-indexed)
FLAP_ENABLE_CHANNEL = 2  # Channel 3 (0-indexed) - on/off switch

# Control parameters
SBUS_MIN = 172   # Minimum S.bus value
SBUS_MAX = 1811  # Maximum S.bus value
SBUS_CENTER = 992  # Center S.bus value

# Flap control parameters
MIN_FLAP_PERIOD = 0.05   # Minimum time between flaps (seconds) - faster flapping
MAX_FLAP_PERIOD = 0.5    # Maximum time between flaps (seconds) - slower flapping
FLAP_AMPLITUDE_FULL = 45  # Full flap amplitude (degrees) - both wings when straight
FLAP_AMPLITUDE_MIN = 22   # Minimum flap amplitude when turning (degrees)
SERVO_CENTER = 90        # Center position for servos (degrees)
SERVO_MIN = 30           # Minimum servo position (degrees) - increased range for turning
SERVO_MAX = 150          # Maximum servo position (degrees) - increased range for turning

# Turn control parameters
MAX_TURN_REDUCTION = 23  # Maximum reduction in flap amplitude for turning (degrees)

# Wing state
left_wing_amplitude = FLAP_AMPLITUDE_FULL
right_wing_amplitude = FLAP_AMPLITUDE_FULL
left_wing_pos = SERVO_CENTER
right_wing_pos = SERVO_CENTER

# Flap timing
last_flap_time = 0
flap_period = 0.2  # Default flap period (seconds)
flap_direction = True  # True = up, False = down
flapping_enabled = False  # Enable/disable flapping based on channel 3

# Safety
failsafe_active = False
lost_frame = False
last_valid_frame_time = 0
FAILSAFE_TIMEOUT = 0.5  # seconds without valid frame = failsafe

# S.bus frame parsing
SBUS_HEADER = 0x0F
SBUS_FOOTER = 0x00
SBUS_LOST_FRAME = 0x04
SBUS_FAILSAFE = 0x08
SBUS_FRAME_SIZE = 25
SBUS_PAYLOAD_SIZE = 24

channels = [0] * 16
sbus_buffer = bytearray(SBUS_FRAME_SIZE)
parser_state = 0
prev_byte = SBUS_FOOTER
valid_frame_count = 0
valid_frame_count_min = 3


def parse_sbus():
    """Parse S.bus data from UART"""
    global parser_state, prev_byte, valid_frame_count, channels, failsafe_active, lost_frame
    
    if uart.in_waiting == 0:
        return False
    
    while uart.in_waiting > 0:
        cur_byte = uart.read(1)
        if cur_byte is None:
            break
        cur_byte = cur_byte[0]
        
        # Find the header
        if parser_state == 0:
            if cur_byte == SBUS_HEADER and prev_byte == SBUS_FOOTER:
                parser_state = 1
                sbus_buffer[0] = cur_byte
        else:
            # Collect payload
            if parser_state < SBUS_FRAME_SIZE:
                sbus_buffer[parser_state] = cur_byte
                parser_state += 1
            
            # Check for footer
            if parser_state == SBUS_FRAME_SIZE:
                if cur_byte == SBUS_FOOTER:
                    parser_state = 0
                    valid_frame_count += 1
                    
                    # Only process if we have enough valid frames
                    if valid_frame_count > valid_frame_count_min:
                        valid_frame_count -= 1
                        # Parse channels
                        parse_channels()
                        return True
                else:
                    parser_state = 0
                    valid_frame_count = 0
        
        prev_byte = cur_byte
    
    return False


def parse_channels():
    """Extract channel values from S.bus payload"""
    global channels, failsafe_active, lost_frame
    
    payload = sbus_buffer[1:25]  # Skip header, get 24 bytes of payload
    
    # Decode 16 channels of 11-bit data
    channels[0] = (payload[0] | payload[1] << 8) & 0x07FF
    channels[1] = (payload[1] >> 3 | payload[2] << 5) & 0x07FF
    channels[2] = (payload[2] >> 6 | payload[3] << 2 | payload[4] << 10) & 0x07FF
    channels[3] = (payload[4] >> 1 | payload[5] << 7) & 0x07FF
    channels[4] = (payload[5] >> 4 | payload[6] << 4) & 0x07FF
    channels[5] = (payload[6] >> 7 | payload[7] << 1 | payload[8] << 9) & 0x07FF
    channels[6] = (payload[8] >> 2 | payload[9] << 6) & 0x07FF
    channels[7] = (payload[9] >> 5 | payload[10] << 3) & 0x07FF
    channels[8] = (payload[11] | payload[12] << 8) & 0x07FF
    channels[9] = (payload[12] >> 3 | payload[13] << 5) & 0x07FF
    channels[10] = (payload[13] >> 6 | payload[14] << 2 | payload[15] << 10) & 0x07FF
    channels[11] = (payload[15] >> 1 | payload[16] << 7) & 0x07FF
    channels[12] = (payload[16] >> 4 | payload[17] << 4) & 0x07FF
    channels[13] = (payload[17] >> 7 | payload[18] << 1 | payload[19] << 9) & 0x07FF
    channels[14] = (payload[19] >> 2 | payload[20] << 6) & 0x07FF
    channels[15] = (payload[20] >> 5 | payload[21] << 3) & 0x07FF
    
    # Check flags in byte 23 (index 22 in payload, but we have header at 0, so payload[22])
    flags = payload[22]
    lost_frame = bool(flags & SBUS_LOST_FRAME)
    failsafe_active = bool(flags & SBUS_FAILSAFE)


def constrain(value, min_val, max_val):
    """Constrain a value between min and max"""
    return max(min_val, min(value, max_val))


def map_value(value, in_min, in_max, out_min, out_max):
    """Map a value from one range to another"""
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


# Initialize servos to center position
servo_left.angle = SERVO_CENTER
servo_right.angle = SERVO_CENTER
last_valid_frame_time = time.monotonic()

print("RC Butterfly Flight Controller (ESP32-C3 Mini + FrSky RXS-R) - CircuitPython")
print("Initializing...")
print(f"S.bus RX pin: GPIO 6")
print(f"Servos attached - Left: GPIO 4 | Right: GPIO 5")
print("Servos initialized and centered at 90 degrees")
print("Ready!")
print("Waiting for S.bus signal from FrSky RXS-R...")

# Main loop
while True:
    current_time = time.monotonic()
    
    # Check for new S.bus message
    if parse_sbus():
        last_valid_frame_time = current_time
        
        if not failsafe_active and not lost_frame:
            # Read channel values
            throttle = channels[THROTTLE_CHANNEL]
            aileron = channels[AILERON_CHANNEL]
            flap_enable = channels[FLAP_ENABLE_CHANNEL]
            
            # Check if flapping is enabled (channel 3 above center = enabled)
            flapping_enabled = (flap_enable > SBUS_CENTER)
            
            # Only process throttle and aileron inputs if flapping is enabled
            if flapping_enabled:
                # Map throttle to flap rate
                # Higher throttle = faster flapping (shorter period)
                flap_period = map_value(throttle, SBUS_MIN, SBUS_MAX, MAX_FLAP_PERIOD, MIN_FLAP_PERIOD)
                flap_period = constrain(flap_period, MIN_FLAP_PERIOD, MAX_FLAP_PERIOD)
                
                # Map aileron to turn adjustment
                # Negative aileron (left) = reduce left wing, increase right wing by same amount
                # Positive aileron (right) = reduce right wing, increase left wing by same amount
                aileron_value = map_value(aileron, SBUS_MIN, SBUS_MAX, -100, 100)
                
                # Calculate amplitude changes for each wing
                # When turning, decrease amplitude on one side and increase on the other by the SAME amount
                amplitude_change = map_value(abs(aileron_value), 0, 100, 0, MAX_TURN_REDUCTION)
                
                if aileron_value < 0:
                    # Turning left - reduce left wing amplitude, increase right wing amplitude by same amount
                    left_wing_amplitude = FLAP_AMPLITUDE_FULL - amplitude_change
                    right_wing_amplitude = FLAP_AMPLITUDE_FULL + amplitude_change
                elif aileron_value > 0:
                    # Turning right - reduce right wing amplitude, increase left wing amplitude by same amount
                    right_wing_amplitude = FLAP_AMPLITUDE_FULL - amplitude_change
                    left_wing_amplitude = FLAP_AMPLITUDE_FULL + amplitude_change
                else:
                    # Flying straight - both wings full amplitude
                    left_wing_amplitude = FLAP_AMPLITUDE_FULL
                    right_wing_amplitude = FLAP_AMPLITUDE_FULL
                
                # Constrain amplitudes to safe range
                max_amplitude = FLAP_AMPLITUDE_FULL + MAX_TURN_REDUCTION
                left_wing_amplitude = constrain(left_wing_amplitude, FLAP_AMPLITUDE_MIN, max_amplitude)
                right_wing_amplitude = constrain(right_wing_amplitude, FLAP_AMPLITUDE_MIN, max_amplitude)
            else:
                # Flapping disabled - ignore all inputs and return to neutral
                left_wing_amplitude = FLAP_AMPLITUDE_FULL
                right_wing_amplitude = FLAP_AMPLITUDE_FULL
                flap_period = MAX_FLAP_PERIOD
            
            # Debug output (can be disabled for production)
            # Uncomment to see channel values
            # print(f"Throttle: {throttle} | Aileron: {aileron} | Flap Enable: {flap_enable} | "
            #       f"Flapping: {'ON' if flapping_enabled else 'OFF'} | "
            #       f"Flap Period: {flap_period:.3f} | Left Amp: {left_wing_amplitude} | Right Amp: {right_wing_amplitude}")
    
    # Check for failsafe timeout
    if current_time - last_valid_frame_time > FAILSAFE_TIMEOUT:
        failsafe_active = True
    
    # Handle failsafe - return wings to neutral position
    if failsafe_active:
        left_wing_amplitude = FLAP_AMPLITUDE_FULL
        right_wing_amplitude = FLAP_AMPLITUDE_FULL
        flap_period = MAX_FLAP_PERIOD
        # Print failsafe message occasionally
        if int(current_time * 2) % 2 == 0:  # Every 2 seconds
            print(f"FAILSAFE ACTIVE - No S.bus signal detected on GPIO 6 | "
                  f"Last valid frame: {current_time - last_valid_frame_time:.1f} s ago")
    
    # Flap the wings (only if enabled)
    if flapping_enabled:
        if current_time - last_flap_time >= flap_period:
            last_flap_time = current_time
            
            # Toggle flap direction
            flap_direction = not flap_direction
            
            # Calculate wing positions
            # Both wings flap around center (90 degrees)
            # Left wing: normal motion (up when flap_direction is True)
            # Right wing: INVERTED motion (down when flap_direction is True) for opposite flapping
            if flap_direction:
                # Left wing up, right wing down (opposite motion)
                left_wing_pos = SERVO_CENTER + left_wing_amplitude
                right_wing_pos = SERVO_CENTER - right_wing_amplitude
            else:
                # Left wing down, right wing up (opposite motion)
                left_wing_pos = SERVO_CENTER - left_wing_amplitude
                right_wing_pos = SERVO_CENTER + right_wing_amplitude
            
            # Constrain to safe servo range
            left_wing_pos = constrain(left_wing_pos, SERVO_MIN, SERVO_MAX)
            right_wing_pos = constrain(right_wing_pos, SERVO_MIN, SERVO_MAX)
            
            # Update servos
            servo_left.angle = left_wing_pos
            servo_right.angle = right_wing_pos
    else:
        # Flapping disabled - return wings to center position
        servo_left.angle = SERVO_CENTER
        servo_right.angle = SERVO_CENTER
        last_flap_time = current_time  # Reset timer so it starts immediately when enabled
    
    # Small delay to prevent tight loop
    time.sleep(0.001)

