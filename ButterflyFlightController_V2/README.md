# Butterfly Flight Controller V2

Flight controller for an RC ornithopter (mechanical butterfly) running on an ESP32-C3 Mini with a FrSky RXS-R receiver.

Based on the [Ctorque Mech-Butterfly](https://github.com/) MicroPython code, rewritten in Arduino C++ with FrSky F.Port support and additional features.

## Features

- **Sinusoidal flap motion** — Smooth cosine-wave wing movement (40 steps/cycle), replacing the original binary snap flap. Produces natural, efficient wing motion.
- **Variable amplitude** — Adjust wing sweep angle in flight (20-70 degrees) via a stick or knob.
- **Pitch control** — Shift both wing centers for climb/dive bias.
- **Differential turning** — Aileron input makes one wing flap more than the other for yaw control.
- **Per-wing trim** — Independent trim correction for left and right wings via aux channels.
- **Battery monitoring** — ADC-based voltage reading with low-pass filter. Displays on serial and sends telemetry to transmitter.
- **F.Port telemetry** — Battery voltage (VFAS) and percentage (Fuel) sent back to your FrSky transmitter via half-duplex F.Port.
- **Non-blocking** — F.Port is read every loop iteration; wing motion never blocks communication.
- **WS2812 wing LEDs** — Addressable LED effects synced to flight state: rainbow wave during flapping, amber breathing when idle, red flash on failsafe, orange pulse on low battery.
- **Failsafe** — Wings center and stop flapping if F.Port signal is lost for 500ms.

## Hardware

| Component | Details |
|-----------|---------|
| MCU | ESP32-C3 Mini Development Board (3.3V logic) |
| Receiver | FrSky RXS-R (uninverted S.Port / F.Port on P pad) |
| Servos | 2x micro servos for wing control |
| Battery | 2S LiPo (7.4V nominal) |
| BEC | 5V BEC from 2S battery to power the ESP32-C3 via VIN pin |
| Voltage divider | Resistor divider on battery line for ADC monitoring |
| LEDs | WS2812 / NeoPixel strip (mounted on wings) |

## Wiring

```
FrSky RXS-R P pad ──── GPIO 7   (F.Port, half-duplex)
Left wing servo ─────── GPIO 4
Right wing servo ────── GPIO 5
Battery voltage divider  GPIO 0   (ADC input)
WS2812 LED data ──────── GPIO 3   (NeoPixel strip)
5V BEC output ────────── VIN      (ESP32-C3 power)
GND ──────────────────── GND
```

### Battery Voltage Divider

The ESP32-C3 ADC can only read up to ~2.5V (with 11dB attenuation). A resistor divider scales the battery voltage down to a safe range.

```
VBAT ──[ R1 30k ]──┬──[ R2 10k ]── GND
                   └── GPIO 0
```

With R1=30k and R2=10k, the divider ratio is 4:1.

- 2S full (8.4V) / 4 = 2.1V (safe for ADC)
- 2S empty (6.6V) / 4 = 1.65V

Set `VDIV_RATIO` in the code to match your actual resistor values:

```
VDIV_RATIO = (R1 + R2) / R2
```

**Important:** Do NOT connect the battery directly to any GPIO pin. The ESP32-C3 is a 3.3V chip — voltages above 3.3V on any GPIO will damage it.

## Channel Mapping

Configure these on your FrSky transmitter (OpenTX / EdgeTX):

| Channel | Input | Function | Range |
|---------|-------|----------|-------|
| Ch 0 | Throttle | Flap speed | Slow ↔ Fast (8-16ms/step) |
| Ch 1 | Aileron | Turning (differential) | Left ↔ Right (±20 deg) |
| Ch 2 | Elevator | Pitch offset | Dive ↔ Climb (±20 deg) |
| Ch 3 | Rudder | Flap amplitude | Small ↔ Large (20-70 deg) |
| Ch 4 | Aux switch | Flap enable/disable | Below center=off, above=on |
| Ch 5 | Aux knob/slider | Left wing trim | ±20 deg |
| Ch 6 | Aux knob/slider | Right wing trim | ±20 deg |
| Ch 7 | Aux switch | LED on/off | Below center=off, above=on |
| Ch 8 | 3-pos switch | LED mode | Low=solid, Mid=rainbow, High=flap-sync |
| Ch 9 | Aux pot/slider | LED brightness | 0-100% |
| Ch 10 | Aux pot/slider | LED color hue | Full color wheel |

## Telemetry

The controller sends battery data back through F.Port. These values appear automatically on your transmitter once discovered:

| Sensor | Value | Description |
|--------|-------|-------------|
| VFAS | Voltage | Battery voltage (e.g. 7.85V) |
| Fuel | Percentage | Battery level (0-100%) |

You can set up voice alerts on the transmitter for low battery warnings.

## LED Effects

WS2812 LEDs on GPIO 3, fully controlled from the transmitter via Ch 7-10.

**Ch 7** turns LEDs on or off. **Ch 9** controls brightness. **Ch 10** selects the color hue (used by solid and flap-sync modes).

**Ch 8** selects the mode with a 3-position switch:

| Switch position | Mode | Description |
|-----------------|------|-------------|
| Low | Solid | All LEDs set to the color selected by the hue pot (Ch 10) |
| Center | Rainbow | Classic auto-cycling rainbow spread across the strip |
| High | Flap-sync | Two complementary colors alternate across the strip, swapping brightness with each wing stroke |

### Flap-sync mode

The color pot (Ch 10) picks a base hue. The complementary color (180 degrees opposite on the color wheel) is calculated automatically. Even-numbered LEDs show color A, odd-numbered show color B. As the wings flap, the two colors swap dominance — at one wing extreme color A is bright and B is dim, at the other extreme they swap. This creates a shimmering, pulsing dual-color effect synced to wing motion.

### Wiring

Power the LED strip from the **5V BEC**, not from the ESP32-C3's 3.3V pin. Connect the data line to GPIO 3 (3.3V logic is fine for WS2812 data at short wire distances).

## Tuning

All parameters are defined as constants at the top of the `.ino` file:

| Constant | Default | Description |
|----------|---------|-------------|
| `DELAY_MIN_MS` | 8 | Fastest step delay (ms) — full throttle |
| `DELAY_MAX_MS` | 16 | Slowest step delay (ms) — zero throttle |
| `AMP_MIN` | 20.0 | Minimum amplitude (degrees) |
| `AMP_MAX` | 70.0 | Maximum amplitude (degrees) |
| `DIFF_MAX` | 20.0 | Maximum turning differential (degrees) |
| `PITCH_MAX` | 20.0 | Maximum pitch offset (degrees) |
| `TRIM_MAX` | 20.0 | Maximum per-wing trim (degrees) |
| `FLAP_STEPS` | 40 | Steps per full flap cycle (smoothness) |
| `VDIV_RATIO` | 4.0 | Battery voltage divider ratio |
| `BAT_VOLT_MIN` | 6.6 | 2S LiPo empty voltage |
| `BAT_VOLT_MAX` | 8.4 | 2S LiPo full voltage |
| `BAT_WARN_VOLTAGE` | 7.0 | Low battery warning threshold |
| `BAT_FILTER_ALPHA` | 0.98 | ADC smoothing (higher = smoother) |
| `LED_COUNT` | 10 | Number of WS2812 LEDs on the strip |
| `LED_UPDATE_INTERVAL_MS` | 25 | LED refresh rate (~40fps) |
| `RAINBOW_SPEED` | 300.0 | Rainbow mode rotation speed |

## Dependencies

- [ESP32Servo](https://github.com/madhephaestus/ESP32Servo) — Install via Arduino Library Manager
- [Adafruit NeoPixel](https://github.com/adafruit/Adafruit_NeoPixel) — Install via Arduino Library Manager

## Flashing

1. Open `ButterflyFlightController_V2.ino` in Arduino IDE
2. Select board: **ESP32C3 Dev Module**
3. Install libraries: **ESP32Servo** and **Adafruit NeoPixel**
4. Connect the ESP32-C3 via USB
5. Upload

## Changes from V1

| Feature | V1 | V2 |
|---------|----|----|
| Flap motion | Binary snap (2 positions) | Sinusoidal cosine wave (40 steps) |
| Amplitude | Fixed 45 deg | Variable 20-70 deg (Ch 3) |
| Pitch control | None | ±20 deg (Ch 2) |
| Wing trims | None | ±20 deg per wing (Ch 5-6) |
| Battery monitoring | None | ADC with low-pass filter |
| Telemetry | None | VFAS + Fuel via F.Port |
| F.Port direction | RX only | Half-duplex (RX + TX) |
| Flap speed control | Period 50-500ms | Step delay 8-16ms (smoother) |
| Wing LEDs | None | WS2812 with 3 modes, RC-controlled brightness/color/on-off |

## Credits

- Original Ctorque Mech-Butterfly MicroPython code
- Wing motion algorithm ported from `Kang_Flyer_Ornithopter.py`
- Battery monitoring approach from `Kang_Battery.py`
