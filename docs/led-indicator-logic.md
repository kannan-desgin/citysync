# CitySync RT+ — LED Indicator Logic

## LED pins and conditions

| LED | BCM GPIO | Physical pin | ON condition |
|---|---:|---:|---|
| LED1 — Fire | GPIO5 | 29 | Flame sensor detection (active-low input) |
| LED2 — Water / distance | GPIO6 | 31 | Valid measured ultrasonic distance > 30 cm |
| LED3 — Current | GPIO13 | 33 | Simulated current > 1500 mA from PCF8591 AIN1 |

## LED1 — Fire

The flame input is active-low. A LOW input is interpreted as Flame detection. The Fire thread updates LED1 and sends the dedicated Fire message to TwinSync.

## LED2 — Ultrasonic distance

The Water thread accepts distance readings in the 2–400 cm configured range. LED2 turns ON when measured distance is greater than 30 cm and OFF otherwise. Invalid/no echo clears LED2 and sends a NO_ECHO status.

**Important:** The code compares sensor-to-surface distance, not calculated water height. For an ultrasonic sensor mounted above a tank, greater distance normally indicates a lower water surface. Use tank geometry and a reference height to calculate actual water level.

## LED3 — Simulated current

The Current thread reads PCF8591 AIN1 and scales the 8-bit ADC value to a simulated 0–5000 mA range:

simulated_current_ma = (raw / 255.0) × 5000.0

LED3 turns ON when the simulated value is greater than 1500 mA. The potentiometer is a test input, not a true current sensor. An ADC read failure clears LED3.

## Independent output handling

The states flame_active, ultrasonic_active, and current_active are maintained independently. update_priority_outputs() updates only the state belonging to the selected sensor. A dedicated QNX mutex protects the state updates and GPIO writes, so multiple LEDs can remain ON at the same time.

All LED pins are configured as outputs and initialized OFF. The LEDs are turned OFF during normal shutdown.

## Wiring

Use one 2.2 kΩ current-limiting resistor in series with each LED:
- GPIO5 → 2.2 kΩ → LED1 anode; cathode → GND.
- GPIO6 → 2.2 kΩ → LED2 anode; cathode → GND.
- GPIO13 → 2.2 kΩ → LED3 anode; cathode → GND.

A 2.2 kΩ resistor is a conservative indicator choice for 3.3 V GPIO; brightness depends on the LED. Never connect an LED directly to a GPIO pin.

## Validation checklist

- [ ] Test LED1 with Flame clear and detected.
- [ ] Test LED2 below, exactly at, and above 30 cm.
- [ ] Test LED3 below, at, and above the 1500 mA simulated threshold.
- [ ] Trigger multiple conditions and confirm LEDs remain ON independently.
- [ ] Verify a failed sensor reading clears its associated LED.
- [ ] Verify all LEDs turn OFF during normal shutdown.
