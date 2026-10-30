# CitySync RT+ – Circuit and Pin Connections

This document records the pin connections shown in the supplied circuit diagram.

The circuit wiring diagram is included in the downloadable project ZIP at `docs/images/circuit-wiring.png`. It is not yet committed to this GitHub repository.

## Raspberry Pi 4 Pin Connections

| Component | Signal | Raspberry Pi connection | Physical pin |
|---|---|---:|---:|
| HC-SR04 | VCC | 5V | 2 |
| HC-SR04 | TRIG | GPIO23 | 16 |
| HC-SR04 | ECHO | GPIO24 | 18 |
| HC-SR04 | GND | GND | 6 |
| IR sensor | VCC | 3.3V | 1 |
| IR sensor | OUT | GPIO17 | 11 |
| IR sensor | GND | GND | 9 |
| Flame sensor | VCC | 3.3V | 1 |
| Flame sensor | DO | GPIO27 | 13 |
| Flame sensor | GND | GND | 14 |
| PCF8591 ADC | VCC | 3.3V | 1 |
| PCF8591 ADC | GND | GND | 9 (common ground) |
| PCF8591 ADC | SDA | GPIO2 / SDA | 3 |
| PCF8591 ADC | SCL | GPIO3 / SCL | 5 |

## Analog Sensor Connections

| PCF8591 channel | Connected device | Purpose |
|---|---|---|
| AIN0 | LM35 Vout | Temperature input |
| AIN1 | Potentiometer output | Simulated current input |

For the LM35 and potentiometer, connect their VCC and GND to the appropriate supply and common-ground rails as represented in the circuit diagram. Verify the LM35 supply and output range for the exact device and ADC setup.

## Power and Ground

- Raspberry Pi 4 is supplied by a 5V power source.
- HC-SR04 VCC is connected to the Pi 5V rail.
- IR sensor, flame sensor, and PCF8591 ADC use the 3.3V rail as shown.
- All sensor grounds and the ADC ground must share a common ground with the Raspberry Pi.
- SDA connects to GPIO2 (physical pin 3), and SCL connects to GPIO3 (physical pin 5).

## Electrical Safety

**Important:** HC-SR04 ECHO can output 5V, while Raspberry Pi GPIO inputs are not 5V tolerant. Use a suitable voltage divider or logic-level shifter between ECHO and GPIO24 before connecting it to the Raspberry Pi. Do not connect a 5V ECHO signal directly to the Pi GPIO.

The diagram is a wiring reference; verify module pin labels and voltage requirements against the exact hardware versions before powering the circuit.
