# Fridge Time Relay

## Overview
Fridge controller offering two operating modes:
- timer driven relay switching for freeze/rest cycles (default logic)
- DS18B20-based thermostat mode with EEPROM stored target temperature (default -12 °C)

The firmware also shows door open/close status on the LCD when a magnetic sensor is attached (top-right indicator: `[]` closed, `<>` open).

A Timer1-based scheduler runs display refresh, DS18B20 sampling, and relay control tasks without blocking delays, ensuring responsive "real-time" updates.

### ToDo
#### Required: 
- [x] LCD info print
- [x] Simple timers for off and on modes

### Components

- 1x Arduino Uno board
- 1x I2C PCF8574 extension 8bit LCD parallel interface
- 1x 1602A LCD display
- 1x relay (5V switching current, 250V/125V 10A contacts)
- 1x DS18B20 digital temperature sensor
- 1x door reed switch (normally closed to ground when door shut)
- 1x rotary encoder with push button (e.g. KY-040)
- 5-9V DC adapter

### Wiring

The principal wiring diagram is shown in [diagram.svg](diagram.svg) (preview below). It highlights the default pin mapping:

- DS18B20 data → `D2` (with 4.7 kΩ pull-up to 5V)
- Relay IN → `D3`
- Door sensor → `D4` (uses internal pull-up)
- Encoder CLK/DT/SW → `D5`/`D6`/`D7`
- LCD I²C backpack → `A4` (`SDA`) and `A5` (`SCL`)
- Power rails shared between every module (5V/GND)

![Wiring diagram](diagram.svg)

### Modes & configuration

- Timer mode follows the predefined Freeze/Rest cycle and still respects DS18B20 safety limits (-18 °C stop, -15 °C resume).
- Thermostat mode keeps the fridge near the stored setpoint (default -12 °C) using 1 °C hysteresis.

Use the serial console (9600 baud) to change and persist settings (times accept `mm:ss` or minutes):

```
mode timer      # return to timer schedule
mode thermo     # thermostat only mode
temp -12.5      # set thermostat setpoint in °C
frost time 30:00  # freeze duration mm:ss (default 30:00)
wait time 45:00   # rest duration mm:ss (default 45:00)
frost temp -18 -15 # timer stop/resume thresholds in °C
status          # print current mode and setpoint
help            # list available commands
```
All values are saved in EEPROM so they survive power cycles.

### Encoder controls

The onboard rotary encoder provides a menu-driven interface without serial commands:

- **Press** the knob (or rotate while on the home screen) to enter the menu.
- **Rotate** to pick an item (Mode, Freeze/Rest timers, frost stop/resume temps, thermostat target, Exit).
- **Press** to edit the highlighted entry; rotate to change values (1-minute or 0.5 °C steps) and press again to save.
- Choose **Exit** to return to the normal status screen.

The top-right indicator (`[]` closed, `<>` open, `??` unknown) continues to reflect the door sensor while navigating the menu.

# Authors

Sergey Bezugliy
- [s.bezugliy@codenv.top](mailto:s.bezugliy@codenv.top) 
- [s.bezugliy@gmail.com](mailto:s.bezugliy@gmail.com)
- https://codenv.top
- https://codenvfron.web.app
