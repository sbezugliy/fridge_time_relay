# Fridge Controller / Thermostat

### WIP

## Overview
Controller providing turning on and off for freezing/heating devices 
depending on temperature received from sensor.

### ToDo
#### Required: 
- [x] LCD info print
- [x] LCD brightness control 
- [x] OneWire thermal sensor programming and data gathering
- [x] OneWire thermal sensor diagnostics
- [x] Buttons increasing/decreasing temperature
- [x] Error handling and state reporting
- [ ] Relay switching when temp goes inside of target temp range
- [ ] Timer preventing early/late switching and frequent switching
 for short period of time. Too keep safety of endpoint electronics.
#### Optional:
- [ ] External permanent settings storage.
- [ ] MQTT or TCP RPC/Messaging with controlling host. 
- [ ] WiFi module interconnection(ESP16, ESP32, etc...) and/or Z-wave
- [ ] humidity sensor
- [ ] Sensor of unfrozen water on the bottom
- [ ] Remote notification about issues
- [ ] Web/Mobile app
- [ ] Statistics gathering


### Components

- 1x Arduino Uno board
- 1..16x 
- 1x 1602A LCD display
- 2x micro buttons
- 3x 1K resistors
- 1x 4.7k resistors
- 1x relay(5V switching currency, 250V/125V 10A switchable currency)
- 1K potentiometer/variable resistor
- 5-9V DC adapter

### Parameters
- Temperature range: -55C .. +85C
- Switchable currency: 
  - 250V 10A
  - 125V 15A
- LCD display

# Authors

Sergey Bezugliy
- [s.bezugliy@codenv.top](mailto:s.bezugliy@codenv.top) 
- [s.bezugliy@gmail.com](mailto:s.bezugliy@gmail.com)
- https://codenv.top
- https://codenvfron.web.app
