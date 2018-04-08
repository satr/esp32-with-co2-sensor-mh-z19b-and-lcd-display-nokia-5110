# The ESP32 with CO2 sensor MH-Z19B (v2) and LCD display from Nokia 5110.
There are few ways to read data from the sensor:
* from PWM output 
* from serial port
* from V0 - analog output (not checked)

This project uses the serial port (works more stable).

Connect the sensor, when data are read from the sensor's serial port.
```
  ESP32        | CO2 sensor MH-Z19B
  pin 16 (RX2) - RX
  pin 17 (TX2) - TX
  GND          - GND
  +5V          - Vin
```
Connect the display
```
  ESP32  | Display Nokia 5110
  pin 14 - Serial clock out: CLK (SCLK)
  pin 13 - Serial data out: DIN
  pin 27 - Data/Command select: DC (D/C)
  pin 15 - LCD chip select: CE (CS)
  pin 26 - LCD reset: RST
  5V    - VCC
  GND   - GND
```

Similar project on an Arduino: https://github.com/satr/arduino-with-co2-sensor-mh-z19b-and-lcd-display-nokia-5110

