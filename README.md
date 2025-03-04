# WP_433 v1.0
## An rtl_433-compatible, Arduino Uno-based remote weather probe

WP_433 is an Arduino microcontroler program for transmitting weather sensor data to an rtl_433 server via 433MHz (ISM band) radio transmission.  It is a redevelopment into the rtl_433 ISM-band wireless remote sensor system of the earlier [WeatherStation](https://github.com/hdtodd/WeatherStation) code.  That system relied upon a USB-serial connection to a host computer whereas WP_433 transmits over the ISM band and so does not require an ongoing connection to a host computer.

## Components

WP_433 was developed with the Arduino Uno, but it should be compatible with other microcontrollers that are supported by the Arduino IDE.  The components supported in the code in this repository include:

*  The Arduino Uno R3
*  A 433MHz transmitter (or other ISM-band transmitter appropriate for your locale)
*  The MPL3115 temperature/barometer sensor
*  The Adafruit DHT20-ADT20 temperature/humidity sensor
*  Multiple DS18B20 thermal sensors
*  The OSEPP Light-01 light sensor.

The WS_433 code supports the use of any combination of these sensors.

If no sensor is found, WP_433 simply reports the voltage of the Arduino power supply.



## Author
David Todd, hdtodd@gmail.com, 2025.03

2025.03.04  v1.0
