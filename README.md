# WP_433 v1.0
## An rtl_433-compatible, Arduino Uno-based remote weather probe

WP_433 is an Arduino microcontroller program for transmitting weather sensor data to an rtl_433 server via 433MHz (ISM band) radio transmission.  It is a redevelopment into the rtl_433 ISM-band wireless remote sensor environment of the earlier [WeatherStation](https://github.com/hdtodd/WeatherStation) code.  That system relied upon a USB-serial connection to a host computer for logging data.  WP_433 transmits over the ISM band so that standard rtl_433 tools can be used to process or log data.  It does not require an ongoing connection to a host computer.

WP_433 uses the [omnisensor](https://github.com/hdtodd/omnisensor_433) protocol to encapulate data from multiple types of sensors into a single message.  That protocol allows for multiple formats for messages, so the WP_433 system can be easily adapted to other types of sensors.

## Components

WP_433 was developed with the Arduino Uno, but it should be compatible with other microcontrollers that are supported by the Arduino IDE.  The sensor components supported in the code in this repository include:

*  The MPL3115 temperature/barometer sensor
*  The Adafruit DHT20-ADT20 temperature/humidity sensor
*  Multiple DS18B20 thermal sensors
*  The OSEPP Light-01 light sensor.

The WS_433 code supports the use of any combination of these sensors.  If no sensor is found, WP_433 simply reports the voltage of the Arduino power supply.

## Requirements

*  A host computer with the Arduino IDE installed
*  An Arduino Uno or similar microcontroller
*  A 433MHz transmitter (or ISM-band transmitter using a frequency legal in your locale)
*  One or more sensors from the set [MPL3115, DHT20, DS18B10, OSEPP Light-01] connected into the microcontroller
*  The appropriate library code for each device installed in the Arduino IDE library.
*  An rtl_433 server with [omnisensor](https://github.com/hdtodd/omnisensor_433) protocol support installed.




## Author
David Todd, hdtodd@gmail.com, 2025.03

2025.03.04  v1.0
