# WP_433 v1.0
## An rtl_433-compatible, Arduino Uno-based remote weather probe

WP_433 is an Arduino microcontroller program for transmitting weather sensor data to an rtl_433 server via 433MHz (ISM band) radio transmission.  It is a redevelopment into the rtl_433 ISM-band wireless remote sensor environment of the earlier [WeatherStation](https://github.com/hdtodd/WeatherStation) code.  That system relied upon a USB-serial connection to a host computer for logging data.  WP_433 transmits over the ISM band so that standard rtl_433 tools can be used to process or log data.  It does not require an ongoing connection to a host computer.


The rtl_433 JSON reports of the transmitted data packets look like this:

```
{"time":"2025-03-04 17:55:08","protocol":275,"model":"omni","id":1,"channel":1,
"temperature_C":23.3,"temperature_2_C":22.8,"humidity":23.0,"Light %":99.0,
"pressure_hPa":1018.8,"voltage_V":4.77,"mic":"CRC","mod":"ASK",
"freq":433.95258,"rssi":-0.220131,"snr":16.9172,"noise":-17.1373}
```


WP_433 uses the [omnisensor](https://github.com/hdtodd/omnisensor_433) protocol to encapulate data from multiple types of sensors into a single message.  That protocol allows for multiple formats for messages, so the WP_433 system can be easily adapted to other types of sensors.

The repository release code includes support for any combination the following sensors:

*  MPL3115 temperature and barometric pressure
*  DHT20 temperature and humidity
*  DS18B20 (one to four) temperature probes (attached with 1m cables)
*  Light-01 photoresistor light sensor.

Collection of data from sensors is isolated to one procedure to facilitate easy incorporation of other models of sensors.

An installation and customization guide is provided in the `docs` directory of this repository.

A breadboard wiring diagram is provided in the `docs` directory of this repository.

## Author
David Todd, hdtodd@gmail.com, 2025.03

2025.03.04  v1.0
