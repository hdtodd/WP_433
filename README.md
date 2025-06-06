# WP_433 v1.2
## An rtl_433-compatible, Arduino Uno-based remote weather probe

WP_433 is an Arduino microcontroller program for transmitting weather sensor data to an rtl_433 server via 433MHz (ISM band) radio transmission.  It is a redevelopment into the rtl_433 ISM-band wireless remote sensor environment of the earlier [WeatherStation](https://github.com/hdtodd/WeatherStation) code.  That system relied upon a USB-serial connection to a host computer for logging data.  WP_433 transmits over the ISM band so that standard rtl_433 tools can be used to process or log data.  It does not require an ongoing connection to a host computer.

![WP_433](https://github.com/user-attachments/assets/543b5e53-7f6d-4d51-9d03-e541546d2cf7)


The rtl_433 JSON reports of the transmitted data packets look like this:
```
{"time":"2025-03-08 11:36:24","protocol":275,"model":"omni","id":1,"channel":1,
"temperature_C":22.5,"temperature_2_C":23.7,"humidity":23.0,"Light %":75.0,
"pressure_hPa":996.4,"voltage_V":4.75,"mic":"CRC","mod":"ASK","freq":433.95155,
"rssi":-0.211433,"snr":16.71867,"noise":-16.9301}
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

## Version History

| Version  | Changes |
|----------|---------|
| V1.2     | 2025.05.15 Correct comments to change "humidity_2" to "Light %". |
| V1.1     | 2025.05.10 Changed CRC-8 'init' from 0x00 to 0xaa, following a recommendation by Christian Zuckschwerdt, reviewing Williams' recommendation not to use 0x00 as 'init', and modeling reliability of error detection in the event of block error insertions [ISMErrDetect](https://github.com/hdtodd/ISMErrDetect).|
| V1.0    | 2025.03.04  First operational version. |

## Author
David Todd, hdtodd@gmail.com, 2025.03
