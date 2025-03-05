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

The WS_433 code supports the use of any combination of these sensors.  If no sensor is found, WP_433 simply reports the voltage of the Arduino power supply and a fictional value for light intensity.

## Requirements

*  A host computer with the Arduino IDE installed
*  An Arduino Uno or similar microcontroller
*  A 433MHz transmitter (or ISM-band transmitter using a frequency legal in your locale)
*  One or more sensors from the set [MPL3115, DHT20, DS18B10, OSEPP Light-01] connected into the microcontroller
*  The appropriate library code for each device installed in the Arduino IDE library.
*  An rtl_433 server with [omnisensor](https://github.com/hdtodd/omnisensor_433) protocol support installed.

## Getting Started

1.  On your host computer, clone this repository as a subdirectory of your "Arduino" directory, from the Arduino IDE, so that it has the Arduino board and device libraries available.
2.  Connect your Arduino to your host computer.
3.  Compile and download to your Arduino the `WP_433.ino` program from this repository.  If any device libraries are not found, install them with the library manager and recompile, then download.
4.  When the program has successfully downloaded, start the Serial Monitor window on your host computer to view the sensor readings and the rtl_433 message as broadcast.
5.  Connect a 433MHz transmitter to pin 3 of the Arduino (with VCC and GND connections).
6.  Monitor the received broadcasts on your `omnisensor`-enable rtl_433 server with, for example, the command `mosquitto_sub -h <your host> -t "rtl_433/<your host>/events"`.  Watch for packets labeled as coming from device `omni`, with device ID = 1 and channel = 1. Confirm that the data decoded by rtl_433 matches the readings reported by the Arduino on your host computer's Serial Monitor window.
7.  You can set the device ID to any value 0..15 in the `WP_433.ino` code, but the channel (labeled `fmt` in the `.ino` code) must remain set to 1 unless you implement your own `omnisense` format in the `.ino` code and in the rtl_433 `omni.c` decoder.

In operation, the Arduino IDE Serial Monitor window will show:

```
17:51:26.418 -> --------------------------------
17:51:26.450 -> Report of readings from sensors:
17:51:26.514 -> DHT20:  	Temp=22, RH=23%
17:51:26.514 -> MPL3115:	Temp=23, Alt=181m, Baro=101895Pa
17:51:26.578 -> DS18B20:	Temp IN: 23    
17:51:26.611 -> Light:		98%
17:51:26.775 -> Transmit msg 21	iTemp=23.30˚C, oTemp=22.90˚C, iHum=23.00%, Light=98.00%, Press=1019.00hPa, VCC=4.75volts
17:51:26.871 -> 	The msg packet, length=80, in hex: 0x 11 0E 90 E5 17 62 27 CE AF 2E 
17:51:26.968 -> 	and as a bit string: 00010001000011101001000011100101000101110110001000100111110011101010111100101110
```

and monitoring the rtl_433 JSON feed with `mosquitto_sub -h pi-1 -t "rtl_433/pi-1/events"`, for example, will show
```
{"time":"2025-03-04 17:55:08","protocol":275,"model":"omni","id":1,"channel":1,
"temperature_C":23.3,"temperature_2_C":22.8,"humidity":23.0,"Light %":99.0,
"pressure_hPa":1018.8,"voltage_V":4.77,"mic":"CRC","mod":"ASK",
"freq":433.95258,"rssi":-0.220131,"snr":16.9172,"noise":-17.1373}
```

## Operational Notes

*  If the light sensor is connected to 3V3 as its VCC, "#define VCC 3" in WS_433.h in the "light sensor" section.
*  The light sensor pin is set in "INPUT_PULLUP" mode, so in the absence of a sensor, the light reading will be ~100% and will be reported even if no other sensors are found.
*  The Arduino Uno VCC measurement is approximate.  Its accuracy can be improved by calibrating VCC with a voltmeter, but the measurement is mostly useful for monitoring trends.



## Author
David Todd, hdtodd@gmail.com, 2025.03

2025.03.04  v1.0
