# WP_433 v1.0 Installation Guide
## An rtl_433-compatible, Arduino Uno-based remote weather probe

## Components

WP_433 was developed with the Arduino Uno, but it should be compatible with other microcontrollers that are supported by the Arduino IDE.  The sensor components supported in the code in this repository include:

*  The MPL3115 temperature/barometer sensor
*  The Adafruit DHT20-ADT20 temperature/humidity sensor
*  Multiple DS18B20 thermal sensors, likely the 1m cable versions
*  The OSEPP Light-01 light sensor.

The WS_433 code supports the use of *any combination* of these sensors.  If no sensor is found, WP_433 simply reports the voltage of the Arduino power supply and a fictional value near 100% for light intensity.

<img width="1452" alt="WP_433_circuit" src="https://github.com/user-attachments/assets/d6b31f6e-6fb4-4beb-ba23-941934daec5b" />

## Requirements

*  A host computer with the Arduino IDE installed
*  An Arduino Uno or similar microcontroller
*  A 433MHz transmitter (or ISM-band transmitter using a frequency legal in your locale)
*  One or more sensors from the set [MPL3115, DHT20, DS18B10, OSEPP Light-01] connected into the microcontroller.  In the circuit diagram, the DS18B20 is shown as a single breadboard device.  A more likely implementation would be to use multiple DS18B20's on 1m cables to monitor temperatures in multiple locations (indoor/outdoor, multiple vats, etc.).
*  The appropriate library code for each device installed in the Arduino IDE library.
*  An rtl_433 server with [omnisensor](https://github.com/hdtodd/omnisensor_433) protocol support installed.

## Getting Started

1.  On your host computer, clone this repository as a subdirectory of your "Arduino" directory, from the Arduino IDE, so that it has the Arduino board and device libraries available.
2.  Connect your Arduino to your host computer.
3.  Compile and download to your Arduino the `WP_433.ino` program from this repository.  If any device libraries are not found, install them with the IDE library manager and recompile.  When successfully compiled, download to the Arduino.
4.  When the program has successfully downloaded, start the Serial Monitor window on your host computer to view the sensor readings and the rtl_433 message as broadcast.
5.  Connect a 433MHz transmitter to pin 3 of the Arduino (with VCC and GND connections).
6.  Monitor the received broadcasts on your `omnisensor`-enabled rtl_433 server with, for example, the command `mosquitto_sub -h <your host> -t "rtl_433/<your host>/events"`.  Watch for packets labeled as coming from `"model":"omni","id":1,"channel":1`.  Confirm that the data decoded by rtl_433 matches the readings reported by the Arduino on your host computer's Serial Monitor window.
7.  Connect from the set [MPL3115, DS18B20, DHT20, Light-01] each of the sensors you have into the Arduino, one at a time, and restart the Arduino after each addition.  As you add sensors, the reports of their readings should appear on the Serial Monitor and as received by the rtl_433 server.  If not, check your wiring.

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

and monitoring the rtl_433 JSON feed with `mosquitto_sub -h pi-1 -t "rtl_433/pi-1/events"`, for example, will show:

```
{"time":"2025-03-04 17:55:08","protocol":275,"model":"omni","id":1,"channel":1,
"temperature_C":23.3,"temperature_2_C":22.8,"humidity":23.0,"Light %":99.0,
"pressure_hPa":1018.8,"voltage_V":4.77,"mic":"CRC","mod":"ASK",
"freq":433.95258,"rssi":-0.220131,"snr":16.9172,"noise":-17.1373}
```

## Reported Values

The values available for transmitting come from the various sensors installed:

*  The set of sensors supported provide up to 6 different temperatures for transmitting: 1 MPL3115, 1 DHT20, up to 4 DS18B20's.
*  Barometric pressure comes from the MPL3115; humidity comes from the DHT20.
*  Light intensity comes from the OSEPP Light-01 sensor.
*  The VCC reported comes from measuring the Arduino internal reference voltage and scaling it relative to the nominal 5v USB power source.

The readings from the individual sensors are displayed on the Serial Monitor window (if `DEBUG` has been defined, as it is by default). The hexadecimal and binary representations of the ISM-band packet transmitted are also displayed.

In the repository code as distributed, the values reported by rtl_433 **as transmitted values** come from these  sources:

*  "protocol":275: set by your `omni.c` decoder protocol number based upon the timings of the signal pattern received matching the timings expected for `omni.c`
*  "model":"omni": set by the rtl_433 decoder based upon the protocol number
*  "id":1,"channel":1: set by the `WP_433.ino` code.  You can change "id" to be anything from 0..15, but "channel" is the format number (`fmt` in `WP_433.ino`) for this `omni` packet type.  If you customize `WP_433.ino` to broadcast other sensor data, you'll need to introduce a new format type into both `WP_433.ino` and rtl_433's `omni.c`.  See the [omnisensor_433](https://github.com/hdtodd/omnisensor_433) repository for details.
*  "temperature_C": Temperature from the DS18B20 labeled "IN", if one is found; otherwise the temperature reading from the MPL3115 if available; else 0.
*  "temperature_2_C": Temperature from the DS18B20 labeled "OU", if one is found; otherwise the temperature reading from the DHT20 if available; else 0.
*  "humidity": Relative Humidity, in %, from DHT20.
*  "Light %": Read as voltage from the Light-01 photoresistor sensor and scaled to be 0-100%; see "Customizing" if Light-01 is connected to 3V3 as VCC.
*  "pressure_hPa": Barometric pressure, in hPa, from MPL3115.
*  "voltage_V": Approximate Arduino VCC supply voltage; scaled from the internal reference voltage of the Arduino.  Not accurate, but may show tends.

Here are details about the data reported and transmitted.

The original `WeatherStation` code was designed to support indoor and outdoor DS18B20 thermal sensors on 1m cables as well as humidity and barometric pressure readings from DHT22 (replaced here by DHT20) and MPL3115 sensors, respectively.  `WP_433` continues that support and adds a light sensor and internal VCC reading.

But with up to 6 sources for temperature and only two slots for transmitting temperature in the `omni` format `fmt=1` packet structure, some arbitrary choices were made that can easily be changed (see "Customizing").

The `WP_433` code assumes that the DS18B20 sensors have been labeled internally as "IN" and "OU" and prioritizes reporting them as the two temperature readings. You can use the `DS18_Labeler` program from the `examples` folder in the [DS18](https://github.com/hdtodd/DS18) repository to label your DS18's.  In the absence of DS18B20's with labels "IN" and "OU", the code in `loop()` assigns these values as "temperature_C" and "temperature_2_C" (`itemp` and `otemp`, respectively:
*  `itemp` is the temperature from DS18B20 labeled "IN", if found; otherwise the reading from MPL3115 if there is one; otherwise 0;
*  `otemp` is the temperature from DS18B20 labeled "OU", if found; otherwise the reading from the DHT20 if there is one; otherwise 0.

If you don't have or intend to use DS18B20's, you may want to customize the code in `loop()` in `WP_433.ino` to select `itemp` and `otemp` from the set of sensors you do have.

## Customizing

###  Operating Parameters

Operating parameters are set in both the `.h` and `.ino` files.

1.  The pin assignments for the sensors are made in the `.h` file.  If you add or change sensors, edit the pin numbers in that file so as to avoid conflicts.
2.  If the Light-01 sensor is connected to 3V3 as its VCC, edit the definition in `WP_433.h` to `#define VCC 3` (which	affects	only the scaling of voltage for	light-intensity	readings).
3.  The barometer reading is affected by the altitude of the MPL3115; if you know your altitude, change the value set by `#define MY_ALTITUDE` (set to 183m by default).
4.  If you do not want the sensor and packet information displayed on the Arduino IDE Serial Monitor, set `#undef DEBUG` in `WP_433.ino` (DEBUG is on by default).
5.  Sensors are sampled and transmitted at a frequency set by `#define LOOPTIME` in `WP_433.ino`.  Change it from the default 30 sec (30*1000 milliseconds) if you want some other sampling rate.
6.  You can set the device ID to any value [0..15] in the `WP_433.ino` code, but the channel (variable `fmt` in the `.ino` code) must remain set to 1 unless you implement your own `omnisensor` format in the `.ino` code and in the rtl_433 `omni.c` decoder.
7.  To change the selection of temperatures reported as "temperature_C" and temperature_2_C", modify the code in `loop()` to select the fields of the `rec` structured variable to use for `itemp` and `otemp`.
8.  If you're using DS18B20 thermal sensors, you can set their internal labels (e.g, "IN" and "OU") using the `DS18_Labeler` program from the `examples` folder in the [DS18](https://github.com/hdtodd/DS18)

### Replacing Sensors

The code was designed to make it relatively easy to replace the repository's set of sensors (MPL3115, DHT20, DS18B20, Light-01) with other sensors:

*  Sensor data for all the sensors are collected in `readSensors()` and reported back to the Serial Monitor by `reportSensors()`, so most editing would be done in those two procedures.
*  The sensor data (except Arduino VCC) are collected in a single `struct recordValues` variable.
*  All data are represented as type `float` in that `struct` record.
*  The intentionally-succinct main `loop()` code converts those `float` values to the type appropriate for the data being transmitted (e.g., in the ISM message packet, the value for `light` is a `uint8_t` byte to represent 0% to 100%; temperatures are of type `int16_t`; etc.), with the data type in that loop designed to match the format needed for transmission in the `omni` protocol and as packed for message transmission by `pack_msg()` in the `class WP_433` definition.

So to use a different type of sensor, you would:

*  Remove references to the sensor you're replacing, including pin assignments in the `.h` file and insert any new pin definitions you need, making sure to avoid pin-assignment conflicts (pin dictionary at the top of the `.h` file).
*  `#include` the `.h` file for the sensor library you want to invoke.
*  In `setup()`, remove any initiation of the sensor you're replacing and insert the initiation for your new sensor.
*  In `readSensors()`, replace the reading of sensor data from the replaced sensor with reading of data from your new sensor and assign the value to the appropriate `rec` field.
*  If you have different types of data (other than temperatures, humdity, barometric pressure, a % reading from some device, or a voltage reading), you'll need to modify the `struct` definition in the `.h` file and the references to them in `readSensors`, `reportSensors`, and the assignment to packet fields in `loop()`.
*  If you need a different format for your data and can't reuse one of the existing packet fields for the data, you'll need to modify the code in the `class WP_433` definitions for `pack_msg()` and `unpack_msg()` to create and retrieve your data from the 8-byte data payload packet transmitted by `WP_433`.  See the documentation and tools in the [ISM_Emulator](https://github.com/hdtodd/ISM_Emulator) repository for code that can help you create and debug that pack/unpack format.

## Operational Notes

*  The light sensor pin is set in "INPUT_PULLUP" mode, so in the absence of a sensor, the light reading will be ~100% and will be reported even if no other sensors are found.
*  The Arduino Uno VCC measurement is approximate.  Its accuracy can be improved by calibrating VCC with a voltmeter, but the measurement is mostly useful for monitoring trends.


## Author
David Todd, hdtodd@gmail.com, 2025.03

2025.03.04  v1.0
