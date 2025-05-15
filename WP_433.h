// Definitions used by the Arduino Uno Weather Sensor,  WP_433.ino
//
#define Vers "WP_433 v1.2"    // <Code-version> 

#define LED    13      // Flash LED on pin 13 when transmitting
#define REPEATS 3      // Number of times to repeat packet in one transmission

/* Summary of Uno pin assignments:
   A3   OSEPP Light-01 light sensor data line
   D3   433Mhz XMT data line
   D5   DS18 data line (OneWire data line)
   SCL  MPL3115 & DHT20 SCL
   SDA  MPL3115 & DHT20 SDA
   MPL3115 & DHT20 have voltage level-shifting;
   no resistor needed to use 5V as VCC
*/

// Transmitter connection
#define TX      3      // Use pin 3 to control transmitter

// OSEPP Light-01 light sensor
//    Data line A3; VCC to Uno 5V, GND to Uno GND
// Analog pin reads 0-1023 for 0-VCC voltage
// Mapped to 0..100 for light intensity
// Since it can operate on 3V3 to 5V0, we need to
//    map the measured voltage against the range
//    that VCC might have.  Set VCC here to do that.
int light_sensor = A3;
#define VCC 5
#if VCC != 5 
    static const long vmax = (3.3/5.0)*1023;  // assume 3V3
#else        // assume 5v0
    static const long vmax = 1023;            // assume 5v0
#endif

// DS18 pin definitions and parameters
// We use powered rather than parasitic mode
//     DS18 data wire to Uno pin D5
//     with 4K7 Ohm pullup to VCC 5V
//     VCC to Uno 5v, GND to Uno GND
#define oneWirePin 5          // Uno pin 5 for OneWire connections to DS18B20
#define dsResetTime 250       // delay time req'd after search reset in msec
#define DSMAX 4               // max number of devices we're prepared to handle

// DHT20 and MPL3115 
//     SCL to Uno SCL
//     SDA to Uno SDA
//     VCC to Uno 5v, GND to Uno G

// MPL3115 settings
uint8_t sampleRate=S_128;     // Sample 128 times for each MPL3115A2 reading
#define MY_ALTITUDE 183       // Set to your GPS-verified altitude in meters
                              // if you want altitude readings to be corrected
                              // Bozeman, MT = 1520m, Williston VT = 169m, Colchester CT = 183
#define MY_ELEV_CORR 18.0*100 // millibars-> Pascals, From Table 2,
                              //   https://www.starpath.com/downloads/calibration_procedure.pdf
#define MY_CALIB_CORR 3.5*100 // millibars-> Pascals, calibrated for my MPL: replace with yours
#define FT_PER_METER 3.28084  // conversion

struct mplReadings {
  float alt;
  float press;
  float tempf;
};

struct dhtReadings {
  float tempf;
  float rh;
};

struct dsReadings {
  char label[DSMAX][3];
  float tempf[DSMAX];
};

struct recordValues {
  struct mplReadings mpl;
  struct dhtReadings dht;
  struct dsReadings ds18;
  uint8_t light;
};
