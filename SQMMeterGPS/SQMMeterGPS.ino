// SQM Meter GPS a DIY Arduino Mega 2560 Project
// Sky Quality Meter, Cloud Sensor, Barometric Sensor, GPS, Lux Sensor

// (c) Copyright Marcus Vasi 2018- . All Rights Reserved.
// The schematic, code and ideas for SQM Meter GPS are released into the public domain. Users are free to implement these but
// may NOT sell this project or projects based on this project for commercial gain without express written permission
// granted from the author. Schematics, Code, Firmware, Ideas, Applications, Layout are protected by Copyright Law.
// Permission is NOT granted to any person to redistribute, market, manufacture or sell for commercial gain the SQM Meter GPS
// products, ideas, circuits, builds, variations and units described or discussed herein or on this site.
// Permission is granted for personal and Academic/Educational use only.

// The idea was derived from https://sourceforge.net/projects/arduinomysqmskyqualitymeter/ by Robert Brown. Some of his
// libraries are used i this project.

// Version history:
// ---------------------------------------------------------------------------------------------------------------------------
// 1.07 (04.06.2018) - adding Arduino Nano via I2C to get values from TSL237 (better performance)
//
// 1.06 (29.05.2018) - adding button to switch to night mode and vice versa
//
// 1.05 (23.05.2018) - adding SQM Meter GPS specific library SQMMeterGPS.h
//                   - removed function which are covered by the libray SQMMeterGPS.h
//
// 1.04 (22.05.2018) - adding Bortle scale (reference is NELM value)
//                   - adding info text option for splash screen
//
// 1.03 (21.05.2018) - adding local time to GPS time
//
// 1.02 (20.05.2018) - adding the option to use a button to scroll the pages forward
//
// 1.01 (20.05.2018) - increased font size for SQM values on page 3
//
// 1.00 (19.05.2018) - initial version
// ---------------------------------------------------------------------------------------------------------------------------

// IMPORTANT:
// To read the sensor data for TSL237 the sketch NanoSlave.ino for the Arduino Nano (as slave) is needed.

char  programName[]   = "SQM Meter GPS";
char  copyright[]     = "(c) M. Vasi 2018";
float programVersion  = 1.07;
char  programInfo1[]  = "SQM Meter GPS is for nightly use. Used during the day the values are not representative.";
char  programInfo2[]  = "Please wait 2-3 Min. for adaption.";

// Connection for NEO-6M-GPS
// GPS-TXD to Arduino Mega Pin 18 (RX1) Serial1
// GPS-RXD to Arduino Mega Pin 19 (TX1) Serial1
// GND to GND
// VCC to 5V

// Connection for 1.8" TFT display 128x160
// VCC to 5V
// GND to GND
// CS   10
// RST   9
// A0    8
// SDA (MOSI) to 51 (Mega)
// SCL (SCK)  to 52 (Mega)
// LED  3.3V

// Connection for BME280 Barometric Sensor, uses I2C interface
// VCC  5V
// GND  GND
// SDA  20
// SCL  21


// Connection for TSL2561 Sensor - be careful what I2C connector you connect this to
// 3.3V or 5V. Connecting to 5V will destroy the 3.3V sensor
// VCC  3.3V
// GND  GND
// SDA  20
// SCL  21

// Connection for MLX90614 IR Sensor - be careful what I2C connector you connect this to
// 3.3V or 5V. Connecting to 5V will destroy the 3.3V sensor
// VCC  3.3V
// GND  GND
// SDA  20
// SCL  21

// Connection for Arduino Nano, uses I2C interface
// VCC  5V
// GND  GND
// SDA  20 -> A4 (Nano)
// SCL  21 -> A5 (Nano)

// Scroll one page forward
// pagepushButton pin 2
int     pagepushButton   = 2;
boolean pagebuttonPushed = false;
int     pageprevious     = LOW;

// Switch between night mode and vice versa
// nightpushButton pin 3
int     nightpushButton   = 3;
boolean nightbuttonPushed = false;
boolean nightmode         = false;
int     nightprevious     = LOW;

// General
#include <Adafruit_GPS.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>
#include <Timezone.h>        // Calculate time and date for local timezone
#include <SQMMeterGPS.h>     // SQMMeterGPS specific library
#include <Wire.h>
#include <I2C_Anything.h>    // Lib to submit via I2C interface different datatypes not only bytes
#include <Math.h>            // required for log()

// Sensors
#include <mybme280.h>
#include <SparkFunTSL2561.h>
#include <myMLX90614AF.h>

#define SENSORSAMPLETIME 1500  // time between updates of sensors

// what's the name of the hardware serial port?
#define GPSSerial Serial1

// Change this to suit your Time Zone
TimeChangeRule CEST = {"", Last, Sun, Mar, 2, 120};     
TimeChangeRule CET = {"", Last, Sun, Oct, 3, 60}; 
Timezone CE(CEST, CET);
TimeChangeRule *tcr;
time_t utc;   // Universal Time
time_t ltime; // Local Time
char localTime[32];
char localDate[32];

// These pins will work for the 1.8" TFT shield
#define TFT_CS          10
#define TFT_RST         9  
#define TFT_DC          8

#define SKYCLEAR        0
#define SKYPCLOUDY      1
#define SKYCLOUDY       2
#define SKYUNKNOWN      3

#define blackperiod     2000              // 2s Gate time, at very dark sites
#define darkperiod      1000              // 1s Gate time, at dark sites
#define lightperiod     100               // 100ms Gate time for day light

#define SQMSAMPLETIME   5000              // an SQM sample is taken every 5s (cannot be less than 5s)
#define LDRCutoff1      275               // boundary between black and dark, used to determine Gate time
#define LDRCutoff2      500               // boundary between dark and light

// I2C addresses
#define BME280_I2CADDR  0x76

// Put in comments if TSL237 is not connected. If it's not connected
// the FreqCounter does not work in function light()
#define TSL237 1

// YOU MUST CHOOSE ONE OF THE FOLLOWING METHODS TO CALCULATE THE SQM VALUE
//#define OLDSQMMETHOD 1
//#define NEWSQMMETHOD 2
//#define NEWSQMMETHODCORRECTED 3
#define IRRADIANCEMETHOD 4

// do not change
#ifdef OLDSQMMETHOD
#define sqm_limit 21.83                 // mag limit for earth is 21.95
#endif
#ifdef NEWSQMMETHOD
#define sqm_limit 23.09                 // mag limit for earth is 21.95
#endif
#ifdef NEWSQMMETHODCORRECTED
//#define sqm_limit 21.95               // mag limit for earth is 21.95
#define sqm_limit 20                    // mostaccurate within 18-22 mpsas
#endif
#ifdef IRRADIANCEMETHOD
#define sqm_limit 21.95                 // mag limit for earth is 21.95
#endif


// Declare sensors
// Barometric sensor (Pressure, Temperature, Humidity)
bme280 mybme280;

// LUX Sensor
SFE_TSL2561 myTSL2561;

// IR Sensor, used as a cloud sensor, values help define sky state - cloudy, partly cloudy, clear
Adafruit_MLX90614 mymlx90614 = Adafruit_MLX90614();

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);
     
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

uint32_t timer = millis();

// Declaration for display
int               page;
int               maxpage = 3;

// Declaration for sensor readings
long              sensorsampletimer;      // used to determine when a sample sensor reading is taken
bool              sensorsamplerequest;    // determines which sensors to read
long              sqmsampletimer;         // used to determine when a sample sqm reading is taken
long              curTime;

int               setpoint1;              // setpoint values used to determine sky state
int               setpoint2;
int               skystate;
double            bme280humidity;
double            bme280temperature;
unsigned long int bme280pressure;
float             dewpoint;
float             mlx90614ambient;
float             mlx90614object;
double            lux;

float             mySQMreading;            // the SQM value, sky magnitude
float             frequency;               // measured TSL237 frequency which is dependent on light
double            irradiance;
double            nelm;

// LDR GL5528
int               LDRpin = A0;             // Light Dependant Resistor is on A0
short             period;

void setup()
{
  // declare button pins for input reading
  pinMode(pagepushButton, INPUT);
  pinMode(nightpushButton, INPUT);

  Wire.begin(); // Initiate the Wire library
  
  Serial.begin(9600);
  Serial.println("SQM Meter GPS basic test!");
     
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
     
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

  // TFT 1.8 Setup
  // Use this initializer if you're using a 1.8" TFT
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab

  tft.setTextWrap(false);
  tft.fillScreen(ST7735_BLACK);
  tft.setRotation(1);
  tft.setCursor(0, 0);
  tft.setTextColor(ST7735_WHITE);
  tft.setTextSize(1);

  page = 0;

  // Default values for readings
  mySQMreading = 0.0;            // the SQM value, sky magnitude
  frequency = 1.0;               // measured TLS237 frequency which is dependent on light
  irradiance = 1.0;
  nelm = 1.0;
  setpoint1 = 22;                // setpoint values used to determine sky state
  setpoint2 = 2;
  sensorsamplerequest = false;
  dewpoint = 10.0;
  bme280temperature = 20.0;
  bme280humidity = 50.0;
  bme280pressure = 1100;
  mlx90614ambient = 20.0;
  mlx90614object = 20.0;
  lux = 1000;
  skystate = SKYCLEAR;

  // Initialize sensors
  mybme280.begin(BME280_I2CADDR);
  mybme280.init();
}

// calculates dew point
// input:   humidity [%RH], temperature in C
// output:  dew point in C
void calc_dewpoint(float t, float h)
{
  float logEx;
  logEx = 0.66077 + 7.5 * t / (237.3 + t) + (log10(h) - 2);
  dewpoint = (logEx - 0.66077) * 237.3 / (0.66077 + 7.5 - logEx);
}

// calculate LUX reading from TSL2561
void getlux()
{
  GPS.pause(true);
  boolean gain;     // Gain setting, 0 = X1, 1 = X16;
  unsigned int ms;  // Integration ("shutter") time in milliseconds

  // If gain = false (0), device is set to low gain (1X)
  // If gain = high (1), device is set to high gain (16X)
  gain = 0;

  // If time = 0, integration will be 13.7ms
  // If time = 1, integration will be 101ms
  // If time = 2, integration will be 402ms
  // If time = 3, use manual start / stop to perform your own integration

  unsigned char time = 2;

  myTSL2561.begin();
  
  // setTiming() will set the third parameter (ms) to the
  // requested integration time in ms (this will be useful later):
  
  Serial.println("Set timing...");
  myTSL2561.setTiming(gain,time,ms);

  // To start taking measurements, power up the sensor:
  
  Serial.println("Powerup...");
  myTSL2561.setPowerUp();

  // Wait between measurements before retrieving the result
  // (You can also configure the sensor to issue an interrupt
  // when measurements are complete)
  
  // This sketch uses the TSL2561's built-in integration timer.
  // You can also perform your own manual integration timing
  // by setting "time" to 3 (manual) in setTiming(),
  // then performing a manualStart() and a manualStop() as in the below
  // commented statements:
  
  // ms = 1000;
  // light.manualStart();
  delay(ms);
  // light.manualStop();
  
  // Once integration is complete, we'll retrieve the data.
  
  // There are two light sensors on the device, one for visible light
  // and one for infrared. Both sensors are needed for lux calculations.
  
  // Retrieve the data from the device:

  unsigned int data0, data1;
  
  if (myTSL2561.getData(data0,data1))
  {
    // getData() returned true, communication was successful
    
    Serial.print("data0: ");
    Serial.print(data0);
    Serial.print(" data1: ");
    Serial.print(data1);
  
    // To calculate lux, pass all your settings and readings
    // to the getLux() function.
    
    // The getLux() function will return 1 if the calculation
    // was successful, or 0 if one or both of the sensors was
    // saturated (too much light). If this happens, you can
    // reduce the integration time and/or gain.
    // For more information see the hookup guide at: https://learn.sparkfun.com/tutorials/getting-started-with-the-tsl2561-luminosity-sensor
  
    // Perform lux calculation:

    myTSL2561.getLux(gain,ms,data0,data1,lux);

    // Print out the results:
  
    Serial.print(" lux: ");
    Serial.println(lux);
  }
  GPS.pause(false);
}

void getskystate()
{
  GPS.pause(true);

  float TempDiff = mlx90614ambient - mlx90614object;
  // object temp is IR temp of sky which at night time will be a lot less than ambient temp
  // so TempDiff is basically ambient + abs(object)
  // setpoint 1 is set for clear skies
  // setpoint 2 is set for cloudy skies
  // setpoint2 should be lower than setpoint1
  // For clear, Object will be very low, so TempDiff is largest
  // For cloudy, Object is closer to ambient, so TempDiff will be lowest

  // Readings are only valid at night when dark and sensor is pointed to sky
  // During the day readings are meaningless
  if ( TempDiff > setpoint1 )
    skystate = SKYCLEAR;          // clear
  else if ( (TempDiff > setpoint2) && (TempDiff < setpoint1) )
    skystate = SKYPCLOUDY;        // partly cloudy
  else if (TempDiff < setpoint2)
    skystate = SKYCLOUDY;         // cloudy
  else
    skystate = SKYUNKNOWN;        // unknown

  GPS.pause(false);
}

void light()
{
  long  pulses = 1L;

  GPS.pause(true);
  Serial.println("Start light() ...");

  readTSL237();
  
  GPS.pause(false);
}

void getButton() {
  int pagebuttonPressed;
  int nightbuttonPressed;

  // get button if page should be turned manually
  pagebuttonPressed = digitalRead(pagepushButton);
  if (pagebuttonPressed == HIGH && pageprevious == LOW) {
    delay(10); // stop reading for 10ms
    if (pagebuttonPressed == HIGH) {
      page = page++;
      pagebuttonPushed = true;
      if (page > maxpage) {
        page = 1;
      }
    }
  }
  pageprevious = pagebuttonPressed;

  // get button if night mode has to be switched on
  nightbuttonPressed = digitalRead(nightpushButton);
  if (nightbuttonPressed == HIGH && nightprevious == LOW) {
    delay(10); // stop reading for 10ms
    if (nightbuttonPressed == HIGH) {
      if (nightmode == true) {
        Serial.println("Night Mode Off");
        tft.setTextColor(ST7735_WHITE);
        nightmode = false;
        pagebuttonPushed = true;
        page = page - 1;
        if (page == 0) {
          page = 3; 
        }
      } else {
        Serial.println("Night Mode On");
        tft.setTextColor(ST7735_RED);
        nightmode = true;
        pagebuttonPushed = true;
        page = page - 1;
        if (page == 0) {
          page = 3; 
        }
      }
    }
  }
  nightprevious = nightbuttonPressed;
}

void readTSL237() {
  int retval;
  
  Wire.beginTransmission(8);
  Wire.write(period & 0xff);
  Wire.write(period >> 8);
  retval = Wire.endTransmission(true);

  // Error handling for I2C communication
  switch ( retval ) {
    case 0:
      Serial.print(F("Err Code: "));
      Serial.println(F("0 - success"));
      break;
    case 1:
      Serial.print(F("Err Code: "));
      Serial.println(F("1 - busy timeout upon entering endTransmission()"));
      break;
    case 2:
      Serial.print(F("Err Code: "));
      Serial.println(F("2 - START bit generation timeout"));
      break;
    case 3:
      Serial.print(F("Err Code: "));
      Serial.println(F("3 - end of address transmission timeout"));
      break;
    case 4:
      Serial.print(F("Err Code: "));
      Serial.println(F("4 - data byte transfer timeout"));
      break;
    case 5:
      Serial.print(F("Err Code: "));
      Serial.println(F("5 - data byte transfer succeeded, busy timeout immediately after"));
      break;
    default:
      Serial.print(F("Err Code: "));
      Serial.println(F("Undefined error"));
      break;
  }

  Wire.requestFrom(8,4); // read data from slave on address #8 with 4 bytes (datatype float)
  if ( Wire.available() ) {
    //frequency = Wire.read();
    //send value to I2C. Function splits according datatype value into bytes
    I2C_readAnything(frequency);

    Serial.print(F("Frequency: "));
    Serial.println(frequency);
  }
}

void loop() // run over and over again
{
  getButton();
  
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  // refresh sensor readings
  curTime = millis();
  if ( ((curTime - sensorsampletimer ) > SENSORSAMPLETIME ) || (curTime < sensorsampletimer) )
  {
    sensorsampletimer = curTime;
    if ( sensorsamplerequest == false )
    {
      // update bmesensor
      mybme280.readData();
      mybme280.getValues(&bme280temperature, &bme280humidity, &bme280pressure);
      calc_dewpoint((float) bme280temperature, (float) bme280humidity);

      // update lux
      getlux();
      sensorsamplerequest = true;
     }
    else
    {
      // update MLX90614
      mlx90614ambient = mymlx90614.readAmbientTempC();
      mlx90614object = mymlx90614.readObjectTempC();
      
      // update skystate
      getskystate();
      sensorsamplerequest = false;
    }

    // refresh sqm readings
    curTime = millis();
    // take a sample every 5 seconds
    if ( ((curTime - sqmsampletimer ) > SQMSAMPLETIME ) || (curTime < sqmsampletimer) )
    {
      sqmsampletimer = curTime;
      int LDRval = analogRead( LDRpin );        // read LDR to determine background light level
      Serial.print("LDRval = ");
      Serial.println(LDRval);
      if ( LDRval < LDRCutoff1 )                // and use this to set the Gate Time (duration) for this samples frequency measurement
      {
        period = blackperiod;                   // its very dark
      }
      else if ( LDRval < LDRCutoff2 )
      {
        period = darkperiod;                    // its dark
      }
      else
        period = lightperiod;                   // its light

      #ifdef TSL237
      Serial.println("Call light() ...");
        light();                                  // read frequency from sensor
      #endif
      
      irradiance = frequency / 2.3e3;           // calculate irradiance as uW/(cm^2)
      
      #ifdef OLDSQMMETHOD
          mySQMreading = ((sqm_limit - (2.5 * log10( (frequency * 1.584) / 2.0 )))) + 1.0; // frequency to magnitudes/arcSecond^2
      #endif
      
      #ifdef NEWSQMMETHOD
          mySQMreading = sqm_limit - 2.5 * log10 (frequency);
      #endif
      
      #ifdef NEWSQMMETHODCORRECTED
          // 0.973 was derived by comparing TLS237 sensor readings against Unihedron and plotting on graph then deriving coefficient
          mySQMreading = (sqm_limit - (2.5 * log10( frequency )) * 0.973);                   // frequency to magnitudes/arcSecond2
      #endif
      
      #ifdef IRRADIANCEMETHOD
          mySQMreading = log10((irradiance / 6.83) / 108000) / -0.4;                        // mag/arcsec^2
      #endif
  
      nelm = 7.93 - 5.0 * log10((pow(10, (4.316 - (mySQMreading / 5.0))) + 1));
    }
  }
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis()) timer = millis();
     
  // approximately every 5 seconds or so, print out the current stats
  if (millis() - timer > 5000 || pagebuttonPushed == true) {
    timer = millis(); // reset the timer
    // reset boolean indicator for button
    pagebuttonPushed = false;
    switch (page) {
      case 0:
        // Splash screen
        Serial.println("Showing Splash screen");
        tft.fillScreen(ST7735_BLACK);
        tft.setCursor(0 , 0);
        tft.setTextSize(2);
        tft.println(programName);
        tft.setTextSize(1);
        tft.print("Ver. ");
        tft.println(programVersion);
        tft.println();
        tft.println(copyright);
        tft.println();
        tft.setTextWrap(true);
        tft.println(programInfo1);
        tft.println();
        tft.println(programInfo2);
        tft.setTextWrap(false);
        page = 1;
        break;
      case 1:
        // GPS Date and Time
        //GPS.pause(true);
        Serial.println("Showing GPS Date and Time");
        tft.fillScreen(ST7735_BLACK);
        tft.setCursor(0, 0);
        tft.println("GPS Data Reading");
        tft.println("---------------------");
        tft.print("Date (UTC) : ");
        if (GPS.day < 10) tft.print('0');
        tft.print(GPS.day, DEC); tft.print('.');
        if (GPS.month < 10) tft.print('0');
        tft.print(GPS.month, DEC); tft.print(".");
        tft.print("20");
        tft.println(GPS.year, DEC);
        
        tft.print("Time (UTC) : ");
        if (GPS.hour < 10) tft.print('0');
        tft.print(GPS.hour, DEC); tft.print(':');
        if (GPS.minute < 10) tft.print('0');
        tft.print(GPS.minute, DEC); tft.print(':');
        if (GPS.seconds < 10) tft.print('0');
        tft.println(GPS.seconds, DEC);
        utc = tmConvert_t((GPS.year + 2000),GPS.month,GPS.day,GPS.hour,GPS.minute,GPS.seconds);
        ltime = CE.toLocal(utc, &tcr);
        sprintf(localTime, "%.02d:%.02d:%.02d", hour(ltime), minute(ltime), second(ltime));
        sprintf(localDate, "%.02d.%02d.%d", day(ltime), month(ltime), year(ltime));
        tft.println("---------------------");
        tft.print("Date (Loc) : ");
        tft.println(localDate);
        tft.print("Time (Loc) : ");
        tft.println(localTime);

        // GPS Coordinates
        if (GPS.fix) {
          tft.println("---------------------");
          tft.print("Lat. : ");
          if ((GPS.latitude / 100) < 10) tft.print('0');
          if ((GPS.latitude / 100) < 100) tft.print('0');
          tft.print(GPS.latitude / 100, 4);
          
          if ( GPS.lat == 'N' )
            tft.println(" N");
          else
            tft.println(" S");
            
          tft.print("Lon. : ");
          if ((GPS.longitude / 100) < 10) tft.print('0');
          if ((GPS.longitude / 100) < 100) tft.print('0');
          tft.print(GPS.longitude / 100, 4);
          
          if ( GPS.lon == 'W' )
            tft.println(" W");
          else
            tft.println(" E");
          
          // satellites
          tft.print("Sat. : ");
          tft.println( GPS.satellites );
          tft.print("Alt. : ");
          tft.print((int)GPS.altitude);
          tft.println(" m");
        } else {
          tft.println("---------------------");
          tft.println("No GPS fix ...");
        }
        tft.setCursor(140, 120);
        tft.print(page);
        tft.print("/");
        tft.print(maxpage);
        page = 2;
        break;
      case 2:
        // Ambient reading
        Serial.println("Showing Ambient reading");
        tft.fillScreen(ST7735_BLACK);
        tft.setCursor(0, 0);
        tft.println("Ambient Reading");
        tft.println("---------------------");
        tft.print("NELM    : ");
        tft.print(nelm);
        tft.println(" mag");
        // ir sensor
        tft.print("IR Temp.: ");
        tft.print(mlx90614ambient);
        tft.println(" C");
        tft.print("IR Obj. : ");
        tft.print(mlx90614object);
        tft.println(" C");
        // LUX
        //memset(tempstr, 0, buf_size);
        tft.print("Illum.  : ");
        tft.print(lux);
        tft.println(" lx");
        tft.println("---------------------");
        tft.print("Skystate: ");
        switch ( skystate )
          {
            case SKYCLEAR:   tft.println("CLEAR"); break;
            case SKYPCLOUDY: tft.println("PARTLY CLOUDY"); break;
            case SKYCLOUDY:  tft.println("CLOUDY"); break;
            default:         tft.println("UNKNOWN"); break;
          }
        // bme280 data, ambient, humidity, dewpoint, barometric pressure
        tft.println("---------------------");
        tft.print("Humid.  : ");
        tft.print(bme280humidity);
        tft.println(" %");
        tft.print("Temp.   : ");
        tft.print(bme280temperature);
        tft.println(" C");
        tft.print("Dewpoint: ");
        tft.print(dewpoint);
        tft.println(" C");
        tft.print("Pressure: ");
        tft.print(bme280pressure);
        tft.println(" hPa");

        tft.setCursor(140, 120);
        tft.print(page);
        tft.print("/");
        tft.print(maxpage);
        page = 3;
        break;
      case 3:
        // SQM calculation
        Serial.println("Showing SQM calculation");
        tft.fillScreen(ST7735_BLACK);
        tft.setCursor(0, 0);
        tft.setTextSize(2);
        tft.println("SQM");
        tft.setTextSize(1);
        tft.println("---------------------");
        tft.setTextSize(2);
        tft.println(mySQMreading);
        tft.setTextSize(1);
        tft.println();
        tft.print("Bortle : ");
        tft.print(getBortle(nelm));

        tft.setCursor(140, 120);
        tft.print(page);
        tft.print("/");
        tft.print(maxpage);
        page = 1;
        break;
    }
  }
}
