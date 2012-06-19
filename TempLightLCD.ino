// include the library code:
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal.h>
#include <Wire.h> 
#include <Format.h>
#include "TSL2561.h"
#include <EEPROM.h>
#include <SimpleTimer.h>

// Setup the luminosity sensor
TSL2561 tsl(TSL2561_ADDR_FLOAT);

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// The address of the Dallas Temperature sensor
DeviceAddress sensorAddress;

#define REDLITE 3
#define GREENLITE 9
#define BLUELITE 10

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(7, 8, 5, 6, 11, 12);

//#define DEBUG

#define NVCHECKSUM 0x44DB606B2340420BULL
#define NVVERSION 2
#define NVOFFSET 5
#define NVFUNCTIONS 2

uint64_t nvchecksum; 
uint8_t nvversion;
uint8_t nvfunctions;

struct LCDSettings
{
  // the colors to use for the lcd
  byte red;
  byte green;
  byte blue;
  // you can change the overall brightness by range 0 -> 255
  byte brightness;
};

struct PrintSettings
{
  boolean required[NVFUNCTIONS];
};

struct ConfigSettings
{  
  LCDSettings lcd;
  PrintSettings print;
};

timer_callback functions[NVFUNCTIONS] = {
  printTemperatures,
  printLight
};

ConfigSettings settings;

//use the first custom character slot
byte symbol = 0;

//create the degree symbol
byte degree[8] = {
B01100,
B10010,
B10010,
B01100,
B00000,
B00000,
B00000,
B00000
};

SimpleTimer timer;

void setup() {
  Serial.begin(9600);
  
  // send the custom degree symbol to the lcd
  lcd.createChar(symbol, degree);
  // set up the LCD's number of rows and columns: 
  lcd.begin(16, 2);  
  
  initConfig();
  
  setBacklight();
  
  // start the luminosity sensor
  tsl.begin();
  // set the gain and timing
  tsl.setGain(TSL2561_GAIN_16X);
  tsl.setTiming(TSL2561_INTEGRATIONTIME_13MS);
  
  // start the Dallas temperature sensor
  sensors.begin();
  // get the address of the sensor at index 0
  sensors.getAddress(sensorAddress, 0);
  //set the resolution of the temperature sensor
  sensors.setResolution(sensorAddress, 12);

  //print the temperatures immediately
  loopPrint();
  
  timer.setInterval(1000, readCommand);
  timer.setInterval(5000, loopPrint);
}

void loop() {
  timer.run();
}

void loopPrint()
{
  static int i = 0;
  if (settings.print.required[i])
  {
    (*functions[i])();
  }
  
  i++;
  
  for(i; i < NVFUNCTIONS; i++)
  {
    if(settings.print.required[i])
    {
      break;
    }
  }
  
  if (i >= NVFUNCTIONS)
    i = 0;  
}

void readCommand()
{
  char cmd;
  
  if (readArgument(&cmd))
  {
    switch(cmd)
    {
      case 'B':
      case 'b':
        readBacklightCmd();
        break;
      case 'P':
      case 'p':
        readPrintCmd();
        break;
      case 'C':
      case 'c':
        readConfigCmd();
        break;
      case 'S':
      case 's':
        readSensorCmd();
        break;
    }
    
    //read any extra characters
    while(Serial.available() > 0)
      Serial.read();
  }
}
boolean readInteger(int * arg)
{
  if (Serial.available() > 0)
  {
    while(Serial.peek() == ' ')
      Serial.read();
      
    int num = Serial.parseInt();
    
    *arg = num;
    
    return true;
  }
  
  return false;
}

boolean readArgument(char* arg)
{
  if (Serial.available() > 0)
  {
    while(Serial.peek() == ' ')
      Serial.read();
    
    char cmd = Serial.read();
    
    *arg = cmd;
    
    return true;
  }
  
  return false;
}

void readConfigCmd()
{
  char cmd;
  int bytes;
  if (readArgument(&cmd))
  {
    switch(cmd)
    {
      case 'R':
      case 'r':
        readConfig(&bytes);
        break;
      case 'W':
      case 'w':
        writeConfig(&bytes);
        break;
    }
  }
#ifdef DEBUG
  Serial.println("tbytes");
  Serial.println(bytes);
  Serial.println("");
#endif
}

void readSensorCmd()
{
  char cmd;
  
  if (readArgument(&cmd))
  {
    switch(cmd)
    {
      case 'L':
      case 'l':
        readLightSensorCmd();
        break;
    }
  }
}

void readLightSensorCmd()
{
  char cmd;
  
  if (readArgument(&cmd))
  {
    switch(cmd)
    {
      case 'I':
      case 'i':
        changeIntegrationTime();
        break;
      case 'G':
      case 'g':
        changeGain();
        break;
    }
  }
}

void readPrintCmd()
{
  char cmd;
  if (readArgument(&cmd))
  {
    switch(cmd)
    {
      case 'T':
      case 't':
        tooglePrint(0);
        break;
      case 'L':
      case 'l':
        tooglePrint(1);
        break;
    }
  }
}

void tooglePrint(int i)
{
  settings.print.required[i] = !settings.print.required[i];
}

void readBacklightCmd()
{
  char cmd;
  if (readArgument(&cmd))
  {
    Serial.read();

    byte value = Serial.parseInt();
    
    switch(cmd)
    {
      case 'R':
      case 'r':
        settings.lcd.red = value;
        break;
      case 'G':
      case 'g':
        settings.lcd.green = value;
        break;
      case 'B':
      case 'b':
        settings.lcd.blue = value;
        break;
      case 'O':
      case 'o':
        settings.lcd.brightness = value;
        break;
    }

    setBacklight();
  }
}

void changeIntegrationTime()
{
  int value;
  
  if (readInteger(&value))
  {
    switch(value)
    {
      case 1:
        tsl.setTiming(TSL2561_INTEGRATIONTIME_13MS);
        break;
      case 2:
        tsl.setTiming(TSL2561_INTEGRATIONTIME_101MS);
        break;
      case 3:
        tsl.setTiming(TSL2561_INTEGRATIONTIME_402MS);
        break;
    }
  }
}

void changeGain()
{
  int value;
  
  if (readInteger(&value))
  {
    switch(value)
    {
      case 0:
        tsl.setGain(TSL2561_GAIN_0X);
        break;
      case 1:
        tsl.setGain(TSL2561_GAIN_16X);
        break;
    }
  }
}

boolean initConfig()
{
  int bytes1 = -1;
  int bytes2 = -1;
  if (!readConfig(&bytes1))
  {
    // turn on the backlight at half brightness
    settings.lcd.brightness = 128;
    settings.lcd.red = 255;
    settings.lcd.green = 255;
    settings.lcd.blue = 255;
   
    settings.print.required[0] = true;
    for(int i = 1; i < NVFUNCTIONS; i++)
    {
      settings.print.required[i] = false;
    }
    
    nvchecksum = NVCHECKSUM;
    nvversion = NVVERSION;
    nvfunctions = NVFUNCTIONS;
    writeConfig(&bytes2);
  }
#ifdef DEBUG
  Serial.println("tbytes");
  Serial.println(bytes1);
  Serial.println("");
  Serial.println("tbytes");
  Serial.println(bytes2);
  Serial.println("");
#endif
}

boolean readConfig(int * bytes)
{
  int offset = 0;
  *bytes = 0;
  *bytes += readEEPROM(&nvversion, offset, sizeof(nvversion));
  offset += sizeof(nvversion);
  *bytes += readEEPROM(&nvfunctions, offset, sizeof(nvfunctions));
  offset += sizeof(nvfunctions);
  *bytes += readEEPROM((byte *)&nvchecksum, offset, sizeof(nvchecksum));
  offset += sizeof(nvchecksum);
  if (nvchecksum == NVCHECKSUM && nvversion == NVVERSION && nvfunctions == NVFUNCTIONS)
  {
    *bytes += readEEPROM((byte *)&settings, offset, sizeof(settings));
    return true;
  }
  else
  {
    return false;
  }
}

void writeConfig(int * bytes)
{
  int offset = 0;
  *bytes = 0;
  *bytes += writeEEPROM(&nvversion, offset, sizeof(nvversion));
  offset += sizeof(nvversion);
  
  *bytes += writeEEPROM(&nvfunctions, offset, sizeof(nvfunctions));
  offset += sizeof(nvfunctions);
  
  *bytes += writeEEPROM((byte *)&nvchecksum, offset, sizeof(nvchecksum));
  offset += sizeof(nvchecksum);
  
  *bytes += writeEEPROM((byte *)&settings, offset, sizeof(settings));
}

int readEEPROM(byte * value, int offset, int len)
{
  int bytes = 0;
#ifdef DEBUG
  Serial.println("read");
  Serial.println(len);
  Serial.println("values");
#endif
  for (int i = 0; i < len; i++)
  {
    value[i] = EEPROM.read(NVOFFSET + i + offset);
#ifdef DEBUG
    Serial.println(value[i]);
#endif
    bytes++;
  }
#ifdef DEBUG  
  Serial.println("bytes");
  Serial.println(bytes);
  Serial.println("");
#endif
  return bytes;
}

int writeEEPROM(byte * value, int offset, int len)
{
  int bytes = 0;
#ifdef DEBUG
  Serial.println("write");
  Serial.println(len);
  Serial.println("values");
#endif
  for (int i = 0; i < len; i++)
  {
#ifdef DEBUG
    Serial.println(value[i]);
#endif
    EEPROM.write(NVOFFSET + i + offset, value[i]);
    bytes++;
  }
#ifdef DEBUG
  Serial.println("bytes");
  Serial.println(bytes);
  Serial.println("");
#endif
  return bytes;
}

void printLight()
{
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;
  
  lcd.clear();
  lcd.print("IR ");
  lcd.print(ir);
  lcd.setCursor(8, 0);
  
  lcd.print("VI ");
  lcd.print(full - ir);

  lcd.setCursor(0, 1);
  lcd.print("FL ");
  lcd.print(full);
  
  lcd.setCursor(8, 1);
  lcd.print("LX ");
  lcd.print(tsl.calculateLux(full, ir));
}

void printTemperatures()
{
  sensors.requestTemperatures();
  float ftemp_c = sensors.getTempC(sensorAddress);
  float ftemp_f = DallasTemperature::toFahrenheit(ftemp_c);
  String stemp_c;
  String stemp_f;
  int width;
  stemp_c = formatFloat(ftemp_c, 1, &width);
  stemp_f = formatFloat(ftemp_f, 1, &width);
  
  lcd.clear();
  lcd.print("DS18B20 Temp");
  lcd.setCursor(0, 1);
  lcd.print(stemp_c);
  lcd.write(symbol);
  lcd.print("C ");
  lcd.print(stemp_f);
  lcd.write(symbol);
  lcd.print("F");
}
 
void setBacklight() {
  uint8_t r, g, b, l;
  
  r = settings.lcd.red;
  g = settings.lcd.green;
  b = settings.lcd.blue;
  l = settings.lcd.brightness;
  
  // normalize the red LED - its brighter than the rest!
  r = map(r, 0, 255, 0, 100);
  g = map(g, 0, 255, 0, 150);
 
  r = map(r, 0, 255, 0, l);
  g = map(g, 0, 255, 0, l);
  b = map(b, 0, 255, 0, l);
 
  // common anode so invert!
  r = map(r, 0, 255, 255, 0);
  g = map(g, 0, 255, 255, 0);
  b = map(b, 0, 255, 255, 0);
  Serial.print("R = "); Serial.print(r, DEC);
  Serial.print(" G = "); Serial.print(g, DEC);
  Serial.print(" B = "); Serial.println(b, DEC);
  analogWrite(REDLITE, r);
  analogWrite(GREENLITE, g);
  analogWrite(BLUELITE, b);
}
