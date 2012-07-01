// include the library code:
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal.h>
#include <Wire.h> 
#include <Format.h>
#include "TSL2561.h"
#include <EEPROM.h>
#include <SimpleTimer.h>
#include <EasyTransferI2C.h>


#define COMMAND_COUNT 5
#define ARGUMENT_COUNT 3

struct CMD_DATA
{
  boolean cmd;
  boolean fields[COMMAND_COUNT + ARGUMENT_COUNT];
  char cmds[COMMAND_COUNT];
  int args[ARGUMENT_COUNT];
};

struct VALUE_DATA
{
  int full;
  int ir;
  int vis;
  int lux;
  float temp_c;
  float temp_f;
  int r;
  int g;
  int b;
};

VALUE_DATA txdata;
CMD_DATA rxdata;

EasyTransferI2C ETtx;
EasyTransferI2C ETrx;

#define I2C_RX_ADDRESS 10
#define I2C_TX_ADDRESS 9

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
#define STATUS_PIN 13
void setup() {
  pinMode(STATUS_PIN, OUTPUT);
  digitalWrite(STATUS_PIN, LOW);
  
  Wire.begin(I2C_RX_ADDRESS);
  
  ETtx.begin(details(txdata), &Wire);
  ETrx.begin(details(rxdata), &Wire);
  Wire.onReceive(rxdata_receive);
  
  txdata.full = 0;
  txdata.ir = 0;
  txdata.vis = 0;
  txdata.lux = 0;
  
  txdata.temp_c = 0.0;
  txdata.temp_f = 0.0;
  
  txdata.r = 0;
  txdata.g = 0;
  txdata.b = 0;
  
  rxdata.cmd = false;
  for(int i = 0; i < COMMAND_COUNT + ARGUMENT_COUNT; i++)
  {
    rxdata.fields[i] = false;
  }
  
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
   ETrx.receiveData();
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

void rxdata_receive(int numBytes) 
{
  static boolean value = false;
  
  value = !value;
  
  if (value)
    digitalWrite(STATUS_PIN, HIGH);
  else
    digitalWrite(STATUS_PIN, LOW);
}

void readCommand()
{  
  char cmd;
 
  if (rxdata.cmd && readArgument(&cmd))
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
    
    rxdata.cmd = false;
  }

}

boolean readIndex(int start, int end, int * index)
{
  for(int i = start; i < end; i++)
  {
    if(rxdata.fields[i])
    {
      rxdata.fields[i] = false;
      *index = i;
      return true;
    }
  }
  
  
  return false;
}

boolean readInteger(int * arg)
{
  int index = -1;

  if (rxdata.cmd && readIndex(COMMAND_COUNT + 1,COMMAND_COUNT + ARGUMENT_COUNT, &index))
  {
    *arg = rxdata.args[index - (COMMAND_COUNT + 1)];
    return true;
  }
  
  return false;
}

boolean readArgument(char* arg)
{
  int index = -1;
  if(rxdata.cmd && readIndex(0, COMMAND_COUNT, &index))
  {
    *arg = rxdata.cmds[index];
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
  int value;
  if (readArgument(&cmd) && readInteger(&value))
  {
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
  for (int i = 0; i < len; i++)
  {
    value[i] = EEPROM.read(NVOFFSET + i + offset);
    bytes++;
  }
  return bytes;
}

int writeEEPROM(byte * value, int offset, int len)
{
  int bytes = 0;
  for (int i = 0; i < len; i++)
  {
    EEPROM.write(NVOFFSET + i + offset, value[i]);
    bytes++;
  }
  return bytes;
}

void printLight()
{
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full, lux;
  ir = lum >> 16;
  full = lum & 0xFFFF;
  lux = tsl.calculateLux(full, ir);
  
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
  lcd.print(lux);
  
  txdata.full = full;
  txdata.ir = ir;
  txdata.vis = full - ir;
  txdata.lux = lux;
  
  ETtx.sendData(I2C_TX_ADDRESS);
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
  
  txdata.temp_c = ftemp_c;
  txdata.temp_f = ftemp_f;
  
  ETtx.sendData(I2C_TX_ADDRESS);
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
  
  analogWrite(REDLITE, r);
  analogWrite(GREENLITE, g);
  analogWrite(BLUELITE, b);
  
  txdata.r = r;
  txdata.g = g;
  txdata.b = b;
  
  ETtx.sendData(I2C_TX_ADDRESS);
}
