/*******************************************************************************
* (C) Copyright 2018-2023 ;  Nepal Digital Systems Pvt. Ltd., Kathmandu, Nepal.
* The attached material and the information contained therein is proprietary to
* Nepal Digital Systems Pvt. Ltd. and is issued only under strict confidentiality
* arrangements.It shall not be used, reproduced, copied in whole or in part,
* adapted,modified, or disseminated without a written license of Nepal Digital  
* Systems Pvt. Ltd.It must be returned to Nepal Digital Systems Pvt. Ltd. upon 
* its first request.
*
*  File Name           : temperature-logger.ino
*
*  Description         : It sample source file
*
*  Change history      : 
*
*     Author        Date          Ver                 Description
*  ------------    --------       ---   --------------------------------------
*  Riken Maharjan  19 Feb 2023    1.0               Initial Creation
*  
*******************************************************************************/

/*******************************************************************************
*                          Include Files
*******************************************************************************/
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <LiquidCrystal.h>
#include "RTClib.h"
#include <MemoryFree.h>

/*******************************************************************************
*                          Type & Macro Definitions
*******************************************************************************/
#define GPIO_SYS_LED        8
#define ONE_WIRE_BUS        9 // Data wire is plugged into port 2 on the Arduino

#define GPIO_LCD_PIN_RS     7
#define GPIO_LCD_PIN_EN     6
#define GPIO_LCD_PIN_D4     5
#define GPIO_LCD_PIN_D5     4
#define GPIO_LCD_PIN_D6     3
#define GPIO_LCD_PIN_D7     2

#define TEMP_LOGGER_LOG_FILENAME  "log.csv"

#define TEMP_LOGGER_LOG_INTERVAL    10000

#define SERIAL_DEBUG

/*******************************************************************************
*                          Static Data Definitions
*******************************************************************************/
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

DeviceAddress insideThermometer;

LiquidCrystal lcd(GPIO_LCD_PIN_RS, GPIO_LCD_PIN_EN, GPIO_LCD_PIN_D4, GPIO_LCD_PIN_D5, GPIO_LCD_PIN_D6, GPIO_LCD_PIN_D7);

File temp_logger_file;
RTC_DS1307 rtc;  

int yr,mo,dy,hr,mi,se;
float TempC; 

uint32_t time_now = 0;
uint32_t time_prev = 0;

/*******************************************************************************
*                          Static Function Definitions
*******************************************************************************/
// function to print in LCD
void LCD_print() {
  char b[16];
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("TEMP: ");
  lcd.print(TempC);
  lcd.print((char)0xDF);
  lcd.print("C");
  lcd.setCursor(0, 1);
  snprintf(b,8,"%d/%d,",mo,dy);
  lcd.print(b);
  lcd.setCursor(7, 1);
  snprintf(b,9,"%d:%d:%d",hr,mi,se);
  lcd.print(b);
}

// function to get the temperature for a device

float printTemperature(DeviceAddress deviceAddress) {
  float tempC = sensors.getTempC(deviceAddress);
  if(tempC == DEVICE_DISCONNECTED_C) {
    #ifdef SERIAL_DEBUG
    Serial.println(F("Error: Could not read temperature data"));
    #endif // End SERIAL_DEBUG
  }
  
  #ifdef SERIAL_DEBUG
  Serial.print(F("Temp C: "));
  Serial.println(tempC);
  #endif 
  return tempC;
}
// function to print a device address
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}
//-----------------------------------------------------------------------------------------

void setup(void) {
  pinMode(GPIO_SYS_LED, OUTPUT); // System Led is taken as OUTPUT
  #ifdef SERIAL_DEBUG
  Serial.begin(9600);
  #endif
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  #ifdef SERIAL_DEBUG
  Serial.println(F("------- Temperature LOGs -----"));
  #endif
  lcd.print("Temperature LOGs");
  // Initialize RTC
    #ifdef SERIAL_DEBUG
    Serial.println(F("Initializing RTC Module..."));
    #endif
    digitalWrite(GPIO_SYS_LED,HIGH);
    if (!rtc.begin())  {
      #ifdef SERIAL_DEBUG
      Serial.println(F("Couldn't find RTC"));
      #endif
      while (1);
    }
    digitalWrite(GPIO_SYS_LED,LOW);
  
    if (! rtc.isrunning()) {
      #ifdef SERIAL_DEBUG
      Serial.println(F("RTC is NOT running, let's set the time!"));
      #endif
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

  // locate sensor
    #ifdef SERIAL_DEBUG
    Serial.print(F("Locating Temperature sensor..."));
    #endif
    sensors.begin();
    #ifdef SERIAL_DEBUG
    Serial.print(F("Found "));
    Serial.print(sensors.getDeviceCount(), DEC);
    Serial.println(F(" devices."));
    #endif

    // report parasite power requirements
    #ifdef SERIAL_DEBUG
    Serial.print(F("Parasite power is: "));     
    if (sensors.isParasitePowerMode()) Serial.println(F("ON"));
    else Serial.println(F("OFF"));
    #endif
    
    #ifdef SERIAL_DEBUG
    if (!sensors.getAddress(insideThermometer, 0)) Serial.println(F("Unable to find address for Device 0")); 
    // show the addresses we found on the bus
    Serial.print(F("Device 0 Address: "));
    #endif
    printAddress(insideThermometer);
    #ifdef SERIAL_DEBUG
    Serial.println();
    #endif
    // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
    sensors.setResolution(insideThermometer, 9);
    #ifdef SERIAL_DEBUG
    Serial.print(F("Device 0 Resolution: "));
    Serial.print(sensors.getResolution(insideThermometer), DEC); 
    Serial.println();
    #endif
  // Initialize SD card
    #ifdef SERIAL_DEBUG
    Serial.print(F("Initializing SD card..."));
    #endif
    digitalWrite(GPIO_SYS_LED,HIGH);
    if (!SD.begin(GPIO_SPI_CS))  {
      lcd.setCursor(0, 1);
      lcd.print("...NO SDCARD...");
      #ifdef SERIAL_DEBUG
      Serial.println(F("Card failed, or not present"));
      #endif
      while (1);
    }
    digitalWrite(GPIO_SYS_LED,LOW);
  #ifdef SERIAL_DEBUG
  Serial.println(F("card initialized."));
  Serial.println(F(" -------------  Initialization done. ------------- "));
  #endif
  lcd.setCursor(0, 1);
  lcd.print("Init .... done");  
  #ifdef SERIAL_DEBUG
  Serial.flush();
  #endif
}


void loop(void) { 
  DateTime NOW = rtc.now();
  yr = NOW.year();
  mo = NOW.month();
  dy = NOW.day();
  hr = NOW.hour();
  mi = NOW.minute();
  se = NOW.second();
  
  // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
  #ifdef SERIAL_DEBUG
  Serial.print(F("Requesting temperatures..."));
  #endif

  sensors.requestTemperatures(); // Send the command to get temperatures
  #ifdef SERIAL_DEBUG
  Serial.println(F("DONE"));
  #endif
  // It responds almost immediately. Let's print out the data
  TempC = printTemperature(insideThermometer); // Use a simple function to print out the data

  // show in LCD display
  LCD_print();

  if((millis() - time_prev) >= TEMP_LOGGER_LOG_INTERVAL) {
    time_prev = millis();
    temp_logger_file = SD.open(TEMP_LOGGER_LOG_FILENAME, FILE_WRITE);
    // if the file opened okay, write to it:
    if (temp_logger_file) {
      digitalWrite(GPIO_SYS_LED,HIGH);
      #ifdef SERIAL_DEBUG
      Serial.println(F("Writing to SD card..."));
      #endif
      // write Date - timestamp
      temp_logger_file.print(yr, DEC);
      temp_logger_file.print('-');
      temp_logger_file.print(mo, DEC);
      temp_logger_file.print('-');
      temp_logger_file.print(dy, DEC);
      temp_logger_file.print(',');// delimiter 

      // write Time - timestamp
      temp_logger_file.print(hr, DEC);
      temp_logger_file.print(':');
      temp_logger_file.print(mi, DEC);
      temp_logger_file.print(':');
      temp_logger_file.print(se, DEC);
      temp_logger_file.print(","); // delimiter 
      
      // write Temperature Data    
      temp_logger_file.print(TempC,5);
      temp_logger_file.write("\n"); // new line
      // close the file:
      temp_logger_file.close();
      #ifdef SERIAL_DEBUG
      Serial.println(F("done....."));
      #endif
      digitalWrite(GPIO_SYS_LED,LOW);
    }  else  {
      digitalWrite(GPIO_SYS_LED,HIGH);
      #ifdef SERIAL_DEBUG
      // if the file didn't open, print an error:
      Serial.println(F("error opening SD card"));
      #endif
      lcd.setCursor(0, 1);
      lcd.print("ERROR IN SDCARD");
    }
  }
  #ifdef SERIAL_DEBUG
  Serial.print(F("freeMemory()="));
  Serial.println(freeMemory());
  #endif
}
/*******************************************************************************
*                          End of File
*******************************************************************************/