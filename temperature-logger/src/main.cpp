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
*  Description         : Temperature logger using arduino uno.
                        Components : 
                        1. Temperature Sensor : DS18B20
                        2. LCD : 16 x 2 Alphanumeric LCD
                        3. SD card
                        4. RTC
                        
                        Features : 
                        1. Loggs temperature data into SD card at the interval 
                        specefied by LOGGER_INTERVAL_DATA_LOG macro defined.
*
*  Change history      : 
*
*     Author        Date          Ver                 Description
*  ------------    --------       ---   --------------------------------------
*  Riken Maharjan  19 Feb 2023    1.0               Initial Creation
*  Lomas Subedi    20 Feb 2023    1.1               Organized
*  
*******************************************************************************/

/*******************************************************************************
*                          Include Files
*******************************************************************************/
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal.h>
#include <RTClib.h>
#include <MemoryFree.h>
#include <ArduinoJson.h>
/*******************************************************************************
*                          Type & Macro Definitions
*******************************************************************************/
#define LOGGER_GPIO_SYS_LED             8
#define LOGGER_ONE_WIRE_BUS             9 // Data wire is plugged into port 2 on the Arduino
#define LOGGER_GPIO_SPI_CS              10

#define LOGGER_GPIO_LCD_PIN_RS          7
#define LOGGER_GPIO_LCD_PIN_EN          6
#define LOGGER_GPIO_LCD_PIN_D4          5
#define LOGGER_GPIO_LCD_PIN_D5          4
#define LOGGER_GPIO_LCD_PIN_D6          3
#define LOGGER_GPIO_LCD_PIN_D7          2


#define LOGGER_LOG_FILENAME             "log.csv"
#define LOGGER_INTERVAL_DATA_LOG        10000
#define LOGGER_SERIAL_BAUD              115200
#define LOGGER_TMP_THRESHOLD            8.00f
#define LOGGER_SENSOR_RESOLUTION        12
#define LOGGER_ERROR_LCD_DELAY          2000
#define LOGGER_SIZE_RTC_STRING          150

#define LOGGER_LCD_SIZE_STRING          (uint8_t)(16)
#define LOGGER_LCD_SIZE_DATE            (uint8_t)(8)
#define LOGGER_LCD_SIZE_TIME            (uint8_t)(9)
#define LOGGER_LCD_SYMBOL_DEGREE        (uint8_t)(0xDF)

#define SERIAL_DEBUG

enum logger_error {
  LOGGER_ERROR_NONE,
  LOGGER_ERROR_RTC,
  LOGGER_ERROR_SD_ACCESS = 2,
  LOGGER_ERROR_FILE_ACCESS = 4,
  LOGGER_ERROR_SENSOR = 8
};

/**
 * Time and Temperature data structure
*/
struct logger_data_rtc_temp {
  uint16_t data_rtc_year;
  uint8_t data_rtc_month;
  uint8_t data_rtc_day;
  uint8_t data_rtc_hour;
  uint8_t data_rtc_minute;
  uint8_t data_rtc_second;
  float data_sensor_temperature_c;
};

/*******************************************************************************
*                          Static Data Definitions
*******************************************************************************/
// Setup a logger_one_wire instance to communicate with any 
// OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire logger_one_wire(LOGGER_ONE_WIRE_BUS);

// Pass our logger_one_wire reference to Dallas Temperature. 
DallasTemperature logger_sensor_temp_ds18b20(&logger_one_wire);

DeviceAddress logger_sensor_temp_address;


LiquidCrystal logger_lcd_16x2(LOGGER_GPIO_LCD_PIN_RS, 
                              LOGGER_GPIO_LCD_PIN_EN, 
                              LOGGER_GPIO_LCD_PIN_D4, 
                              LOGGER_GPIO_LCD_PIN_D5, 
                              LOGGER_GPIO_LCD_PIN_D6, 
                              LOGGER_GPIO_LCD_PIN_D7);

File logger_log_file_handle;

RTC_DS1307 logger_rtc_ds1307;

enum logger_error logger_error_global;

struct logger_data_rtc_temp logger;

StaticJsonDocument<LOGGER_SIZE_RTC_STRING> logger_rtc_json_buffer;

uint32_t logger_log_time_prev = 0;

/*******************************************************************************
*                          Static Function Definitions
*******************************************************************************/
void logger_lcd_set_cursor_first_line(void) {
    logger_lcd_16x2.setCursor(0, 0);
}

void logger_lcd_set_cursor_second_line(void) {
    logger_lcd_16x2.setCursor(0, 1);
}

void logger_sysled_set() {
  digitalWrite(LOGGER_GPIO_SYS_LED, HIGH);
}

void logger_sysled_clear() {
  digitalWrite(LOGGER_GPIO_SYS_LED, LOW);
}

void logger_error_clear() {
  logger_error_global = LOGGER_ERROR_NONE;  
}

/**
 * @brief Write string to LCD 
 * 
 * @param msg String to write
 */
void logger_lcd_write_string(const char * msg) {
  
  logger_lcd_16x2.clear();
  uint8_t len = strlen(msg);
  uint8_t numChunks = len / LOGGER_LCD_SIZE_STRING;
  uint8_t remainder = len % LOGGER_LCD_SIZE_STRING;
  char chunk[LOGGER_LCD_SIZE_STRING + 1];
  int startIndex;
  for (int i = 0; i < numChunks; i++) {
    startIndex = i * LOGGER_LCD_SIZE_STRING;
    strncpy(chunk, msg + startIndex, LOGGER_LCD_SIZE_STRING);
    chunk[16] = '\0';
    logger_lcd_16x2.setCursor(0, i);
    logger_lcd_16x2.print(chunk);
  }
  if (remainder > 0) {
    startIndex = numChunks * LOGGER_LCD_SIZE_STRING;
    strncpy(chunk, msg + startIndex, remainder);
    chunk[remainder] = '\0';
    logger_lcd_16x2.setCursor(0, numChunks);
    logger_lcd_16x2.print(chunk);
  }  

  delay(1000);
}

/**
 * @brief Write temperate and date time data to LCD
 * 
 * @param d logger data handle
 */
void logger_lcd_write_data(struct logger_data_rtc_temp d) {
  
  char lcd_string[LOGGER_LCD_SIZE_STRING];
  
  logger_lcd_16x2.clear();
  
  logger_lcd_set_cursor_first_line();
  
  // Print Temperature data
  logger_lcd_16x2.print("TEMP: ");
  logger_lcd_16x2.print(d.data_sensor_temperature_c);
  logger_lcd_16x2.print((char)LOGGER_LCD_SYMBOL_DEGREE);
  logger_lcd_16x2.print("C");

  logger_lcd_set_cursor_second_line();

  snprintf(lcd_string,LOGGER_LCD_SIZE_DATE,"%d/%d,", d.data_rtc_month, d.data_rtc_day);
  logger_lcd_16x2.print(lcd_string);

  logger_lcd_16x2.setCursor(7, 1);
  snprintf(lcd_string,LOGGER_LCD_SIZE_TIME,"%d:%d:%d", d.data_rtc_hour, d.data_rtc_minute, d.data_rtc_second);
  logger_lcd_16x2.print(lcd_string);
}

/**
 * @brief Print error to the LCD
 * 
 * @param e : Error object
 */
void logger_lcd_print_error(enum logger_error e) { 
  
  
  if(!e) {
    return;
  }

  logger_lcd_16x2.clear();
  
  logger_lcd_set_cursor_first_line();
  logger_lcd_16x2.print("ERROR : ");
  logger_lcd_16x2.setCursor(0, 11);


  if(e & LOGGER_ERROR_RTC) {
    logger_lcd_16x2.print(LOGGER_ERROR_RTC, DEC);
    logger_lcd_set_cursor_second_line();
    logger_lcd_16x2.print("RTC");
    delay(LOGGER_ERROR_LCD_DELAY);
  }

  if(e & LOGGER_ERROR_FILE_ACCESS) {
    logger_lcd_16x2.print(LOGGER_ERROR_FILE_ACCESS, DEC);
    logger_lcd_set_cursor_second_line();
    logger_lcd_16x2.print("File Access");
    delay(LOGGER_ERROR_LCD_DELAY);
  }

  if(e & LOGGER_ERROR_SD_ACCESS) {
    logger_lcd_16x2.print(LOGGER_ERROR_SD_ACCESS, DEC);
    logger_lcd_set_cursor_second_line();
    logger_lcd_16x2.print("SD Card");
    delay(LOGGER_ERROR_LCD_DELAY);
  }

  if(e & LOGGER_ERROR_SENSOR) {
    logger_lcd_16x2.print(LOGGER_ERROR_SENSOR, DEC);
    logger_lcd_set_cursor_second_line();
    logger_lcd_16x2.print("Sensor");
    delay(LOGGER_ERROR_LCD_DELAY);
  }      
}

/**
 * @brief get date and time data from RTC
 * 
 * @param d pointer logger data
 */
void logger_get_rtc_data(struct logger_data_rtc_temp *d) {
  
  DateTime NOW = logger_rtc_ds1307.now();

  d->data_rtc_year = NOW.year();
  d->data_rtc_month = NOW.month();
  d->data_rtc_day = NOW.day();
  d->data_rtc_hour = NOW.hour();
  d->data_rtc_minute = NOW.minute();
  d->data_rtc_second = NOW.second();

}

/**
 * @brief function to get the temperature for a device
 * 
 * @param deviceAddress : Address of the temperature sensor
 * @return float temperature data
 */
float logger_get_temperature(DeviceAddress deviceAddress) {
  float tempC = logger_sensor_temp_ds18b20.getTempC(deviceAddress);
  if(tempC == DEVICE_DISCONNECTED) {
    logger_error_global |= LOGGER_ERROR_SENSOR;
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

/**
 * @brief Append csv file content with passed data
 * 
 * @param d logger data
 */
void logger_file_append(struct logger_data_rtc_temp d) {

  #ifdef SERIAL_DEBUG
  Serial.println(F("Writing to SD card..."));
  #endif
  // write Date - timestamp
  logger_log_file_handle.print(d.data_rtc_year, DEC);
  logger_log_file_handle.print('-');
  logger_log_file_handle.print(d.data_rtc_month, DEC);
  logger_log_file_handle.print('-');
  logger_log_file_handle.print(d.data_rtc_day, DEC);
  logger_log_file_handle.print(',');// delimiter 

  // write Time - timestamp
  logger_log_file_handle.print(d.data_rtc_hour, DEC);
  logger_log_file_handle.print(':');
  logger_log_file_handle.print(d.data_rtc_minute, DEC);
  logger_log_file_handle.print(':');
  logger_log_file_handle.print(d.data_rtc_second, DEC);
  logger_log_file_handle.print(","); // delimiter 

  // write Temperature Data    
  logger_log_file_handle.print(d.data_sensor_temperature_c,5);
  logger_log_file_handle.write("\n"); // new line
}

/**
 * @brief update RTC data from the data which is recieved from serial
 *        communication
 * 
 * @param d logger data
 */
void logger_rtc_update(struct logger_data_rtc_temp d) {

  DateTime date_time = DateTime(d.data_rtc_year, 
                                d.data_rtc_month, 
                                d.data_rtc_day, 
                                d.data_rtc_hour,
                                d.data_rtc_minute, 
                                d.data_rtc_second);

  logger_rtc_ds1307.adjust(date_time);

}

/**
 * @brief function to print a device address
 * 
 * @param deviceAddress address of temperature sensor
 */
void logger_sensor_printaddr(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}
//-----------------------------------------------------------------------------------------

void setup(void) {
  
  logger_error_global = LOGGER_ERROR_NONE;

  pinMode(LOGGER_GPIO_SYS_LED, OUTPUT); // System Led is taken as OUTPUT
  
  #ifdef SERIAL_DEBUG
  Serial.begin(LOGGER_SERIAL_BAUD);
  #endif

  // set up the LCD's number of columns and rows:
  logger_lcd_16x2.begin(LOGGER_LCD_SIZE_STRING, 2);

  logger_lcd_16x2.clear();  
  logger_lcd_write_string("Nepal Digital Systems Pvt. Ltd.");
  delay(5000);
  logger_lcd_write_string("Phone :         9841784514");
  delay(5000);
  
  if (!logger_rtc_ds1307.begin())  {
    #ifdef SERIAL_DEBUG
    Serial.println(F("Couldn't find RTC!\r\n Please reset device !"));    
    #endif

    logger_lcd_write_string("Couldn't find RTC!Please reset device!");
    logger_error_global |= LOGGER_ERROR_RTC;
  }
      
  if (!logger_rtc_ds1307.isrunning()) {
    #ifdef SERIAL_DEBUG
    Serial.println(F("RTC is NOT running, setting new time !"));
    #endif

    logger_lcd_write_string("RTC NOT running!, setting new time!");

    logger_rtc_ds1307.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // locate sensor
    #ifdef SERIAL_DEBUG
    Serial.print(F("Locating Temperature sensor..."));
    #endif

    logger_lcd_write_string("Locating Temperature sensor...");

    logger_sensor_temp_ds18b20.begin();


    #ifdef SERIAL_DEBUG
    Serial.print(F("Found "));
    Serial.print(logger_sensor_temp_ds18b20.getDeviceCount(), DEC);
    Serial.println(F(" devices."));
    #endif

    // report parasite power requirements
    #ifdef SERIAL_DEBUG
    Serial.print(F("Parasite power is: "));     
    if (logger_sensor_temp_ds18b20.isParasitePowerMode()) Serial.println(F("ON"));
    else Serial.println(F("OFF"));
    #endif
    
    #ifdef SERIAL_DEBUG
    if (!logger_sensor_temp_ds18b20.getAddress(logger_sensor_temp_address, 0)) Serial.println(F("Unable to find address for Device 0")); 
    // show the addresses we found on the bus
    Serial.print(F("Device 0 Address: "));
    #endif
    logger_sensor_printaddr(logger_sensor_temp_address);
    #ifdef SERIAL_DEBUG
    Serial.println();
    #endif
    // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
    logger_sensor_temp_ds18b20.setResolution(logger_sensor_temp_address, LOGGER_SENSOR_RESOLUTION);
    #ifdef SERIAL_DEBUG
    Serial.print(F("Device 0 Resolution: "));
    Serial.print(logger_sensor_temp_ds18b20.getResolution(logger_sensor_temp_address), DEC); 
    Serial.println();
    #endif
  // Initialize SD card
    #ifdef SERIAL_DEBUG
    Serial.print(F("Initializing SD card..."));    
    #endif
    logger_lcd_write_string("Initializing SD card...");
    
  if (!SD.begin(LOGGER_GPIO_SPI_CS))  {
    
    logger_lcd_write_string("...NO SDCARD...");

    #ifdef SERIAL_DEBUG
    Serial.println(F("Card failed, or not present"));
    #endif

    logger_lcd_write_string("Card failed, or not present");
    
    logger_error_global |= LOGGER_ERROR_SD_ACCESS;
  }

  #ifdef SERIAL_DEBUG
  Serial.println(F("card initialized."));  
  #endif

  logger_lcd_write_string("card initialized.");

  logger_lcd_write_string("Init .... done");  
  
  #ifdef SERIAL_DEBUG
  Serial.flush();
  #endif

  if(logger_error_global) {
    logger_lcd_print_error(logger_error_global);
    logger_error_clear();
    return;
  }
}


void loop(void) {

  logger_get_rtc_data(&logger);
  
  // call logger_sensor_temp_ds18b20.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
  logger_sensor_temp_ds18b20.requestTemperatures(); // Send the command to get temperatures

  // It responds almost immediately. Let's print out the data
  logger.data_sensor_temperature_c = logger_get_temperature(logger_sensor_temp_address); // Use a simple function to print out the data

  // show in LCD display
  logger_lcd_write_data(logger);

  if(logger.data_sensor_temperature_c >= LOGGER_TMP_THRESHOLD) {
    logger_sysled_set();
  } else {
    logger_sysled_clear();
  }

  if((millis() - logger_log_time_prev) >= LOGGER_INTERVAL_DATA_LOG) {
    
    logger_log_time_prev = millis();

    logger_log_file_handle = SD.open(LOGGER_LOG_FILENAME, FILE_WRITE);
    
    if (logger_log_file_handle) {
      logger_file_append(logger);
      logger_log_file_handle.close();
    }  else  {
      logger_error_global |= LOGGER_ERROR_FILE_ACCESS;
    }
  }

  logger_lcd_print_error(logger_error_global);
  logger_error_clear();  
  
  #ifdef SERIAL_DEBUG
  Serial.print(F("Free RAM ="));
  Serial.println(freeMemory());
  #endif
}
/*******************************************************************************
*                          End of File
*******************************************************************************/