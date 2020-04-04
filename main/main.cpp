//include
#include <Arduino.h> //Arduino

#include <HardwareSerial.h> //Fingerprint sensor(R307)
#include <FPM.h>

#include <SPI.h> //OLED(SSD1331)
#include <SSD_13XX.h>

#include "esp32-hal-ledc.h" //Servo(SG92R)

#include <Wire.h> //RTC(RTC-8564NB)

#include <WiFi.h> //WiFi
#include "esp_wpa2.h"

/*#include <stdint.h> //BLE
#include <string.h>

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"*/

//initialization
HardwareSerial fserial(1); //FPM
FPM finger(&fserial);
FPM_System_Params params;

SSD_13XX tft = SSD_13XX(15, 16, 4); //SSD_13XX(SDA: GPIO23, SCL: GPIO18, CS: GPIO15, DC: GPIO16, RS: GPIO4)

int search_database(void) { //FPM
    int16_t p = -1;

    /* first get the finger image */
    Serial.println("Waiting for valid finger");
    finger.getImage();
    //vTaskDelay(10);

    /* convert it */
     finger.image2Tz();

    /* search the database for the converted print */
    uint16_t fid, score;
    p = finger.searchDatabase(&fid, &score);
    
    /* now wait to remove the finger, though not necessary; 
       this was moved here after the search because of the R503 sensor, 
       which seems to wipe its buffers after each scan */
    //Serial.println("Remove finger");
    
    if (p == FPM_OK) {
        Serial.println("Found a print match!");
    } else if (p == FPM_PACKETRECIEVEERR) {
        Serial.println("Communication error");
        return p;
    } else if (p == FPM_NOTFOUND) {
        Serial.println("Did not find a match");
        return p;
    } else if (p == FPM_TIMEOUT) {
        Serial.println("Timeout!");
        return p;
    } else if (p == FPM_READ_ERROR) {
        Serial.println("Got wrong PID or length!");
        return p;
    } else {
        Serial.println("Unknown error");
        return p;
    }
    Serial.print("Found ID #"); Serial.print(fid);
    Serial.print(" with confidence of "); Serial.println(score);
    return fid;
}

int deg2pw(int deg, int bit) //esp32-hal-ledc
{
  double ms = ((double)deg - 90.0) * 0.95 / 90.0 + 1.45;
  return (int)(ms / 20.0 * pow(2, bit));
}

const char *ssid = ""; //WPA2_Personal
const char *password = "";
/*#define EAP_ID ""  //WPA2_Enterprise
#define EAP_USERNAME ""
#define EAP_PASSWORD ""*/

#define REG_ADDR_CONTROL1 0x00
#define REG_ADDR_CONTROL2 0x01
#define REG_ADDR_SECONDS 0x02
#define REG_ADDR_MINUTES 0x03
#define REG_ADDR_HOURS 0x04
#define REG_ADDR_DAYS 0x05
#define REG_ADDR_WEEKDAYS 0x06
#define REG_ADDR_MONTHS 0x07
#define REG_ADDR_YEARS 0x08
#define REG_ADDR_MINUTE_ALARM 0x09
#define REG_ADDR_HOUR_ALARM 0x0A
#define REG_ADDR_DAY_ALARM 0x0B
#define REG_ADDR_WEEKDAY_ALARM 0x0C
#define REG_ADDR_CLOCKOUT_FREQ 0x0D
#define REG_ADDR_TIMER_CONTROL 0x0E
#define REG_ADDR_TIMER 0x0F

#define STOP_BIT 5                  // CONTROL1
#define INTERRUPT_PERIODIC 4        // CONTROL2
#define ALARM_FLAG 3                // CONTROL2
#define TIMER_FLAG 2                // CONTROL2
#define ALARM_INTERRUPT_ENABLE 1    // CONTROL2
#define TIMER_INTERRUPT_ENABLE 0    // CONTROL2
#define VOLTAGE_LOW 7               // SECONDS
#define ALARM_ENABLE 7              // MIN ALARAM - WEEKDAY ALARAM
#define CLOCK_OUT_ENABLE 7          // CLKOUT
#define CLOCK_OUT_FREQ_32768Hz 0x00 // CLKOUT
#define CLOCK_OUT_FREQ_1024Hz 0x01  // CLKOUT
#define CLOCK_OUT_FREQ_32Hz 0x02    // CLKOUT
#define CLOCK_OUT_FREQ_1Hz 0x03     // CLKOUT
#define TIMER_ENABLE 7              // TIMER CONTROL
#define TIMER_CLOCK_4096Hz 0        // TIMER CONTROL
#define TIMER_CLOCK_64Hz 1          // TIMER CONTROL
#define TIMER_CLOCK_1Hz 2           // TIMER CONTROL
#define TIMER_CLOCK_1_60Hz 3        // TIMER CONTROL

#define MINUTES_MASK 0b01111111
#define HOURS_MASK 0b00111111
#define DAYS_MASK 0b00111111
#define WEEKDAYS_MASK 0b00000111
#define MONTHS_MASK 0b00011111

#define RTC8564_ADDR 0x51 // I2C 7bit address

typedef struct
{
  int year;
  byte month;
  byte day;
  byte hour;
  byte min;
  byte sec;
  byte weekday;
} RTC8564_TIME;

void rtc8564_get_time(RTC8564_TIME *tm);
void rtc8564_sprintf(char *buf, RTC8564_TIME tm);

byte rtc8564_dec2bcd(byte data);
byte rtc8564_bcd2dec(byte data);
void rtc8564_clock_out_disable(void);
void rtc8564_write_byte(byte addr, byte data);
byte rtc8564_read_byte(byte addr);
boolean rtc8564_test_bit(byte addr, byte bit_position);
void rtc8564_set_bit(byte addr, byte bit_position);
void rtc8564_clear_bit(byte addr, byte bit_position);
void rtc8564_init(int year, byte month, byte day, byte hour, byte minute, byte second);
void rtc8564_set_alarm_day(byte day);
void rtc8564_set_alarm_hour(byte hour);
void rtc8564_set_alarm_minute(byte minute);
void rtc8564_alarm_interrupt_disable(void);
void rtc8564_alarm_disable(void);
void rtc8564_alarm_clear(void);
boolean rtc8564_alarm_test(void);
void rtc8564_timer_disable(void);
void rtc8564_timer_clear(void);
void rtc8564_set_time(int year, byte month, byte day, byte hour, byte minute, byte second);
char *rtc8564_calc_weekday(int year, int month, int day);
void setup();
void loop();

byte rtc8564_dec2bcd(byte data)
{
  return (((data / 10) << 4) + (data % 10));
}

byte rtc8564_bcd2dec(byte data)
{
  return (((data >> 4) * 10) + (data % 16));
}

void rtc8564_clock_out_disable(void)
{
  rtc8564_clear_bit(REG_ADDR_CLOCKOUT_FREQ, CLOCK_OUT_ENABLE);
}

void rtc8564_write_byte(byte addr, byte data)
{
  Wire.beginTransmission(RTC8564_ADDR);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission();
}

byte rtc8564_read_byte(byte addr)
{
  Wire.beginTransmission(RTC8564_ADDR);
  Wire.write(addr);
  Wire.endTransmission();
  Wire.requestFrom(RTC8564_ADDR, 1);

  return Wire.read();
}

boolean rtc8564_test_bit(byte addr, byte bit_position)
{
  byte data;

  data = rtc8564_read_byte(addr);
  data &= (0x01 << bit_position);
  if (data == 0x00)
  {
    return false;
  }
  else
  {
    return true;
  }
}

void rtc8564_set_bit(byte addr, byte bit_position)
{
  byte data;

  data = rtc8564_read_byte(addr);
  data |= (0x01 << bit_position);
  rtc8564_write_byte(addr, data);
}

void rtc8564_clear_bit(byte addr, byte bit_position)
{
  byte data;

  data = rtc8564_read_byte(addr);
  data &= ~(0x01 << bit_position);
  rtc8564_write_byte(addr, data);
}

void rtc8564_init(
    int year,
    byte month,
    byte day,
    byte hour,
    byte minute,
    byte second)
{
  rtc8564_write_byte(REG_ADDR_CONTROL1, 0x20);
  rtc8564_write_byte(REG_ADDR_CONTROL2, 0x00);
  rtc8564_set_time(year, month, day, hour, minute, second);
  rtc8564_alarm_disable();
  rtc8564_clock_out_disable();
  rtc8564_write_byte(REG_ADDR_TIMER_CONTROL, 0x00);
  rtc8564_write_byte(REG_ADDR_TIMER, 0x00);
  rtc8564_clear_bit(REG_ADDR_CONTROL1, STOP_BIT);
}

void rtc8564_set_alarm_day(byte day)
{
  rtc8564_write_byte(REG_ADDR_DAY_ALARM, rtc8564_dec2bcd(day));
}

void rtc8564_set_alarm_hour(byte hour)
{
  rtc8564_write_byte(REG_ADDR_HOUR_ALARM, rtc8564_dec2bcd(hour));
}

void rtc8564_set_alarm_minute(byte minute)
{
  rtc8564_write_byte(REG_ADDR_MINUTE_ALARM, rtc8564_dec2bcd(minute));
}

void rtc8564_alarm_interrupt_disable(void)
{
  rtc8564_clear_bit(REG_ADDR_CONTROL2, ALARM_INTERRUPT_ENABLE);
  rtc8564_alarm_disable();
}

void rtc8564_alarm_disable(void)
{
  rtc8564_set_bit(REG_ADDR_DAY_ALARM, ALARM_ENABLE);
  rtc8564_set_bit(REG_ADDR_HOUR_ALARM, ALARM_ENABLE);
  rtc8564_set_bit(REG_ADDR_MINUTE_ALARM, ALARM_ENABLE);
  rtc8564_set_bit(REG_ADDR_WEEKDAY_ALARM, ALARM_ENABLE);
  rtc8564_alarm_clear();
}

void rtc8564_alarm_clear(void)
{
  rtc8564_clear_bit(REG_ADDR_CONTROL2, ALARM_FLAG);
}

boolean rtc8564_alarm_test(void)
{
  if (rtc8564_test_bit(REG_ADDR_CONTROL2, ALARM_FLAG))
  {
    return true;
  }
  else
  {
    return false;
  }
}

void rtc8564_timer_disable(void)
{
  rtc8564_clear_bit(REG_ADDR_TIMER_CONTROL, TIMER_ENABLE);
}

void rtc8564_timer_clear(void)
{
  rtc8564_clear_bit(REG_ADDR_CONTROL2, TIMER_FLAG);
}

void rtc8564_set_time(
    int year,
    byte month,
    byte day,
    byte hour,
    byte minute,
    byte second)
{
  rtc8564_write_byte(REG_ADDR_SECONDS, rtc8564_dec2bcd(second));
  rtc8564_write_byte(REG_ADDR_MINUTES, rtc8564_dec2bcd(minute));
  rtc8564_write_byte(REG_ADDR_HOURS, rtc8564_dec2bcd(hour));
  rtc8564_write_byte(REG_ADDR_DAYS, rtc8564_dec2bcd(day));
  //rtc8564_write_byte(REG_ADDR_WEEKDAYS, rtc8564_calc_weekday(year,month,day));
  rtc8564_write_byte(REG_ADDR_MONTHS, rtc8564_dec2bcd(month));
  rtc8564_write_byte(REG_ADDR_YEARS, rtc8564_dec2bcd((byte)(year - 2000)));
}

void rtc8564_get_time(RTC8564_TIME *tm)
{
  tm->year = rtc8564_bcd2dec(rtc8564_read_byte(REG_ADDR_YEARS)) + 2000;
  tm->month = rtc8564_bcd2dec(rtc8564_read_byte(REG_ADDR_MONTHS) & MONTHS_MASK);
  tm->day = rtc8564_bcd2dec(rtc8564_read_byte(REG_ADDR_DAYS) & DAYS_MASK);
  tm->hour = rtc8564_bcd2dec(rtc8564_read_byte(REG_ADDR_HOURS) & HOURS_MASK);
  tm->min = rtc8564_bcd2dec(rtc8564_read_byte(REG_ADDR_MINUTES) & MINUTES_MASK);
  tm->sec = rtc8564_bcd2dec(rtc8564_read_byte(REG_ADDR_SECONDS));
  //tm->weekday = rtc8564_bcd2dec( rtc8564_read_byte( REG_ADDR_WEEKDAYS ));
}

char *rtc8564_calc_weekday( int year, int month, int day )
{
  if( month <= 2 ) {
    month += 12;
    year--;
  }
  
  int num;
  num = ((year + year/4 - year/100 + year/400 + ((13 * month + 8)/5) + day) % 7);

  if (num == 0) {
    return "Sun";
  } 
  else if (num == 1)
  {
    return "Mon";
  } 
  else if (num == 2)
  {
    return "Tue";
  } 
  else if (num == 3)
  {
    return "Wed";
  } 
  else if (num == 4)
  {
    return "Thu";
  } 
  else if (num == 5)
  {
    return "Fri";
  } 
  else if (num == 6) 
  {
    return "Sat";
  }
  else
  {
    return "Err";
  }
}

RTC8564_TIME rtc_time;
struct tm timeInfo;
char *weekday;
char date[17];   //YYYY/MM/DD WWW
char instant[9]; //HH:MM:SS
int fingerprint;
int ad;
int error;
int timeout;

void setup()
{
  //setting
  Serial.begin(115200); //Serial (MicroUSB)

  fserial.begin(57600, SERIAL_8N1, 5, 17); //FPM(RX: GPIO5, TX: GPIO17)

  if (finger.begin(1234))
  { //Password: 1234
    finger.readParams(&params);
    Serial.println("Found fingerprint sensor");
    Serial.print("Capacity: ");
    Serial.println(params.capacity);
    Serial.print("Packet length: ");
    Serial.println(FPM::packet_lengths[params.packet_len]);
    error = 0;
  }
  else
  {
    Serial.println("Did not find fingerprint sensor");
    error++;
  }

  ledcSetup(1, 50, 16); //esp32-hal-ledc(channel 1, 50 Hz, 16-bit depth)
  ledcAttachPin(19, 1); //esp32-hal-ledc(GPIO19)

  tft.begin(); //SSD_13XX

  Wire.begin(21, 22); //RTC(SDA: GPIO21, SCL: GPIO22)

  //wakeup or startup (起動か復帰か)
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();

  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0)
  {
    //wakeup
    Serial.println("wakeup");
  }
  else
  {
    //startup
    Serial.println("startup");
    Serial.println("WHS-Lock");
    Serial.println("v1.0-Alpha");

    tft.setInternalFont(); //load font
    tft.setTextScale(1); //set scale
    tft.setCursor(CENTER, 19);
    tft.println("WHS-Lock");
    tft.setCursor(CENTER, 35);
    tft.println("v1.0-Alpha");
    vTaskDelay(100);

    WiFi.disconnect(true);      
    /*esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)EAP_ID, strlen(EAP_ID)); //Enterprise magic
    esp_wifi_sta_wpa2_ent_set_username((uint8_t *)EAP_USERNAME, strlen(EAP_USERNAME));
    esp_wifi_sta_wpa2_ent_set_password((uint8_t *)EAP_PASSWORD, strlen(EAP_PASSWORD));
    esp_wifi_sta_wpa2_ent_enable();*/

    WiFi.begin(ssid, password); //WiFi on

    while (true)
    {
      Serial.print("Connecting to ");
      Serial.print(ssid);
      Serial.println("");

      tft.clearScreen(); //clear screen
      tft.setCursor(CENTER, 19);
      tft.println("Connecting to ");
      tft.setCursor(CENTER, 35);
      tft.print(ssid);
      tft.print("");
      vTaskDelay(25);

      Serial.print("Connecting to ");
      Serial.print(ssid);
      Serial.println(".");

      tft.clearScreen();
      tft.setCursor(CENTER, 19);
      tft.println("Connecting to ");
      tft.setCursor(CENTER, 35);
      tft.print(ssid);
      tft.print(".");
      vTaskDelay(25);

      Serial.print("Connecting to ");
      Serial.print(ssid);
      Serial.println("..");

      tft.clearScreen();
      tft.setCursor(CENTER, 19);
      tft.println("Connecting to ");
      tft.setCursor(CENTER, 35);
      tft.print(ssid);
      tft.print("..");
      vTaskDelay(25);

      Serial.print("Connecting to ");
      Serial.print(ssid);
      Serial.println("...");

      tft.clearScreen();
      tft.setCursor(CENTER, 19);
      tft.println("Connecting to ");
      tft.setCursor(CENTER, 35);
      tft.print(ssid);
      tft.print("...");
      vTaskDelay(25);

      timeout++;

      if (WiFi.status() == WL_CONNECTED) //success
      {
        Serial.println("WiFi connected");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());

        tft.clearScreen();
        tft.setCursor(CENTER, 15);
        tft.println("WiFi connected");
        tft.setCursor(CENTER, 31);
        tft.println("IP address: ");
        tft.setCursor(15, 39);
        tft.println(WiFi.localIP());
        vTaskDelay(100);

        configTime(9 * 3600L, 0, "ntp.nict.jp", "time.google.com", "ntp.jst.mfeed.ad.jp"); //setting
        getLocalTime(&timeInfo); //get time
        vTaskDelay(50); //adjustment
        rtc8564_init(timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday, timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec - 1); //set time

        Serial.println("Time was adjusted");

        tft.clearScreen();
        tft.setCursor(CENTER, 27);
        tft.println("Time was adjusted");
        vTaskDelay(100);
        break;
      }
      else if (timeout == 10) //failure(timeout 10s)
      {
        Serial.println("Failed to connect");
        Serial.println("Skipped");
        Serial.println("time adjustment");

        tft.clearScreen();
        tft.setCursor(CENTER, 15);
        tft.println("Failed to connect");
        tft.setCursor(CENTER, 31);
        tft.println("Skipped");
        tft.setCursor(CENTER, 39);
        tft.println("time adjustment");
        vTaskDelay(100);
        break;
      }
    }
    WiFi.disconnect(true); //WiFi off
    WiFi.mode(WIFI_OFF);

    rtc8564_set_alarm_hour(0); //set the alarm at 0:00
    rtc8564_set_alarm_minute(0);

    timeout = 0;

    if (error > 0)
    {
      tft.clearScreen();
      tft.setInternalFont(); //load font
      tft.setTextScale(1);
      tft.setCursor(CENTER, 23);
      tft.println("Did not find");
      tft.setCursor(CENTER, 31);
      tft.println("fingerprint sensor");
      vTaskDelay(1000);
    }
  }
  rtc8564_get_time(&rtc_time);
  weekday = rtc8564_calc_weekday(rtc_time.year, rtc_time.month, rtc_time.day); //set weekday(曜日)

  Serial.println("Ended setup");
}

void loop()
{
  rtc8564_get_time(&rtc_time); //get time

  sprintf(date, "%04u/%02u/%02u %s", rtc_time.year, rtc_time.month, rtc_time.day, weekday);
  sprintf(instant, "%02u:%02u:%02u", rtc_time.hour, rtc_time.min, rtc_time.sec);

  Serial.println(date);
  Serial.println(instant);

  tft.clearScreen();
  tft.setInternalFont();
  tft.setTextScale(1);
  tft.setTextColor(WHITE);
  tft.setCursor(CENTER, 15);
  tft.println(date);
  tft.setTextScale(2);
  tft.setCursor(CENTER, 31);
  tft.println(instant);

  fingerprint = search_database(); //read fingerprint
  
  ad = analogRead(33); //ADC(PIN: GPIO33)

  timeout++;

  vTaskDelay(10);

  if (ad > 3000) //RTC detect
  {
    timeout = 0;
    }

  if (fingerprint == 0) //fingerprint found
  {
    ledcWrite(1, deg2pw(60, 16));

    Serial.println("Unlocked"); //unlocked

    tft.clearScreen();
    tft.setTextScale(2);
    tft.setCursor(CENTER, CENTER);
    tft.setTextColor(GREEN);
    tft.println("Unlocked");
    vTaskDelay(100);
  } 
  else if (fingerprint == 1)
  {
    ledcWrite(1, deg2pw(90, 16)); //locked

    Serial.println("Locked");
    
    tft.clearScreen();
    tft.setTextScale(2);
    tft.setCursor(CENTER, CENTER);
    tft.setTextColor(RED);
    tft.println("Locked");
    vTaskDelay(100);
  }
  
  if (timeout == 60) //deep sleep(timeout 60s)
  {
    Serial.println("deep sleep");

    tft.clearScreen();

    ledcWrite(1, deg2pw(90, 16)); //locked

    esp_sleep_enable_ext0_wakeup(GPIO_NUM_33,1); //wakeup by PIR
    esp_deep_sleep_start();
  }

  if(rtc8564_alarm_test()) //maintenance
  {
    Serial.println("Maintenance");

    tft.clearScreen();
    tft.setTextScale(1);
    tft.setCursor(CENTER, CENTER);
    tft.setTextColor(WHITE);
    tft.println("Maintenance");
    vTaskDelay(100);

    rtc8564_alarm_clear();

    while (true) //wait until 0:01
    {
        rtc8564_get_time(&rtc_time);
        vTaskDelay(100);
        if (rtc_time.hour != 0 || rtc_time.min != 0)
        {
          break;
        }
    }

    ESP.restart(); //restart
  }
}