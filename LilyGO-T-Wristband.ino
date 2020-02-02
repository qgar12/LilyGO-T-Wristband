
#include <pcf8563.h>      // Library for NXP PCF8563 RTC chip
#include <TFT_eSPI.h>     // Graphics and font library for ST7735 driver chip
#include <SPI.h>          // Serial Peripheral Interface (SPI) 
#include <Wire.h>         // I2C library
#include <WiFi.h>
#include <MPU9250.h>      // IMU / MPU-9250 lib (see https://github.com/bolderflight/MPU9250)
//O#include "sensor.h"
#include "esp_adc_cal.h"
#include "ttgo.h"
#include "charge.h"

#define FACTORY_HW_TEST     //! Test RTC and WiFi scan when enabled
#define ARDUINO_OTA_UPDATE      //! Enable this line OTA update
//#define CALIBRATE_MAGNETOMETER //! calibrate magnemoter -> move in a 8



#ifdef ARDUINO_OTA_UPDATE
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#endif

const char *ssid = "xx";
const char *password = "xx";

#define TP_PIN_PIN          33
#define I2C_SDA_PIN         21
#define I2C_SCL_PIN         22
#define IMU_INT_PIN         39
#define RTC_INT_PIN         34
#define BATT_ADC_PIN        35
#define VBUS_PIN            36
#define TP_PWR_PIN          25
#define LED_PIN             4
#define CHARGE_PIN          32

// chips
MPU9250         IMU(Wire,0x69);
TFT_eSPI        tft = TFT_eSPI(); 
PCF8563_Class   rtc;

char buff[256];
bool rtcIrq = false;
bool initial = 1;
bool otaStart = false;

uint8_t func_select = 1;
uint8_t omm = 99;
uint8_t xcolon = 0;
uint32_t targetTime = 0;       // for next 1 second timeout
uint32_t colour = 0;
int vref = 1100;

bool pressed = false;
uint32_t pressedTime = 0;
bool charge_indication = false;

uint8_t hh, mm, ss ;

// Flag set to indicate MPU 9250 data is ready 
volatile bool imu_data_ready = false;

// TFT output
#define TFT_NB_OF_LINES 5
int tftCurrentLine = 0;
char tftBuffer[TFT_NB_OF_LINES+1][80]; // reserve one extra line for scrolling

void tfdClr() {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    tftCurrentLine=0;
 }

void tfdPrintLine(const char *buf, const int lineNb) {
  tft.drawString(buf, 0, lineNb * 16);
}

void tfdPrint(const char *buf) {
  if (tftCurrentLine >= TFT_NB_OF_LINES) {
    // scroll & print
    tfdClr();
    for (int ix=0; ix < TFT_NB_OF_LINES; ix++) {
      strncpy(tftBuffer[ix], tftBuffer[ix+1], sizeof(tftBuffer[ix]));
      tfdPrintLine(tftBuffer[ix], ix);
    } 
  }
  else {
    // print
    tfdPrintLine(tftBuffer[tftCurrentLine], tftCurrentLine);
    tftCurrentLine +=1;
  }
  
}

char* tfdGetBuffer() {
  return tftBuffer[tftCurrentLine];
}

#define DCLR() {tfdClr();}
//#define DPRINT(...) { char b = tfdGetBuffer(); snprintf(b, sizeof(b), __VA_ARGS__); tfdPrint(b); }
#define DPRINT(...) { char* b = tfdGetBuffer(); snprintf(tftBuffer[tftCurrentLine], sizeof(tftBuffer[tftCurrentLine]), __VA_ARGS__); tfdPrint(tftBuffer[tftCurrentLine]); }
#define DWAIT() {delay(2000);}

void scanI2Cdevice(void)
{
  DCLR();
  uint8_t err, addr;
  int nDevices = 0;
  for (addr = 1; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      err = Wire.endTransmission();
      if (err == 0) {
          Serial.print("I2C device found at address 0x");            
          if (addr < 16)
              Serial.print("0");
          Serial.print(addr, HEX);
          Serial.println(" !");
          nDevices++;  
          DPRINT("I2C device found at 0x%x", addr);
      } else if (err == 4) {
          Serial.print("Unknow error at address 0x");
          if (addr < 16)
              Serial.print("0");
          Serial.println(addr, HEX);
          DPRINT("Unknow error at 0x%x", addr);
      }
  }
  if (nDevices == 0)
      Serial.println("No I2C devices found\n");
  else
      Serial.println("Done\n");
  DWAIT();
}


void wifi_scan()
{
    DCLR();
    DPRINT("Scan Network:");

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    int16_t n = WiFi.scanNetworks();
    if (n == 0) {
        DPRINT("no networks found");
    } else {
        for (int i = 0; i < n; ++i) {
            DPRINT("[%d]:%s(%d)", i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i));
        }
    }
    WiFi.mode(WIFI_OFF);
    DWAIT();
}


void drawProgressBar(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint8_t percentage, uint16_t frameColor, uint16_t barColor)
{
    if (percentage == 0) {
        tft.fillRoundRect(x0, y0, w, h, 3, TFT_BLACK);
    }
    uint8_t margin = 2;
    uint16_t barHeight = h - 2 * margin;
    uint16_t barWidth = w - 2 * margin;
    tft.drawRoundRect(x0, y0, w, h, 3, frameColor);
    tft.fillRect(x0 + margin, y0 + margin, barWidth * percentage / 100.0, barHeight, barColor);
}

void rtcSelfTest() {
    DCLR();
    DPRINT("RTC Interrupt self test");

    int yy = 2019, mm = 5, dd = 15, h = 2, m = 10, s = 0;
    rtc.begin(Wire);
    rtc.setDateTime(yy, mm, dd, h, m, s);
    delay(500);
    RTC_Date dt = rtc.getDateTime();
    if (dt.year != yy || dt.month != mm || dt.day != dd || dt.hour != h || dt.minute != m) {
        DPRINT("Write DateTime FAIL");
    } else {
        DPRINT("Write DateTime PASS");
    }

     delay(2000);

    //! RTC Interrupt Test
    pinMode(RTC_INT_PIN, INPUT_PULLUP); //need change to rtc_pin
    attachInterrupt(RTC_INT_PIN, [] {
        rtcIrq = 1;
    }, FALLING);

    rtc.disableAlarm();

    rtc.setDateTime(2019, 4, 7, 9, 5, 57);

    rtc.setAlarmByMinutes(6);

    rtc.enableAlarm();

    for (;;) {
        DPRINT("%s", rtc.formatDateTime());
        if (rtcIrq) {
            rtcIrq = 0;
            detachInterrupt(RTC_INT_PIN);
            rtc.resetAlarm();
            break;
        }
        delay(1000);
    }
    DPRINT("RTC Interrupt PASS");
    
    DWAIT();
}

void factoryTest()
{
    scanI2Cdevice();
    rtcSelfTest();
    wifi_scan();
}

void setupWiFi()
{
#ifdef ARDUINO_OTA_UPDATE
    DCLR();
    DPRINT("Wifi setup");
    DPRINT("Connection to %s", ssid);
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
        DPRINT("Connection Failed! Rebooting...");
        delay(5000);
        ESP.restart();
    }

    DPRINT("IP address: %s", WiFi.localIP().toString().c_str());

    DWAIT();
       
#endif
}

void setupOTA()
{
#ifdef ARDUINO_OTA_UPDATE
    DCLR();
    DPRINT("OTA setup");
    
    // Port defaults to 3232
    // ArduinoOTA.setPort(3232);

    // Hostname defaults to esp3232-[MAC]
    ArduinoOTA.setHostname("T-Wristband");

    // No authentication by default
    // ArduinoOTA.setPassword("admin");

    // Password can be set with it's md5 value as well
    // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
    // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
            type = "sketch";
        else // U_SPIFFS
            type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
        otaStart = true;
        tft.fillScreen(TFT_BLACK);
        tft.drawString("Updating...", tft.width() / 2 - 20, 55 );
    })
    .onEnd([]() {
        Serial.println("\nEnd");
        delay(500);
    })
    .onProgress([](unsigned int progress, unsigned int total) {
        // Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
        int percentage = (progress / (total / 100));
        tft.setTextDatum(TC_DATUM);
        tft.setTextPadding(tft.textWidth(" 888% "));
        tft.drawString(String(percentage) + "%", 145, 35);
        drawProgressBar(10, 30, 120, 15, percentage, TFT_WHITE, TFT_BLUE);
    })
    .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");

        tft.fillScreen(TFT_BLACK);
        tft.drawString("Update Failed", tft.width() / 2 - 20, 55 );
        delay(3000);
        otaStart = false;
        initial = 1;
        targetTime = millis() + 1000;
        tft.fillScreen(TFT_BLACK);
        tft.setTextDatum(TL_DATUM);
        omm = 99;
    });

    ArduinoOTA.begin();

    DPRINT("..done");
    DWAIT();
#endif
}


void setupADC()
{
    esp_adc_cal_characteristics_t adc_chars;
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize((adc_unit_t)ADC_UNIT_1, (adc_atten_t)ADC1_CHANNEL_6, (adc_bits_width_t)ADC_WIDTH_BIT_12, 1100, &adc_chars);
    //Check type of calibration value used to characterize ADC
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        Serial.printf("eFuse Vref:%u mV", adc_chars.vref);
        vref = adc_chars.vref;
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        Serial.printf("Two Point --> coeff_a:%umV coeff_b:%umV\n", adc_chars.coeff_a, adc_chars.coeff_b);
    } else {
        Serial.println("Default Vref: 1100mV");
    }
}

void setupRTC()
{
    rtc.begin(Wire);
    //Check if the RTC clock matches, if not, use compile time
    rtc.check();

    RTC_Date datetime = rtc.getDateTime();
    hh = datetime.hour;
    mm = datetime.minute;
    ss = datetime.second;
}

String getVoltage()
{
    uint16_t v = analogRead(BATT_ADC_PIN);
    float battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
    return String(battery_voltage) + "V";
}

void RTC_Show()
{
    if (targetTime < millis()) {
        RTC_Date datetime = rtc.getDateTime();
        hh = datetime.hour;
        mm = datetime.minute;
        ss = datetime.second;
        // Serial.printf("hh:%d mm:%d ss:%d\n", hh, mm, ss);
        targetTime = millis() + 1000;
        if (ss == 0 || initial) {
            initial = 0;
            tft.setTextColor(TFT_GREEN, TFT_BLACK);
            tft.setCursor (8, 60);
            tft.print(__DATE__); // This uses the standard ADAFruit small font
        }

        tft.setTextColor(TFT_BLUE, TFT_BLACK);
        tft.drawCentreString(getVoltage(), 120, 60, 1); // Next size up font 2


        // Update digital time
        uint8_t xpos = 6;
        uint8_t ypos = 0;
        if (omm != mm) { // Only redraw every minute to minimise flicker
            // Uncomment ONE of the next 2 lines, using the ghost image demonstrates text overlay as time is drawn over it
            tft.setTextColor(0x39C4, TFT_BLACK);  // Leave a 7 segment ghost image, comment out next line!
            // tft.setTextColor(TFT_BLACK, TFT_BLACK); // Set font colour to black to wipe image
            // Font 7 is to show a pseudo 7 segment display.
            // Font 7 only contains characters [space] 0 1 2 3 4 5 6 7 8 9 0 : .
            tft.drawString("88:88", xpos, ypos, 7); // Overwrite the text to clear it
            tft.setTextColor(0xFBE0, TFT_BLACK); // Orange
            omm = mm;

            if (hh < 10) xpos += tft.drawChar('0', xpos, ypos, 7);
            xpos += tft.drawNumber(hh, xpos, ypos, 7);
            xcolon = xpos;
            xpos += tft.drawChar(':', xpos, ypos, 7);
            if (mm < 10) xpos += tft.drawChar('0', xpos, ypos, 7);
            tft.drawNumber(mm, xpos, ypos, 7);
        }

        if (ss % 2) { // Flash the colon
            tft.setTextColor(0x39C4, TFT_BLACK);
            xpos += tft.drawChar(':', xcolon, ypos, 7);
            tft.setTextColor(0xFBE0, TFT_BLACK);
        } else {
            tft.drawChar(':', xcolon, ypos, 7);
        }
    }
}

// ISR to set data ready flag */
void data_ready()
{
  imu_data_ready = true;
}

void IMU_Show()
{
  if (imu_data_ready) {
    imu_data_ready = false;
    
    DCLR();
    
    // read the sensor
    IMU.readSensor();
    
    DPRINT("--  ACC  GYR   MAG");
    tft.drawString(buff, 0, 0);
    DPRINT("x %+06.2f  %+06.2f  %+06.2f", IMU.getAccelX_mss(), IMU.getGyroX_rads(), IMU.getMagX_uT());
    tft.drawString(buff, 0, 16);
    DPRINT("y %+06.2f  %+06.2f  %+06.2f", IMU.getAccelY_mss(), IMU.getGyroY_rads(), IMU.getMagY_uT());
    tft.drawString(buff, 0, 32);
    DPRINT("z %+06.2f  %+06.2f  %+06.2f", IMU.getAccelZ_mss(), IMU.getGyroZ_rads(), IMU.getMagZ_uT());
    tft.drawString(buff, 0, 48);
    float heading = atan2(IMU.getMagY_uT(), IMU.getMagX_uT()) * 180 / PI;
    DPRINT("heading = %.2f", heading);

  }
} 

// START code from https://github.com/bolderflight/MPU9250/issues/33

//  accelerometer and magnetometer data 
float h, hx, hy, hz;
//  euler angles 
float yaw_rad, heading_rad;
//  filtered heading 
float filtered_heading_rad;
float window_size = 20;
// conversion radians to degrees 
const float R2D = 180.0f / PI;

void IMU_ShowHeading()
{
  if (imu_data_ready) {
    imu_data_ready = false;

    DCLR();
    
    /* Read the MPU 9250 data */
    IMU.readSensor();
    hx = IMU.getMagX_uT();
    hy = IMU.getMagY_uT();
    hz = IMU.getMagZ_uT();

    // Normalize magnetometer data 
    h = sqrtf(hx * hx + hy * hy + hz * hz);
    hx /= h;
    hy /= h;
    hz /= h; 
    // Compute euler angles 
    yaw_rad = atan2f(-hy, hx);
    heading_rad = constrainAngle360(yaw_rad);
    // Filtering heading 
    filtered_heading_rad = (filtered_heading_rad * (window_size - 1.0f) + heading_rad) / window_size;
    // Display the results 
    DPRINT("yaw_rad:      %.2f", yaw_rad * R2D);
    DPRINT("heading_rad:  %.2f", heading_rad * R2D);
    DPRINT("filtered_rad: %.2f", filtered_heading_rad * R2D); 
  }
} 

/* Bound angle between 0 and 360 */
float constrainAngle360(float dta) {
  dta = fmod(dta, 2.0 * PI);
  if (dta < 0.0)
    dta += 2.0 * PI;
  return dta;
}
// END code from https://github.com/bolderflight/MPU9250/issues/33

void sleep() 
{
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Press again to wake up",  tft.width() / 2, tft.height() / 2 );
//O  IMU.setSleepEnabled(true);
  Serial.println("Go to Sleep");
  delay(3000);
  
  // updated according https://github.com/Xinyuan-LilyGO/LilyGO-T-Wristband/issues/2
  tft.writecommand(ST7735_SWRESET);
  delay(100);
  tft.writecommand(ST7735_SLPIN);
  delay(150);
  tft.writecommand(ST7735_DISPOFF);
  
  esp_sleep_enable_ext1_wakeup(GPIO_SEL_33, ESP_EXT1_WAKEUP_ANY_HIGH);
  esp_deep_sleep_start();
}

void setupMpu9250() {
    // initialize IMU 
    DCLR();
    DPRINT("Initializing IMU / MPU9250");
    int status = IMU.begin();
    DPRINT("status = %i", status);
    if (status < 0) {
      DPRINT("... ERROR");
    }
    else {
      // setting the accelerometer full scale range to +/-8G 
      IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
      // setting the gyroscope full scale range to +/-500 deg/s
      IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
      // setting DLPF bandwidth to 20 Hz
      IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
      // setting SRD to 19 for a 50 Hz update rate
      IMU.setSrd(19);
      DPRINT("... done");
    }
    DWAIT();

#ifdef CALIBRATE_MAGNETOMETER
    // Calibrate magnetometer
    DCLR();
    DPRINT("Calibrating magnetometer,");
    DPRINT(".. please slowly move in a");
    DPRINT(".. figure 8 until complete...");
    IMU.calibrateMag();
    DPRINT("Done!");
    DWAIT(); 
#endif

    //  Attach the data ready interrupt to the data ready ISR
    IMU.enableDataReadyInterrupt();
    pinMode(IMU_INT_PIN, INPUT_PULLUP);
    attachInterrupt(IMU_INT_PIN, data_ready, RISING); 
}

void setup()
{
    Serial.begin(115200);

    tft.init();
    tft.setRotation(1);
    tft.setSwapBytes(true);
    tft.pushImage(0, 0,  160, 80, ttgo);

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);

#ifdef FACTORY_HW_TEST
    factoryTest();
#endif

    setupRTC();

    setupADC();

    setupWiFi();

    setupOTA();

    setupMpu9250();
    
    tft.fillScreen(TFT_BLACK);

    tft.setTextColor(TFT_YELLOW, TFT_BLACK); // Note: the new fonts do not draw the background colour

    targetTime = millis() + 1000;

    pinMode(TP_PIN_PIN, INPUT);
    //! Must be set to pull-up output mode in order to wake up in deep sleep mode
    pinMode(TP_PWR_PIN, PULLUP);
    digitalWrite(TP_PWR_PIN, HIGH);

    pinMode(LED_PIN, OUTPUT);

    pinMode(CHARGE_PIN, INPUT_PULLUP);
    attachInterrupt(CHARGE_PIN, [] {
        charge_indication = true;
    }, CHANGE);

    if (digitalRead(CHARGE_PIN) == LOW) {
        charge_indication = true;
    }
}


void loop()
{
#ifdef ARDUINO_OTA_UPDATE
    ArduinoOTA.handle();
#endif

    //! If OTA starts, skip the following operation
    if (otaStart)
        return;

    if (charge_indication) {
        charge_indication = false;
        if (digitalRead(CHARGE_PIN) == LOW) {
            tft.pushImage(140, 55, 34, 16, charge);
        } else {
            tft.fillRect(140, 55, 34, 16, TFT_BLACK);
        }
    }


    if (digitalRead(TP_PIN_PIN) == HIGH) {
        if (!pressed) {
            initial = 1;
            targetTime = millis() + 1000;
            tft.fillScreen(TFT_BLACK);
            omm = 99;
            func_select = func_select + 1 > 2 ? 0 : func_select + 1;
            digitalWrite(LED_PIN, HIGH);
            delay(100);
            digitalWrite(LED_PIN, LOW);
            pressed = true;
            pressedTime = millis();
        } else {
            if (millis() - pressedTime > 3000) {
 //             sleep();
            }
        }
    } else {
        pressed = false;
    }

    switch (func_select) {
    case 0:
        RTC_Show();
        break;
    case 1:
        IMU_Show();
        delay(200);
        break;
    case 2:
        IMU_ShowHeading();
        delay(200);
        break;
    default:
        break;
    }
}
