#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include "EEPROM.h"


#define TIME_TO_WAKEUP_HTTP 120        
const char* HOST = "http://192.168.1.50:8000"; /*хост куда слать данные*/
#define SLEEP_AFTER 60000

#define PINS_WAKE_BIT_MASK 0x300000000 //00000001100000000000000000000000000000000 32, 33 pins bit mask (count from right (0) to left (39))
#define PIN_32_MASK 0x100000000
#define PIN_33_MASK 0x200000000

#define HOT_ADDR 0
#define COLD_ADDR 4


//#define PINS_WAKE_BIT_MASK 0x0

#define uS_TO_S_FACTOR 1000000 /* коэффициент пересчета микросекунд в секунды */

WiFiMulti wifiMulti;
long coldCount = 0;
long hotCount = 0;
bool wasPressedHot = false;
bool wasPressedCold = false;
bool doSendHttp = false;
uint32_t actionTimer;


/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();
  Serial.print("wakeup_reason: ");
  Serial.println(wakeup_reason);

  if(wakeup_reason == ESP_SLEEP_WAKEUP_EXT1){
    uint64_t wakePin = esp_sleep_get_ext1_wakeup_status();
    if (wakePin == PIN_32_MASK) {
      triggeredHot();
    }
    if (wakePin == PIN_33_MASK) {
      triggeredCold();
    }
    Serial.print("esp_ext1_wakeup_status: ");
    Serial.println(wakePin);
  }

  if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
    doSendHttp = true;
  }

  if (wakeup_reason == 0) {//on reset
      Serial.println("Going to sleep in setup. ");
      esp_deep_sleep_start();
  }

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}


void setup(){
  Serial.begin(115200);
  delay(1000); //Take some time to open up the Serial Monitor

  if (!EEPROM.begin(1000)) {
    Serial.println("Failed to initialise EEPROM");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }

  hotCount = EEPROM.readLong(HOT_ADDR);
  coldCount = EEPROM.readLong(COLD_ADDR);
  Serial.print("Cold: ");
  Serial.print(coldCount);
  Serial.print("Hot: ");
  Serial.println(hotCount);
  
  
  pinMode(GPIO_NUM_33, INPUT_PULLDOWN);
  pinMode(GPIO_NUM_32, INPUT_PULLDOWN);
  //esp_sleep_enable_ext0_wakeup(GPIO_NUM_32,1); //1 = High, 0 = Low  //button wake up


  /*https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/sleep_modes.html
   * if RTC peripherals are powered down, internal pullup and pulldown resistors will be disabled. To use internal pullup or pulldown resistors, request the RTC peripherals power domain to be kept on during sleep, and configure pullup/pulldown resistors using rtc_gpio_ functions before entering sleep:
   * esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
   * gpio_pullup_dis(gpio_num);
   * gpio_pulldown_en(gpio_num);
   */
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  esp_sleep_enable_ext1_wakeup(PINS_WAKE_BIT_MASK, ESP_EXT1_WAKEUP_ANY_HIGH);
  gpio_pulldown_en(GPIO_NUM_32);
  gpio_pulldown_en(GPIO_NUM_33);

  
  esp_sleep_enable_timer_wakeup(TIME_TO_WAKEUP_HTTP * uS_TO_S_FACTOR);    //timer wake up
  print_wakeup_reason();
}
/*
    WL_NO_SHIELD        = 255,   // for compatibility with WiFi Shield library
    WL_IDLE_STATUS      = 0,
    WL_NO_SSID_AVAIL    = 1,
    WL_SCAN_COMPLETED   = 2,
    WL_CONNECTED        = 3,
    WL_CONNECT_FAILED   = 4,
    WL_CONNECTION_LOST  = 5,
    WL_DISCONNECTED     = 6
*/

void sendHttp(){
  actionTimer = millis();
  wifiMulti.addAP("gbox-a", "bo7ro3da");
  Serial.println("Start wifi");
  int res = wifiMulti.run();
  int i = 0;
  Serial.println("Start run");
  while (res != WL_CONNECTED && i < 10) {
    res = wifiMulti.run();
    i++;
    Serial.print("Trying to connect.. ");
    Serial.println(res);
    delay(1000);
  }

  Serial.println(res);
  if(res == WL_CONNECTED) {
        HTTPClient http;
        Serial.print("[HTTP] begin...\n");
        char out[40];
        snprintf(out, 40, "%s/?cold=%d&hot=%d", HOST, coldCount, hotCount);
        http.begin(out);
        Serial.print("[HTTP] GET... " );
        Serial.println(out);
        int httpCode = http.GET();
        if(httpCode > 0) {
            Serial.printf("[HTTP] GET... code: %d\n", httpCode);
            if(httpCode == HTTP_CODE_OK) {
                String payload = http.getString();
                Serial.println(payload);
            }
        } else {
            Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
        }
        http.end();
        actionTimer = millis();
    }
        
}

void triggeredHot() {
      hotCount = EEPROM.readLong(HOT_ADDR);
      ++hotCount;
      EEPROM.writeLong(HOT_ADDR, hotCount);
      EEPROM.commit();
      Serial.print("Hot: ");
      Serial.println(hotCount);
      actionTimer = millis();
      wasPressedHot = true;
}

void triggeredCold() {
      coldCount = EEPROM.readLong(COLD_ADDR);
      ++coldCount;
      EEPROM.writeLong(COLD_ADDR, coldCount);
      EEPROM.commit();
      Serial.print("Cold: ");
      Serial.println(coldCount);
      actionTimer = millis();
      wasPressedCold = true;
}


void loop(){

if (doSendHttp) {
  doSendHttp = false;
  sendHttp();
}

    if (digitalRead(GPIO_NUM_32) && !wasPressedHot) {
        triggeredHot();
    }
    if (!digitalRead(GPIO_NUM_32)) {
      wasPressedHot = false;
    }

    if (digitalRead(GPIO_NUM_33) && !wasPressedCold) {
        triggeredCold();
    }
    if (!digitalRead(GPIO_NUM_33)) {
      wasPressedCold = false;
    }
    
if (millis() - actionTimer > SLEEP_AFTER) {
    Serial.println("Going to sleep in loop. ");
    esp_deep_sleep_start();
}
}
