#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiManager.h>        // https://github.com/tzapu/WiFiManager
#include <PubSubClient.h>       // https://pubsubclient.knolleary.net/api 
#include <Adafruit_NeoPixel.h>  // https://github.com/adafruit/Adafruit_NeoPixel
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>               
#include <EEPROM.h>            // Use LittleFS instead of SPIFFS
#include <ArduinoJson.h>
#include <INA219.h>
#include <DoubleResetDetect.h>
#include <EasyButton.h>
#include <Adafruit_AHTX0.h>

//custom header
#include "Timer.h"
#include "objects.h"
#include "MCP7940_Scheduler.h"
#include "helper.h"

BearSSL::WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);
MCP7940Scheduler rtc;
Adafruit_NeoPixel led(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
LedColor ledColorPicker[2] = {LedColor::RED,LedColor::OFF};
State deviceState;
INA219 INA(INA219_I2C_ADDR);
DoubleResetDetect drd(DRD_TIMEOUT, DRD_ADDRESS);
EasyButton button(BUTTON_PIN,BUTTON_DEBOUNCE_TIME,false,false);
Adafruit_AHTX0 aht20;
bool hasAht20 = false;
#ifdef INA219_I2C_ADDR
bool hasIna219 = false;
#endif
String deviceId;


bool picker = false;
bool resetTrigger = false;
bool resetInitiatorMode = false;
bool mqttloop,firmwareUpdate,firmwareUpdateOngoing;
float current  = 0;
unsigned long resetModeStartTime = 0;
String deviceBootTime = "";


WiFiManager wm;
MqttCredentials mqttDetails;
volatile uint8_t lowCurrentBelowThresholdCount = 0;
// Set up WiFiManager parameters
WiFiManagerParameter custom_mqtt_server("mqtt_server", "MQTT Server", "", 60);
WiFiManagerParameter custom_mqtt_port("mqtt_port", "MQTT Port", "", 4);
WiFiManagerParameter custom_mqtt_username("username", "Username", "", 32);
WiFiManagerParameter custom_mqtt_password("password", "Password", "", 32);


// Flag for saving data
bool shouldSaveConfig = false;

// Callback notifying us of the need to save config
void saveConfigCallback() {
  Serial.println("Should save config");

  strcpy(mqttDetails.mqtt_server,custom_mqtt_server.getValue());
  strcpy(mqttDetails.mqtt_user,custom_mqtt_username.getValue());
  strcpy(mqttDetails.mqtt_password,custom_mqtt_password.getValue());
  mqttDetails.mqtt_port = static_cast<uint16_t>(atoi(custom_mqtt_port.getValue()));

  Serial.println("Value of mqttDetails param that wiil be saved");
  Serial.println(mqttDetails.mqtt_server);
  Serial.println(mqttDetails.mqtt_user);
  Serial.println(mqttDetails.mqtt_password);
  Serial.println(mqttDetails.mqtt_port);
  Serial.println("****");

  Serial.println("Saving config...");
  eeprom_saveconfig();
  ESP.restart();
}

void configPotrtalTimeoutCalback() {
  if (!digitalRead(MOSFET_PIN)) {
    wm.reboot();
  }
}

void gracefullShutownprep(){
  mqttClient.disconnect();
  wm.disconnect();
  pumpStop();
  led.setPixelColor(0,LedColor::OFF);
  led.show();
  delay(100);
}

void doubleClickHandler() {
  if (resetInitiatorMode) {
      Serial.println("Double-click in reset mode - wiping credentials and restarting...");
      wipeCredentialsAndRestart();
      
  } else {
      Serial.println("Double-click detected, toggling pump.");
      if (!digitalRead(MOSFET_PIN)) {
          pumpStart();
      } else {
          pumpStop();
      }
  }
}

void enterConfigPortal() {
  // Disconnect services and cleanly stop peripherals
  gracefullShutownprep();
  // Indicate local-only mode and bring up non-blocking config portal AP
  deviceState.radioStatus = ConnectivityStatus::LOCALNOTCONNECTED;
  wm.startConfigPortal(generateApSSID().c_str());
}
void wipeCredentialsAndRestart() {
  // Clear EEPROM for stored MQTT details
  EEPROM.begin(sizeof(mqttDetails) + 10);
  for (int i = 0; i < sizeof(mqttDetails) + 10; i++) {
    EEPROM.write(i, 0);
  }
  EEPROM.commit();
  EEPROM.end();

  // Graceful shutdown
  gracefullShutownprep();
  // Wipe WiFi and WM settings
  wm.resetSettings();
  // Restart device
  delay(1000);
  ESP.restart();
}
            
void longPressHandler() {
  if (!resetInitiatorMode) {
      resetInitiatorMode = true;
      resetModeStartTime = millis();
      Serial.println("Reset initiator mode activated - double-click to reset credentials");
  } else {
    resetInitiatorMode = false;
    Serial.println("Reset initiator mode deactivated");
  }
}

void buttonISR()
{
  // When button is being used through external interrupts, parameter INTERRUPT must be passed to read() function.
  button.read();
}

// Method to generate the string in the format "prefix_chipID_last4mac"
String generateDeviceID() {
  String chipID = String(WIFI_getChipId(),HEX);
  chipID.toUpperCase();
  String macAddress = WiFi.macAddress();
  macAddress.replace(":", "");
  String last4Mac = macAddress.substring(macAddress.length() - 4);
  String deviceID = chipID + "-" + last4Mac;
  return deviceID;
}

// Check if received topic's last segment matches the last segment of a constant
static inline bool topicMatchesSuffix(const char* received, const char* fullConst) {
  const char* r = strrchr(received, '/');
  r = r ? r + 1 : received;
  const char* s = strrchr(fullConst, '/');
  s = s ? s + 1 : fullConst;
  return strcmp(r, s) == 0;
}

// Method to generate WiFi AP SSID in format: BEEGREEN-<chipID><last4mac>
String generateApSSID() {
  return String("BEEGREEN-") + generateDeviceID();
}

// remove per-device topic helpers

void wifiScanReport(){
 int n = WiFi.scanNetworks();
  StaticJsonDocument<1024> doc; // Adjust size for your needs
  JsonArray wifiList = doc.to<JsonArray>();

  for (int i = 0; i < n; ++i) {
    JsonObject obj = wifiList.createNestedObject();
    obj["ssid"] = WiFi.SSID(i);
    obj["strength"] = WiFi.RSSI(i);
    obj["protected"] = WiFi.encryptionType(i) != ENC_TYPE_NONE; // true if encrypted
  }

  String json;
  serializeJson(wifiList, json);
   wm.server->send(200, "application/json", json);
}

void bindServerCallback(){
  wm.server->on("/wifiscan",wifiScanReport);
}

void setupWiFi() {
  // WiFi.mode(WIFI_STA);  // explicitly set mode, esp defaults to STA+AP
  wm.setCaptivePortalEnable(false);
  wm.setConfigPortalBlocking(false);
  wm.setConnectTimeout(20);
  wm.setWiFiAutoReconnect(true);
  wm.setConfigPortalTimeout(120);
  wm.setConfigPortalTimeoutCallback(configPotrtalTimeoutCalback);
  wm.addParameter(&custom_mqtt_server);
  wm.addParameter(&custom_mqtt_port);
  wm.addParameter(&custom_mqtt_username);
  wm.addParameter(&custom_mqtt_password);
  wm.setWebServerCallback(bindServerCallback);
  wm.setSaveConfigCallback(saveConfigCallback);
  
  //automatically connect using saved credentials if they exist
  //If connection fails it starts an access point with the specified name

  if (wm.autoConnect(generateApSSID().c_str())) {
    Serial.println("WiFi connected...yeey :)");
    deviceState.radioStatus = ConnectivityStatus::LOCALCONNECTED;
    checkForOTAUpdate();
    
  } else {
    Serial.println("Configportal running");
    deviceState.radioStatus = ConnectivityStatus::LOCALNOTCONNECTED;
  }

}

void mqttCallback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  char payloadStr[length + 1];
  strncpy(payloadStr, (char *)payload, length);
  payloadStr[length] = '\0';
  Serial.println(payloadStr);

  if (topicMatchesSuffix(topic, PUMP_CONTROL_TOPIC)) {
    int duration = atoi(payloadStr);

    if (duration == 0) {
      pumpStop();
    } else if (duration > 0) {
      pumpStart();
      rtc.setManualStopTime(duration);
    }
  } else if (topicMatchesSuffix(topic, SET_SCHEDULE)) {
    onSetScheduleCallback(payloadStr);
    if (!digitalRead(MOSFET_PIN)) {
      updateAndPublishNextAlarm();
    }
  } else if (topicMatchesSuffix(topic, REQUEST_ALL_SCHEDULES)) {
    WateringSchedules allSchedules;
    rtc.getSchedules(allSchedules);

    DynamicJsonDocument doc(512);
    JsonArray scheduleArray = doc.to<JsonArray>();

    char schedule_string[20]; 
    bool schedulesModified = false;

    for (int i = 0; i < MAX_SCHEDULES; i++) {
        auto& item = allSchedules.items[i];

        if (item.hour > 23 || item.minute > 59 || item.daysOfWeek > 127) {
            item.hour = 0;
            item.minute = 0;
            item.duration_sec = 0;
            item.daysOfWeek = 0;
            item.enabled = false;
            schedulesModified = true;
        }

        if (item.enabled) {
            snprintf(schedule_string, sizeof(schedule_string), "%d:%d:%d:%u:%d",
                     i,
                     item.hour,
                     item.minute,
                     item.duration_sec,
                     item.daysOfWeek);
            
                     scheduleArray.add(schedule_string);
        }
    }

    if (schedulesModified) {
        Serial.println("Found and fixed corrupt schedule data in RTC RAM.");
        rtc.setSchedules(allSchedules);
    }

    char payload_buffer[256];
    serializeJson(doc, payload_buffer, sizeof(payload_buffer));
    publishMsg(GET_ALL_SCHEDULES, payload_buffer, false);
  } else if (topicMatchesSuffix(topic, GET_UPDATE_REQUEST)) {
    if (atoi(payloadStr) == 1) {
      firmwareUpdate = true;
    }
  } else if (topicMatchesSuffix(topic, RESTART)) {
    gracefullShutownprep();
    ESP.restart();
  } else if (topicMatchesSuffix(topic, RESET_SETTINGS)) {
    enterConfigPortal();
  } else {
    Serial.print("Topic action not found");
  }
}

// Callback for SET_SCHEDULE
void onSetScheduleCallback(const char* payload) {
    int index;
    ScheduleItem newItem;
    
    if (parseSchedulePayload(payload, index, newItem)) {
        WateringSchedules allSchedules;
        
        if (!rtc.getSchedules(allSchedules)) {
            Serial.println("Could not read schedules, initializing new set.");
            memset(&allSchedules, 0, sizeof(allSchedules));
        }

        allSchedules.items[index] = newItem;

        if (rtc.setSchedules(allSchedules)) {
            Serial.printf("Schedule at index %d saved successfully.\n", index);
        } else {
            Serial.println("Failed to save schedules to RTC RAM.");
        }
    } else {
        Serial.println("Invalid schedule format. Expected index:HH:MM:duration:daysOfWeek:enabled");
    }
}

// It parses the payload for setting a schedule: "index:HH:MM:duration:daysOfWeek:enabled"
bool parseSchedulePayload(const char* payload, int& index, ScheduleItem& item) {
    int enabled_int;
    int parsed = sscanf(payload, "%d:%2hhu:%2hhu:%hu:%hhu:%d", 
                        &index, &item.hour, &item.minute, 
                        &item.duration_sec, &item.daysOfWeek, &enabled_int);
    
    if (parsed == 6 && index >= 0 && index < MAX_SCHEDULES) {
        item.enabled = (enabled_int == 1);
        return true;
    }
    return false;
}

void publishMsg(const char *topic, const char *payload,bool retained){
  if (mqttClient.connected()) {
      // Prefix deviceId to the topic using the last segment as suffix
      char fullTopic[64];
      const char* suf = strrchr(topic, '/');
      suf = suf ? suf + 1 : topic;
      snprintf(fullTopic, sizeof(fullTopic), "%s/%s", deviceId.c_str(), suf);

      String jsonPayload = "{";
      jsonPayload += "\"payload\":";
      jsonPayload += "\"";
      jsonPayload += payload;
      jsonPayload += "\",";
      jsonPayload += "\"timestamp\":";
      jsonPayload += "\"";
      jsonPayload += rtc.getCurrentTimestamp();
      jsonPayload += "\"";
      jsonPayload += "}";

      mqttClient.publish(fullTopic, jsonPayload.c_str(),retained);
    }
}

void publishPowerStatusIfAny(){
  if (rtc.getPowerFail()) {
    DateTime pd = rtc.getPowerDown();
    DateTime pu = rtc.getPowerUp();

    char ton[20];
    char toff[20];
    char payload[41];

    snprintf(toff, sizeof(toff), "%04d-%02d-%02d %02d:%02d:%02d",
             pd.year(), pd.month(), pd.day(), pd.hour(), pd.minute(), pd.second());
    snprintf(ton, sizeof(ton), "%04d-%02d-%02d %02d:%02d:%02d",
             pu.year(), pu.month(), pu.day(), pu.hour(), pu.minute(), pu.second());
    snprintf(payload, sizeof(payload), "off:%s,on:%s", toff, ton);
    publishMsg(POWER_STATUS_TOPIC, payload, true);
  } else {
    publishMsg(POWER_STATUS_TOPIC, "no power failure detected", true);
  }
  rtc.clearPowerFail();
}

static inline void subscribeMsg(const char *topic) {
  char fullTopic[64];
  const char* suf = strrchr(topic, '/');
  suf = suf ? suf + 1 : topic;
  snprintf(fullTopic, sizeof(fullTopic), "%s/%s", deviceId.c_str(), suf);
  mqttClient.subscribe(fullTopic);
}

bool readAHT20() {
  if (!hasAht20) {
    return false;
  }

  sensors_event_t humidityEvent, tempEvent;
  aht20.getEvent(&humidityEvent, &tempEvent);

  deviceState.temp = tempEvent.temperature;
  deviceState.humidity = humidityEvent.relative_humidity;
  return true;
}

bool readINA219() {
  if (!hasIna219) {
    return false;
  }

  if (INA.isConnected()) {
    deviceState.currentConsumption = INA.getCurrent_mA();
    return true;
  }
  return false;
}

void calibrateAHT20(uint8_t samples = 5, uint16_t settleMs = 50) {
  if (!hasAht20) {
    return;
  }
  sensors_event_t humidityEvent, tempEvent;
  for (uint8_t i = 0; i < samples; i++) {
    aht20.getEvent(&humidityEvent, &tempEvent);
    delay(settleMs);
  }
  deviceState.temp = tempEvent.temperature;
  deviceState.humidity = humidityEvent.relative_humidity;
}

Timer monitorPumpCurrent(1000, Timer::SCHEDULER, []() {
  if (!deviceState.pumpRunning) {
    return;
  }
  if (readINA219()) {
    if (deviceState.currentConsumption < CURRENT_CONSUMPTION_THRESHOLD) {
      lowCurrentBelowThresholdCount++;
      if (lowCurrentBelowThresholdCount >= 3) {
        deviceState.waterTankEmpty = true;
        publishMsg(TANK_EMPTY, "1", true);
        pumpStop();
      }
    } else {
      lowCurrentBelowThresholdCount = 0;
    }
  }
});

Timer tankEmptyAnnounceAfterStart(3000, Timer::ONESHOT, []() {
  if (deviceState.pumpRunning && !deviceState.waterTankEmpty) {
    publishMsg(TANK_EMPTY, "0", true);
  }
});

void pumpStart(){
  if (!digitalRead(MOSFET_PIN) && (!firmwareUpdate)) {
    Serial.println("Starting pump");
    digitalWrite(MOSFET_PIN, HIGH);
    deviceState.pumpRunning = true;
    if (mqttClient.connected()) {
      publishMsg(PUMP_STATUS_TOPIC, "on",true);
    }
    deviceState.waterTankEmpty = false;
    lowCurrentBelowThresholdCount = 0;
    if (hasIna219) {
      monitorPumpCurrent.start();
    }
    tankEmptyAnnounceAfterStart.restart();
    return;
  } 
  Serial.println("Pump already in running state or upgrade in progress");
}

void pumpStop() {
  if (digitalRead(MOSFET_PIN)) {
    Serial.println("Stopping pump");
    monitorPumpCurrent.stop();
    tankEmptyAnnounceAfterStart.stop();
    lowCurrentBelowThresholdCount = 0;
    digitalWrite(MOSFET_PIN, LOW);
    deviceState.pumpRunning = false;
    publishMsg(PUMP_STATUS_TOPIC, "off",true);

    updateAndPublishNextAlarm();
  } else {
    Serial.println("Pump already in idle state");
  }
}

void checkForOTAUpdate() {
  HTTPClient http;
  Serial.println("Checking for OTA updates...");

  String updateURL = String(UPDATEURL) + "?nocache=" + String(millis()); // Force fresh request
  if (http.begin(espClient, updateURL)) { 
    Serial.println("Connected to update server...");
    http.setTimeout(5000); // Set 5-second timeout

    int httpCode = http.GET(); // Perform GET request to fetch the version file
    if (httpCode == HTTP_CODE_OK) {
      String fetchedFirmwareVersionString = http.getString(); // Store the String object
      fetchedFirmwareVersionString.trim(); // Trim whitespace

      Serial.println("Fetched Firmware Version: " + fetchedFirmwareVersionString);
      Serial.println("System Firmware Version: " + String(FIRMWARE_VERSION));
      

      // Compare fetched version with current firmware version
      if (v1GreaterThanV2(fetchedFirmwareVersionString.c_str(),FIRMWARE_VERSION)) {
        firmwareUpdateOngoing = true;
        Serial.println("Update available. Starting OTA...");
        mqttClient.disconnect(); // Ensure MQTT is disconnected during OTA
        String firmwareURL = String(FIRMWAREDOWNLOAD)+ fetchedFirmwareVersionString + ".bin";
        t_httpUpdate_return ret = ESPhttpUpdate.update(espClient, firmwareURL);

        if (ret == HTTP_UPDATE_OK) {
          Serial.println("OTA Update Successful");
          gracefullShutownprep();
          ESP.restart();
        } else {
            Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
          firmwareUpdateOngoing = false;
        }
      } else {
        Serial.println("No update available.");
      }
    } else {
      Serial.printf("Failed to fetch update file. HTTP code: %d\n", httpCode);
    }

    http.end(); // End the HTTP connection
  } else {
    Serial.println("Unable to connect to OTA update server.");
  }
  http.end();
}

void connectNetworkStack() {
  if (WiFi.status() == WL_CONNECTED && mqttClient.connected()) {
    deviceState.radioStatus = ConnectivityStatus::SERVERCONNECTED;
    return;
  }

  if (WiFi.status() == WL_CONNECTED && !mqttClient.connected()) {
    const char* clientId = deviceId.length() ? deviceId.c_str() : "beegreen";
    // Build LWT topic (deviceId/suffix of BEEGREEN_STATUS) and payload
    char willTopic[64];
    const char* wsuf = strrchr(BEEGREEN_STATUS, '/');
    wsuf = wsuf ? wsuf + 1 : BEEGREEN_STATUS;
    snprintf(willTopic, sizeof(willTopic), "%s/%s", deviceId.c_str(), wsuf);
    String willPayload = String("{\"payload\":\"offline\",\"timestamp\":\"") + deviceBootTime + "\"}";

    if (mqttClient.connect(clientId, mqttDetails.mqtt_user, mqttDetails.mqtt_password,
                           willTopic, 1, true, willPayload.c_str())) {
      subscribeMsg(PUMP_CONTROL_TOPIC);
      subscribeMsg(SET_SCHEDULE);
      subscribeMsg(REQUEST_ALL_SCHEDULES);
      subscribeMsg(GET_UPDATE_REQUEST);
      subscribeMsg(RESTART);
      subscribeMsg(RESET_SETTINGS);
      // Publish immediate online with retain so status reflects current state
      publishMsg(BEEGREEN_STATUS, "online", true);
      publishPowerStatusIfAny();
      deviceState.radioStatus = ConnectivityStatus::SERVERCONNECTED;
      return;
    }
    deviceState.radioStatus = ConnectivityStatus::SERVERNOTCONNECTED;
    return;
  }

  if (WiFi.status() != WL_CONNECTED && wm.getConfigPortalActive()) {
     deviceState.radioStatus = ConnectivityStatus::LOCALNOTCONNECTED;
     return;
  }

  if (WiFi.status() != WL_CONNECTED && !wm.getConfigPortalActive() && !digitalRead(MOSFET_PIN)) {
     wm.reboot();
  }
}

void updateAndPublishNextAlarm() {
  bool alarmWasSet = rtc.setNextAlarm();
  
  if (alarmWasSet) {
    DateTime nextAlarm = rtc.getNextDueAlarm();
    
    char buffer[20];
    snprintf(buffer, sizeof(buffer), "%04d-%02d-%02d %02d:%02d:%02d",
             nextAlarm.year(), nextAlarm.month(), nextAlarm.day(),
             nextAlarm.hour(), nextAlarm.minute(), nextAlarm.second());
             
    publishMsg(NEXT_SCHEDULE, buffer, true);
  } else {
    publishMsg(NEXT_SCHEDULE, "", true);
  }
}

Timer heartBeat(HEARTBEAT_TIMER,Timer::SCHEDULER,[]() {
    if (mqttClient.connected()) {
      String csv = String(FIRMWARE_VERSION);
    if (readAHT20()) {
      csv += "," + String(deviceState.temp, 2) + "," + String(deviceState.humidity, 2);
    }
    if (readINA219()) {
      csv += "," + String(deviceState.currentConsumption, 2);
    }
    publishMsg(HEARBEAT_TOPIC, csv.c_str(), true);
  }
});

Timer setLedColor(500,Timer::SCHEDULER,[](){
  if (firmwareUpdateOngoing){
    ledColorPicker[0] = LedColor::MAGENTA;
    ledColorPicker[1] = LedColor::MAGENTA;
  } else if (resetInitiatorMode){
    ledColorPicker[0] = LedColor::MAGENTA;
    ledColorPicker[1] = LedColor::OFF;
  } else if (deviceState.pumpRunning){
    ledColorPicker[0] = LedColor::BLUE;
    ledColorPicker[1] = LedColor::BLUE;
  } else {
    switch (deviceState.radioStatus) {
      case ConnectivityStatus::SERVERCONNECTED:
        ledColorPicker[0] = LedColor::GREEN;
        ledColorPicker[1] = LedColor::GREEN;
        break;
      case (ConnectivityStatus::LOCALNOTCONNECTED):
        ledColorPicker[0] = LedColor::RED;
        ledColorPicker[1] = LedColor::OFF;
        break;
      default:
        ledColorPicker[0] = LedColor::RED;
        ledColorPicker[1] = LedColor::RED;
        break;
    }
  }
  if (deviceState.waterTankEmpty) {
    ledColorPicker[1] = LedColor::BLUE;
  }

  picker = !picker;
  led.setPixelColor(0,ledColorPicker[int(picker)]);
  led.show();
});

Timer alarmHandler(1000, Timer::SCHEDULER, []() {
  if (rtc.alarmTriggered(ALARM::ONTRIGGER) && !digitalRead(MOSFET_PIN)) {
    Serial.println("onAlarm triggered: ");
    pumpStart();
  }

  if (rtc.alarmTriggered(ALARM::OFFTRIGGER) && digitalRead(MOSFET_PIN)) {
    Serial.println("offAlarm triggered: ");
    // pumpStop now correctly handles stopping the pump AND setting the next alarm.
    pumpStop();
  }
});

Timer loopMqtt(5000,Timer::SCHEDULER,[]() {
  mqttloop = true;
});

void eeprom_read() {
  EEPROM.begin(sizeof(mqttDetails) + 10);
  EEPROM.get(EEPROM_START_ADDR, mqttDetails);
  EEPROM.end();
}

void eeprom_saveconfig() {
  EEPROM.begin(sizeof(mqttDetails) + 10);
  EEPROM.put(EEPROM_START_ADDR, mqttDetails);
  if (EEPROM.commit()){
  EEPROM.end();
  } else { Serial.println ("SAving failed"); }
}

void stopServices() {
  mqttClient.disconnect();  // Disconnect MQTT
  // WiFi.disconnect();        // Disconnect WiFi
  // Stop any running timers or tasks
  heartBeat.stop();
  setLedColor.stop();
  alarmHandler.stop();
  loopMqtt.stop();
  // Ensure the pump is stopped
  pumpStop();
}




void setup() {
  Serial.begin(115200);
  if (drd.detect()) {
    Serial.println("Entering config mode via double reset");
    wipeCredentialsAndRestart();
  }

  firmwareUpdate = false;
  mqttloop = true;
  firmwareUpdateOngoing = false;
  pinMode(MOSFET_PIN, OUTPUT);
  digitalWrite(MOSFET_PIN, LOW);

  pinMode(LED_PIN, OUTPUT);
  led.begin();
  led.clear();

  Wire.begin(SDA_PIN, SCL_PIN);

  espClient.setInsecure();
  setupWiFi();
  eeprom_read();
  deviceId = generateDeviceID();
  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());
  Serial.println("Mqtt Details:");
  Serial.printf("- Server: %s\n",mqttDetails.mqtt_server);
  Serial.printf("- Port: %d\n",mqttDetails.mqtt_port);
  Serial.printf("- User: %s\n",mqttDetails.mqtt_user);
  mqttClient.setServer(mqttDetails.mqtt_server, mqttDetails.mqtt_port);
  mqttClient.setCallback(mqttCallback);

  pinMode(LED_PIN, OUTPUT);
  led.begin();
  led.clear();
  
  button.begin();
  button.onPressedFor(BUTTON_LONG_CLICK_TIME, longPressHandler);
  button.onSequence(2, BUTTON_DOUBLE_CLICK_TIME, doubleClickHandler);

  if (button.supportsInterrupt())
  {
    button.enableInterrupt(buttonISR);
    Serial.println("Button will be used through interrupts");
  }

  rtc.begin();
  deviceBootTime = rtc.getCurrentTimestamp();
  hasAht20 = aht20.begin();
  if (hasAht20) {
    Serial.println("AHT20 detected and initialized");
    calibrateAHT20();
  } else {
    Serial.println("AHT20 not detected");
  }

  #ifdef INA219_I2C_ADDR
    hasIna219 = INA.begin();
    if(hasIna219) {
        INA.setMaxCurrentShunt(MAX_CURRENT, SHUNT);
        Serial.println("INA219 detected and initialized");
    } else { Serial.println("INA219: Could not connect. Fix and Reboot"); }
  #endif

  heartBeat.start();
  setLedColor.start();
  alarmHandler.start();
  loopMqtt.start();
  updateAndPublishNextAlarm();
}

void loop() {
  wm.process();
  button.update();  // Handle button events - called every loop for responsiveness
  
  // Check WiFi status and attempt reconnection if needed
  if (WiFi.status() != WL_CONNECTED && !wm.getConfigPortalActive()) {
    static unsigned long lastWifiAttempt = 0;
    if (millis() - lastWifiAttempt > 30000) { // Every 30 seconds
      lastWifiAttempt = millis();
      WiFi.disconnect();
      WiFi.begin();
    }
  }
  mqttClient.loop();
  
  // Auto-exit reset initiator mode after 10 seconds if no action
  if (resetInitiatorMode && (millis() - resetModeStartTime > 10000)) {
    resetInitiatorMode = false;
    Serial.println("Reset initiator mode timeout - returning to normal mode");
  }

  if(mqttloop) {
    connectNetworkStack();
    mqttloop = false;
  }

  if ((firmwareUpdate) && (!digitalRead(MOSFET_PIN))) {
    checkForOTAUpdate();
    firmwareUpdate = false;
  }
}