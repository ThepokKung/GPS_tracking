
///////////////////////////////////////////////////////////////////////////////////
// This is a GPS Tracking Project for NT inture
// Update 1.3
// LCD_1602 for show detail
// >> 1.3 Add time to wait wifi connecting
///////////////////////////////////////////////////////////////////////////////////
// include libarry
#include <WiFi.h> //Wifi
#include <PubSubClient.h> //Call MQTT
#include <ArduinoJson.h> //Make json file
#include <TinyGPS++.h> //Talk GPS
#include <HardwareSerial.h> //Talk GPS2
#include <Wire.h> //Wire connecct
#include <LiquidCrystal_I2C.h> // LCD

// Set pinout GPS
#define RXPin 16
#define TXPin 17

//Set serial band for GPS
static const uint32_t GPSBaud = 9600;

//Wifi connect
////////////////////////////////////////////////////////////////////////////////////
/*
  char* ssid = "Violet Evergarden"; //Wifi Name
  char* password = "25022021"; //Wifi Password
*/
////////////////////////////////////////////////////////////////////////////////////
char* ssid = "GPSTracking"; //Wifi Name
char* password = "043234794";
////////////////////////////////////////////////////////////////////////////////////
//MQTT connected
//Test server
const char* mqttServer = "180.180.216.61";
const int mqttPort = 1883;
const char* mqttUser = "juub";
const char* mqttPassword = "t0t12345";
char mqtt_name_id[] = "";
const char* mqtt_topic = "ntnode/gps/";
////////////////////////////////////////////////////////////////////////////////////
// Call Value
WiFiClient client;
PubSubClient mqtt(client);
TinyGPSPlus gps;
HardwareSerial ss(2);
LiquidCrystal_I2C lcd(0x27, 16, 2);
////////////////////////////////////////////////////////////////////////////////////
//Json config
StaticJsonDocument<256> doc;
char buffer[256];
////////////////////////////////////////////////////
void setupwifi() { //Fucniton Wifi setup for easy setup wifi (Update LCD)
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.begin(ssid, password);
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Wifi Status");
  lcd.setCursor(2, 1);
  lcd.print("Connecting");

  while (WiFi.status() != WL_CONNECTED) {
    delay(1500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Connecting");
  lcd.setCursor(6, 1);
  lcd.print("Succeed!");
  delay(2000);
  lcd.clear();
}

void setuplcd() {
  lcd.begin();
  lcd.backlight();
}

void callback (char* topic, byte* payload, unsigned int length) { //Talk with MQTT server
  deserializeJson(doc, payload, length);
  size_t n = serializeJson(doc, buffer);
  ///////////////////////////////////////////////////////////////////////////////////////////////
  if (mqtt.connect(mqtt_topic, mqttUser, mqttPassword)) {
    if (mqtt.publish(mqtt_topic, buffer, n) == true) {
      Serial.println("publish Valve status success");
    } else {
      Serial.println("publish Fail");
    }
  } else {
    Serial.println("Connect Fail MQTT");
  }
}

void reconnect_mqtt() { //Reconnect for MQTT or Wifi in ESP Down it is reconnect now
  while (!mqtt.connected()) {
    unsigned long currentMillis = millis();
    unsigned long previousMillis = 0;
    unsigned long interval = 30000;
    // if WiFi is down, try reconnecting every CHECK_WIFI_TIME seconds
    if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >= interval)) {
      Serial.print(millis());
      Serial.println("Reconnecting to WiFi...");
      WiFi.disconnect();
      WiFi.reconnect();
      previousMillis = currentMillis;
    }
    Serial.print("Attempting MQTT connection...");
    reconnect_lcd_text_1();
    //WiFi.reconnect();
    if (mqtt.connect(mqtt_name_id, mqttUser, mqttPassword)) {
      Serial.println("connected");
      reconnect_lcd_text_2();
      // Subscribe or resubscribe to a topic
      // You can subscribe to more topics (to control more LEDs in this example)
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setupnt() {
  lcd.clear();
  lcd.setCursor(6, 0);
  lcd.print("NT");
  lcd.setCursor(2, 1);
  lcd.print("GPS Tracking");
  delay(3000);
}

void setup() {
  Serial.begin(115200);
  ss.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin, false);
  Serial.println(TinyGPSPlus::libraryVersion());
  setuplcd();
  setupnt();
  setupwifi();
  mqtt.setServer(mqttServer, mqttPort); //Login MQTT
  mqtt.setCallback(callback);
}

void reconnect_lcd_text_1() {
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("Connecting lost");
  lcd.setCursor(1, 1);
  lcd.print("Reconnect...");
}

void reconnect_lcd_text_2() {
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Reconnect...");
  lcd.setCursor(6, 1);
  lcd.print("Succeed");
  delay(500);
}

void loop() {
  while (ss.available() > 0)
    if (gps.encode(ss.read()))
      displayInfo(); //It call only funciton you can config it in void displayInfo
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while (true);
  }
  delay(1000);
}

void displayInfo()
{
  if (!mqtt.connected()) {
    Serial.println("---Reconnect MQTT ---");
    reconnect_mqtt();
  }
  mqtt.loop();

  String device_id = "GPS tracking 2";
  doc["deviceid"] = device_id;

  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    Serial.print("Latitude :");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longtitude :");
    Serial.println(gps.location.lng(), 6);
    Serial.print("Speed :");
    Serial.println(gps.speed.kmph());
    Serial.print("Date :");
    Serial.println(gps.date.value());
    doc["latitude"] = gps.location.lat();
    doc["longitude"] = gps.location.lng();
    doc["speed"] = gps.speed.kmph();
    /////////////////////////////////////
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Lat :");
    lcd.setCursor(6, 0);
    lcd.print(gps.location.lat(), 6);
    lcd.setCursor(0, 1);
    lcd.print("Long:");
    lcd.setCursor(6, 1);
    lcd.print(gps.location.lng(), 6);
    /////////////////////////////////////
  }
  else
  {
    Serial.print(F("INVALID"));
    lcd.clear();
    lcd.setCursor(6, 0);
    lcd.print("GPS");
    lcd.setCursor(2, 1);
    lcd.print("Undecttable");
    doc["latitude"] = "NULL";
    doc["longitude"] = "NULL";
    doc["speed"] = "NULL";
  }

  size_t n = serializeJson(doc, buffer);

  if (mqtt.connect(mqtt_name_id, mqttUser, mqttPassword)) {
    Serial.println("\nConnected MQTT: ");
    if (mqtt.publish(mqtt_topic, buffer , n) == true) {
      Serial.println("publish success");
    } else {
      Serial.println("publish Fail");
    }
  } else {
    Serial.println("Connect Fail MQTT");
  }
  serializeJsonPretty(doc, Serial);
  Serial.println();
  Serial.println("=============");
  delay(3000);
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("Device_id");
  lcd.setCursor(0, 1);
  lcd.print(device_id);
  delay(2000);
}
