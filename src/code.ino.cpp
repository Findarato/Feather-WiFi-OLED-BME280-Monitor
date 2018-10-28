# 1 "/tmp/tmpYJT2k4"
#include <Arduino.h>
# 1 "/home/joe/Documents/src/PlatformIO/Feather-WiFi-OLED-BME280-Monitor/src/code.ino"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <Adafruit_BME280.h>
#include "Pubvars.h"


#define BME_SCK vault.readSCK();
#define BME_MISO vault.readMISO();
#define BME_MOSI vault.readMISO();
#define BME_CS vault.readCS();
#define SEALEVELPRESSURE_HPA (1013.25)
#define OLED_RESET 3
#define LED 0

Adafruit_BME280 bme;
unsigned long delayTime;
Adafruit_SSD1306 display(OLED_RESET);





#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH 16

#if (SSD1306_LCDHEIGHT != 32)
#error ("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

int buttonStateIP = LOW;
int lastButtonStateIP = HIGH;
int buttonStateTemp = LOW;
int lastButtonStateTemp = HIGH;
int buttonStateToggleDisplay = LOW;
int lastButtonStateToggleDisplay = HIGH;
bool toggleDisplay = true;
int tracker = 0;
byte mac[6];
String hardwareID;
Varstore vault;
int lastArea = 9;

WiFiServer server(vault.readServerPort());

float Dewpoint,Humidity,Temperature,pfVcc,TemperatureF,battery,Altitude,Pressure;
String macToStr(const uint8_t* mac);
void connectionInfo ();
void displayTempValues();
void readTempValues();
bool readRequest(WiFiClient& client);
JsonObject& prepareResponse(JsonBuffer& jsonBuffer);
void writeResponse(WiFiClient& client, JsonObject& json);
void setup();
void loop();
#line 55 "/home/joe/Documents/src/PlatformIO/Feather-WiFi-OLED-BME280-Monitor/src/code.ino"
String macToStr(const uint8_t* mac) {
        String result;
        for (int i = 0; i < 6; ++i) {
                result += String(mac[i], 16);
                if (i < 5)
                        result += ':';
        }
        return result;
}

void connectionInfo () {
        if(lastArea != 0) {
            display.clearDisplay();
            display.display();
            display.setTextSize(1);
            display.setTextColor(WHITE);
            display.setCursor(0,0);
            display.println(vault.readSSID());
            display.println(WiFi.localIP());
            WiFi.macAddress(mac);
            hardwareID = macToStr(mac);
            display.println(vault.readDeviceName());

            display.println(hardwareID);
            display.display();
            Serial.println("Connection Info");
        }
}




void displayTempValues(){
    if( Temperature > 0) {
        if(lastArea !=1) {

            display.display();
            display.setCursor(0,0);
            display.setTextSize(1);
            display.setTextColor(WHITE);
            display.print("Temp C: ");
            display.println(Temperature);
            display.print("Temp F: ");
            display.println(TemperatureF);
            display.print("Humidity: ");
            display.println(Humidity);
            display.print("pressure hPa: ");
            display.println(Pressure);
            display.display();
        }
    }
}




void readTempValues() {
    Temperature = bme.readTemperature();
    Humidity = bme.readHumidity();
    Altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    Pressure = bme.readPressure() / 100.0F;
    TemperatureF = ((Temperature*9)/5)+32;

    Dewpoint = Temperature - ((100 - Humidity )/5);

    displayTempValues();
    Serial.println(Temperature);
    Serial.println(TemperatureF);
    Serial.println(Humidity);
    Serial.println(Dewpoint);
    Serial.println(Pressure);
    Serial.println(Altitude);
    long rssi = WiFi.RSSI();
    Serial.print("RSSI:");
    Serial.println(rssi);
}
bool readRequest(WiFiClient& client) {
    bool currentLineIsBlank = true;
    while (client.connected()) {
        if (client.available()) {
            char c = client.read();
            if (c == '\n' && currentLineIsBlank) {
                    return true;
            } else if (c == '\n') {
                    currentLineIsBlank = true;
            } else if (c != '\r') {
                    currentLineIsBlank = false;
            }
        }
    }
    return false;
}

JsonObject& prepareResponse(JsonBuffer& jsonBuffer) {

        long rssi = WiFi.RSSI();
        Serial.print("RSSI:");
        Serial.println(rssi);
        JsonObject& root = jsonBuffer.createObject();
        root["DeviceName"] = vault.readDeviceName();
        root["DeviceID"] = vault.readDeviceID();
        root["HardwareID"] = hardwareID;
        root["tempF"] = TemperatureF;
        root["tempC"] = Temperature;
        root["humidity"] = Humidity;
        root["altitude"] = Altitude;
        root["pressure"] = Pressure;
        root["dewpoint"] = Dewpoint;
        root["rssi"] = rssi;


        return root;
}

void writeResponse(WiFiClient& client, JsonObject& json) {
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: application/json");
        client.println("Connection: close");
        client.println();
        json.prettyPrintTo(client);

}

void setup() {
    bool status;
    Serial.println("OLED FeatherWing test");

    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

    Serial.println("OLED begun");
    display.display();

    Serial.begin(115200);
    Serial.print("Connecting to ");
    Serial.println(vault.readSSID());
    delay(100);

    WiFi.begin(vault.readSSID(), vault.readPassword());
    WiFi.softAPdisconnect();
    WiFi.mode(WIFI_STA);


    delay(200);


    display.clearDisplay();
    while (WiFi.status() != WL_CONNECTED) {
        delay(2000);
        Serial.print("");
        Serial.println("");
        Serial.print("Connecting to: ");
        Serial.print(vault.readSSID());
        Serial.println("");
        Serial.print("Status: ");
        Serial.print(WiFi.status());
        Serial.println("");
        Serial.print("IP Address: ");
        Serial.print(WiFi.localIP());
        Serial.println("");
        Serial.print("Loop: ");
        Serial.print(tracker);
        Serial.println("");

        display.clearDisplay();
        display.setTextColor(WHITE);
        display.setCursor(0,0);
        display.println("Connecting to: ...");
        display.println(vault.readSSID());
        display.display();
        tracker++;
    }


    status = bme.begin();
    if (!status) {
            Serial.println("Could not find a valid BME280 sensor, check wiring!");
            while (1) ;
    }

    connectionInfo();
    delay(5000);


    server.begin();
    Serial.println("Server started");


    pinMode(vault.readButtonPinIP(), INPUT_PULLUP);
    pinMode(vault.readButtonPinTemp(), INPUT_PULLUP);
    pinMode(vault.readButtonPinDisplay(), INPUT_PULLUP);
}

void loop() {


    WiFiClient client = server.available();

    if (client) {
        bool success = readRequest(client);
        if (success) {
            delay(1000);
            readTempValues();
            display.clearDisplay();
            display.display();
            StaticJsonBuffer<500> jsonBuffer;
            JsonObject& json = prepareResponse(jsonBuffer);
            writeResponse(client, json);
        }
        delay(1000);
        client.stop();
    }


    buttonStateIP = digitalRead(vault.readButtonPinIP());

    if (buttonStateIP != lastButtonStateIP) {

        connectionInfo();
        lastArea = 0;
    }


    if( ! digitalRead(vault.readButtonPinTemp())){
        readTempValues();
        lastArea = 1;
    }







    buttonStateToggleDisplay = digitalRead(vault.readButtonPinDisplay());

    if (buttonStateToggleDisplay != lastButtonStateToggleDisplay ) {


        lastArea = 2;
    }







}