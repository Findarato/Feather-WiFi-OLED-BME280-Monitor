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
// needed to avoid link error on ram check

#define BME_SCK                 vault.readSCK();
#define BME_MISO                vault.readMISO();
#define BME_MOSI                vault.readMISO();
#define BME_CS                  vault.readCS();
#define SEALEVELPRESSURE_HPA    (1013.25)
#define OLED_RESET              3
#define LED                     0

Adafruit_BME280 bme; // I2C
unsigned long delayTime;
Adafruit_SSD1306 display(OLED_RESET);

//Button A is #0
//Button B is #16
//Button C is #2

#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH  16

#if (SSD1306_LCDHEIGHT != 32)
#error ("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

int buttonStateIP                   = LOW;          // current state of the button
int lastButtonStateIP               = HIGH;         // previous state of the button
int buttonStateTemp                 = LOW;          // current state of the button
int lastButtonStateTemp             = HIGH;         // previous state of the button
int buttonStateToggleDisplay        = LOW;         // current state of the button
int lastButtonStateToggleDisplay    = HIGH;          // last state of the botton
bool toggleDisplay                  = true;        // Turns the display on or off
int tracker                         = 0;            // How many times it has tried to connect
byte mac[6];                                        // the MAC address of your Wifi shield
String hardwareID;                                  // Display of the mac address
Varstore vault;
int lastArea                        = 9;

WiFiServer server(vault.readServerPort());

float Dewpoint,Humidity,Temperature,pfVcc,TemperatureF,battery,Altitude,Pressure;     // Setting up some variable states


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
            // display.println("");
            display.println(hardwareID);
            display.display();
            Serial.println("Connection Info");
        }
}
/**
 * Display Temperature data to OLED screen
 * @method displayTempValues
 */
void displayTempValues(){
    if( Temperature > 0) { // We are only going to show data if we have data
        if(lastArea !=1) {
            // display.clearDisplay();
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
/**
 * Read Temperature data from DHT sensor
 * @method readTempValues
 */
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
        // root["SystemV"]["pfVcc"] = pfVcc/1000,3;
        // root["SystemV"]["battery"] = battery/1000,3;
        return root;
}

void writeResponse(WiFiClient& client, JsonObject& json) {
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: application/json");
        client.println("Connection: close");
        client.println();
        json.prettyPrintTo(client);
        // display.clearDisplay();
}

void setup() {
    bool status;
    Serial.println("OLED FeatherWing test");
    // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
    // init done
    Serial.println("OLED begun");
    display.display();

    Serial.begin(115200);
    Serial.print("Connecting to ");
    Serial.println(vault.readSSID());
    delay(100);
    // Connect to WiFi network
    WiFi.begin(vault.readSSID(), vault.readPassword());
    WiFi.softAPdisconnect();
    WiFi.mode(WIFI_STA);
    // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
    // display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // initialize with the I2C addr 0x3C (for the 128x32)
    delay(200);

    // Clear the buffer.
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

    // default settings
    status = bme.begin(); // Start Sensor
    if (!status) {
            Serial.println("Could not find a valid BME280 sensor, check wiring!");
            while (1) ;
    }

    connectionInfo();
    delay(5000);

    // Start the server
    server.begin();
    Serial.println("Server started");

    // initialize the button pin as a input:
    pinMode(vault.readButtonPinIP(), INPUT_PULLUP);
    pinMode(vault.readButtonPinTemp(), INPUT_PULLUP);
    pinMode(vault.readButtonPinDisplay(), INPUT_PULLUP);
}

void loop() {
    // Serial.println("Start: ");
    // Serial.println(lastArea);
    WiFiClient client = server.available();
    // Serial.println("WiFi Started: ");
    if (client) { // We have a client connected
        bool success = readRequest(client);
        if (success) {
            delay(1000); // Wait a second to ensure that we do not overload the seonsor
            readTempValues();
            display.clearDisplay();
            display.display();
            StaticJsonBuffer<500> jsonBuffer;
            JsonObject& json = prepareResponse(jsonBuffer);
            writeResponse(client, json);
        }
        delay(1000); // pause 10 milliseconds and then kill the connection
        client.stop();
    }

    // read the pushbutton input pin:
    buttonStateIP = digitalRead(vault.readButtonPinIP());
    // compare the buttonState to its previous state
    if (buttonStateIP != lastButtonStateIP) {
        // if the state has changed, increment the counter
        connectionInfo();
        lastArea = 0;
    }


    if( ! digitalRead(vault.readButtonPinTemp())){
        readTempValues();
        lastArea = 1;
    }
    // buttonStateTemp = digitalRead(vault.readButtonPinTemp());
    // // compare the buttonState to its previous state
    // if (buttonStateTemp != lastButtonStateTemp) {
    //     // if the state has changed, increment the counter

    // }
    // @TODO:This needs to be finished and the two buttons above need to work
    buttonStateToggleDisplay = digitalRead(vault.readButtonPinDisplay());
    // compare the buttonState to its previous state
    if (buttonStateToggleDisplay != lastButtonStateToggleDisplay ) {
        // if the state has changed, increment the counter
        // showDisplay();
        lastArea = 2;
    }

    // if (! digitalRead(BUTTON_A)) display.print("A");
    // if (! digitalRead(BUTTON_B)) display.print("B");
    // if (! digitalRead(BUTTON_C)) display.print("C");
    // Serial.print("lastArea: ");
    // Serial.println(lastArea);

}
