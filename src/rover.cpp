/**
   ESPNOW - Basic communication - Slave
   Date: 26th September 2017
   Author: Arvind Ravulavaru <https://github.com/arvindr21>
   Purpose: ESPNow Communication between a Master ESP32 and multiple ESP32 Slaves
   Description: This sketch consists of the code for the Slave module.
   Resources: (A bit outdated)
   a. https://espressif.com/sites/default/files/documentation/esp-now_user_guide_en.pdf
   b. http://www.esploradores.com/practica-6-conexion-esp-now/
   << This Device Slave >>
   Flow: Master
   Step 1 : ESPNow Init on Master and set it in STA mode
   Step 2 : Start scanning for Slave ESP32 (we have added a prefix of `slave` to the SSID of slave for an easy setup)
   Step 3 : Once found, add Slave as peer
   Step 4 : Register for send callback
   Step 5 : Start Transmitting data from Master to Slave(s)
   Flow: Slave
   Step 1 : ESPNow Init on Slave
   Step 2 : Update the SSID of Slave with a prefix of `slave`
   Step 3 : Set Slave in AP mode
   Step 4 : Register for receive callback and wait for data
   Step 5 : Once data arrives, print it in the serial monitor
   Note: Master and Slave have been defined to easily understand the setup.
         Based on the ESPNOW API, there is no concept of Master and Slave.
         Any devices can act as master or salve.
*/

#include <esp_now.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <Wire.h> //Needed for I2C to GPS

#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS
#include "ble.h"
#include "ArduinoJson.h"

#define CHANNEL 1

SFE_UBLOX_GPS myGPS;
HardwareSerial RTKSerial(2);
long lastTime = 0;
char locationData[100];

// constants won't change. They're used here to set pin numbers:
const int buttonPin = 34; // the number of the pushbutton pin
const int ledPin = 13;   // the number of the LED pin

// Variables will change:
int ledState = HIGH;       // the current state of the output pin
int buttonState;           // the current reading from the input pin
int lastButtonState = LOW; // the previous reading from the input pin

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0; // the last time the output pin was toggled
unsigned long debounceDelay = 50;

// Init ESP Now with fallback
void InitESPNow()
{
    WiFi.disconnect();
    if (esp_now_init() == ESP_OK)
    {
        Serial.println("ESPNow Init Success");
    }
    else
    {
        Serial.println("ESPNow Init Failed");
        // Retry InitESPNow, add a counte and then restart?
        // InitESPNow();
        // or Simply Restart
        ESP.restart();
    }
}

// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    RTKSerial.write(data, data_len);
    Serial.print("Last Packet Recv from: ");
    Serial.println(macStr);
    Serial.print("Last Packet Recv Data: ");
    Serial.println(*data);
    Serial.println("");
}

// config AP SSID
void configDeviceAP()
{
    String Prefix = "RTK_Rover:";
    String Mac = WiFi.macAddress();
    String SSID = Prefix + Mac;
    String Password = "123456789";
    bool result = WiFi.softAP(SSID.c_str(), Password.c_str(), CHANNEL, 0);
    if (!result)
    {
        Serial.println("AP Config failed.");
    }
    else
    {
        Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
    }
}
void setupGPS(){
    Wire.begin();
    // myGPS.enableDebugging(Serial);
    if (myGPS.begin(Wire) == false) //Connect to the Ublox module using Wire port
    {
        Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
        while (1)
            ;
    }

    // Check that this platform supports 64-bit (8 byte) double
    if (sizeof(double) < 8)
    {
        Serial.println(F("Warning! Your platform does not support 64-bit double."));
        Serial.println(F("The latitude and longitude will be inaccurate."));
    }

    myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
    myGPS.setNavigationFrequency(20); //Set output to 20 times a second

    byte rate = myGPS.getNavigationFrequency(); //Get the update rate of this module
    Serial.print("Current update rate: ");
    Serial.println(rate);

    //myGPS.saveConfiguration(); //Save the current settings to flash and BBR
}

void loopGps(){
    //Query module only every second.
    //The module only responds when a new position is available.
    if (millis() - lastTime > 1000)
    {
        lastTime = millis(); //Update the timer

        // getHighResLatitude: returns the latitude from HPPOSLLH as an int32_t in degrees * 10^-7
        // getHighResLatitudeHp: returns the high resolution component of latitude from HPPOSLLH as an int8_t in degrees * 10^-9
        // getHighResLongitude: returns the longitude from HPPOSLLH as an int32_t in degrees * 10^-7
        // getHighResLongitudeHp: returns the high resolution component of longitude from HPPOSLLH as an int8_t in degrees * 10^-9
        // getElipsoid: returns the height above ellipsoid as an int32_t in mm
        // getElipsoidHp: returns the high resolution component of the height above ellipsoid as an int8_t in mm * 10^-1
        // getMeanSeaLevel: returns the height above mean sea level as an int32_t in mm
        // getMeanSeaLevelHp: returns the high resolution component of the height above mean sea level as an int8_t in mm * 10^-1
        // getHorizontalAccuracy: returns the horizontal accuracy estimate from HPPOSLLH as an uint32_t in mm * 10^-1

        // First, let's collect the position data
        int32_t latitude = myGPS.getHighResLatitude();
        int8_t latitudeHp = myGPS.getHighResLatitudeHp();
        int32_t longitude = myGPS.getHighResLongitude();
        int8_t longitudeHp = myGPS.getHighResLongitudeHp();
        int32_t ellipsoid = myGPS.getElipsoid();
        int8_t ellipsoidHp = myGPS.getElipsoidHp();
        int32_t msl = myGPS.getMeanSeaLevel();
        int8_t mslHp = myGPS.getMeanSeaLevelHp();
        uint32_t accuracy = myGPS.getHorizontalAccuracy();

        // Defines storage for the lat and lon as double
        double d_lat; // latitude
        double d_lon; // longitude

        // Assemble the high precision latitude and longitude
        d_lat = ((double)latitude) / 10000000.0;       // Convert latitude from degrees * 10^-7 to degrees
        d_lat += ((double)latitudeHp) / 1000000000.0;  // Now add the high resolution component (degrees * 10^-9 )
        d_lon = ((double)longitude) / 10000000.0;      // Convert longitude from degrees * 10^-7 to degrees
        d_lon += ((double)longitudeHp) / 1000000000.0; // Now add the high resolution component (degrees * 10^-9 )

        // Print the lat and lon
        Serial.print("Lat (deg): ");
        Serial.print(d_lat, 9);
        Serial.print(", Lon (deg): ");
        Serial.print(d_lon, 9);

        // Now define float storage for the heights and accuracy
        float f_ellipsoid;
        float f_msl;
        float f_accuracy;

        // Calculate the height above ellipsoid in mm * 10^-1
        f_ellipsoid = (ellipsoid * 10) + ellipsoidHp;
        // Now convert to m
        f_ellipsoid = f_ellipsoid / 10000.0; // Convert from mm * 10^-1 to m

        // Calculate the height above mean sea level in mm * 10^-1
        f_msl = (msl * 10) + mslHp;
        // Now convert to m
        f_msl = f_msl / 10000.0; // Convert from mm * 10^-1 to m

        // Convert the horizontal accuracy (mm * 10^-1) to a float
        f_accuracy = accuracy;
        // Now convert to m
        f_accuracy = f_accuracy / 10000.0; // Convert from mm * 10^-1 to m

        // Finally, do the printing
        Serial.print(", Ellipsoid (m): ");
        Serial.print(f_ellipsoid, 4); // Print the ellipsoid with 4 decimal places

        Serial.print(", Mean Sea Level (m): ");
        Serial.print(f_msl, 4); // Print the mean sea level with 4 decimal places

        Serial.print(", Accuracy (m): ");
        Serial.println(f_accuracy, 4); // Print the accuracy with 4 decimal places
        StaticJsonDocument<200> doc;
        doc["lat"] = d_lat;
        doc["lon"] = d_lon;
        // doc["alt"] = "gps";
        // doc["hdn"] = "gps";
        // doc["spd"] = "gps";
        doc["acu"] = f_accuracy;
        // doc["tme"] = 1351824120;
        serializeJson(doc, locationData);
    }
}

void setup()
{
    Serial.begin(115200);
    RTKSerial.begin(115200, SERIAL_8N1, 16, 17);
    Serial.println("Elyir GPS-RTK Rover  (Using ESPNow)");
    //Set device in AP mode to begin with
    WiFi.mode(WIFI_AP);
    // configure device AP mode
    configDeviceAP();
    // This is the mac address of the Slave in AP Mode
    Serial.print("AP MAC: ");
    Serial.println(WiFi.softAPmacAddress());
    // Init ESPNow with a fallback logic
    InitESPNow();
    // Once ESPNow is successfully Init, we will register for recv CB to
    // get recv packer info.
    esp_now_register_recv_cb(OnDataRecv);
    setupGPS();
    setup_ble();
    pinMode(buttonPin, INPUT_PULLUP);
    pinMode(ledPin, OUTPUT);
}

void listenForButton(){
    int reading = digitalRead(buttonPin);

    // check to see if you just pressed the button
    // (i.e. the input went from LOW to HIGH), and you've waited long enough
    // since the last press to ignore any noise:

    // If the switch changed, due to noise or pressing:
    if (reading != lastButtonState)
    {
        // reset the debouncing timer
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay)
    {
        // whatever the reading is at, it's been there for longer than the debounce
        // delay, so take it as the actual current state:

        // if the button state has changed:
        if (reading != buttonState)
        {
            buttonState = reading;

            // only toggle the LED if the new button state is HIGH
            if (buttonState == LOW)
            {
                ledState = HIGH;
                send_ble_data(locationData);
            } else {
                ledState = LOW;
            }
        }
    }

    // set the LED:
    digitalWrite(ledPin, ledState);

    // save the reading. Next time through the loop, it'll be the lastButtonState:
    lastButtonState = reading;
}

void loop()
{
    listenForButton();
    loopGps();
    loop_ble();
}