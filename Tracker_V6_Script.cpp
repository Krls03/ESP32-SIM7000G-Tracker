#define TINY_GSM_MODEM_SIM7000
#define SerialMon Serial
#define SerialAT Serial1
#define TINY_GSM_DEBUG SerialMon

#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false

// GPRS credentials
const char apn[] = "Your APN";
const char gprsUser[] = "User"; // If Applicable
const char gprsPass[] = "Pass"; // If Applicable

// MQTT Credentials
#define MQTT_USER "Adafruit User"
#define MQTT_PASS "Paste Key Here"
#define MQTT_SERVER "io.adafruit.com"
#define MQTT_PORT 1883 // Change to secure port if needed

#define GPS_LOC_TOPIC MQTT_USER "Paste/Topic/Here"
#define BATTERY_TOPIC MQTT_USER "Paste/Topic/Here"

#include <TinyGsmClient.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <vfs_api.h>

// Debugging and Modem Setup
#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif
TinyGsmClient client(modem);
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, MQTT_PORT, MQTT_USER, MQTT_PASS);
Adafruit_MQTT_Publish gpsLoc = Adafruit_MQTT_Publish(&mqtt, GPS_LOC_TOPIC);
Adafruit_MQTT_Publish battery = Adafruit_MQTT_Publish(&mqtt, BATTERY_TOPIC);

#define UART_BAUD 9600
#define PIN_DTR 25
#define PIN_TX 27
#define PIN_RX 26
#define PWR_PIN 4
#define BAT_ADC 35
#define LED_PIN 12
#define PIN_LED 2

float lat = 0, lon = 0, altitude = 0, speed_mph = 0;
char gpsdata[120];
char batteryData[10];
RTC_DATA_ATTR int bootCount = 0;

// Function to read battery voltage
float readBattery(uint8_t pin)
{
    int vref = 1100;
    uint16_t volt = analogRead(pin);
    float battery_voltage = ((float)volt / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
    return battery_voltage;
}

// Function to publish battery data
void publishBattery()
{
    float battery_voltage = readBattery(BAT_ADC);
    int battery_percentage = map(battery_voltage * 1000, 3300, 4200, 0, 100);
    if (battery_percentage < 0)
        battery_percentage = 0;
    if (battery_percentage > 100)
        battery_percentage = 100;

    snprintf(batteryData, sizeof(batteryData), "%d%%", battery_percentage);
    Serial.print("Battery Voltage: ");
    Serial.print(battery_voltage);
    Serial.print(" V, ");
    Serial.print("Battery Percentage: ");
    Serial.print(battery_percentage);
    Serial.println("%");

    if (!battery.publish(batteryData))
    {
        Serial.println("Battery publish failed");
    }
    else
    {
        Serial.println("Battery publish success");
    }
}

// Function to transmit GPS coordinates
void transCoordinates()
{
    unsigned long previousMillis = 0;
    unsigned long ledInterval = 500;     // 500ms when trying to get GPS fix
    unsigned long gpsFixInterval = 5000; // 5 seconds after obtaining GPS fix
    bool gpsFixObtained = false;

    while (true)
    {
        modem.sendAT("+SGPIO=0,4,1,1");
        if (modem.waitResponse(10000L) != 1)
        {
            Serial.println(" SGPIO=0,4,1,1 false ");
        }

        modem.enableGPS();
        Serial.println("Getting Current Coordinates...");

        unsigned long currentMillis = millis();

        if (gpsFixObtained)
        {
            // Blink every 5 seconds after obtaining GPS fix
            ledInterval = gpsFixInterval;
        }
        else
        {
            // Blink every 500ms while trying to get a GPS fix
            ledInterval = 500;
        }

        // Toggle LED based on interval
        if (currentMillis - previousMillis >= ledInterval)
        {
            previousMillis = currentMillis;
            digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Toggle LED
        }

        if (modem.getGPS(&lat, &lon, &speed_mph, &altitude))
        {
            // If a valid GPS fix is obtained
            gpsFixObtained = true;
            Serial.println("Latitude: " + String(lat, 8) + "\tLongitude: " + String(lon, 8));

            snprintf(gpsdata, sizeof(gpsdata), "%.6f,%.6f,%.2f,%.2f", speed_mph, lat, lon, altitude);

            if (!gpsLoc.publish(gpsdata))
            {
                Serial.println("GPS publish failed");
            }
            else
            {
                Serial.println("GPS publish success");
            }

            publishBattery();
            delay(5000); // Wait 5 seconds before checking again
        }
        else
        {
            Serial.println("No valid GPS fix found, retrying...");
        }
    }
}

// MQTT Connection Handler
bool mqttConnect()
{
    SerialMon.print(F("Connecting to Adafruit IO... "));
    int8_t ret;
    while ((ret = mqtt.connect()) != 0)
    {
        SerialMon.print("MQTT Connection failed (code ");
        SerialMon.print(ret);
        SerialMon.println("), retrying...");
        mqtt.disconnect();
        delay(10000);
    }
    SerialMon.println(F("Connected to Adafruit IO"));
    return true;
}

// Setup function
void setup()
{
    Serial.begin(115200);
    delay(10);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, HIGH);
    delay(300);
    digitalWrite(PWR_PIN, LOW);

    Serial.println("\nWait...");
    delay(1000);

    SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
    if (bootCount == 0)
    {
        Serial.println("Initializing modem...");
        if (!modem.restart())
        {
            Serial.println("Failed to restart modem, attempting to continue without restarting");
        }
        bootCount++;
    }

    if (!modem.waitForNetwork())
    {
        Serial.println("Network connection failed");
        delay(10000);
        return;
    }
    if (!modem.gprsConnect(apn, gprsUser, gprsPass))
    {
        Serial.println("GPRS connection failed");
        delay(10000);
        return;
    }
}

void loop()
{
    if (!mqtt.connected())
    {
        if (!mqttConnect())
        {
            delay(5000);
            return;
        }
    }

    transCoordinates();
}
