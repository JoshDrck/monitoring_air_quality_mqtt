#include <Arduino.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <Wire.h>
#include <PMserial.h>
#include <Multichannel_Gas_GMXXX.h>
#define HOSTNAME "ESP32"

WiFiClient espClient;
PubSubClient client(espClient);

// Wifi credentials

const char *ssid = "Josh";
const char *pass = "Lorejo1999";
const char *temp = "10";

// Wifi variables
const char *MQTT_HOST = "192.168.1.104";
const int MQTT_PORT = 1883;
const char *MQTT_USER = "homeassistant";
const char *MQTT_PASS = "civeiWiefu0Viemuy8lohng7iedah6sha8che3thie4goSh2eg2ohX7eegairika";

// MQTT TOPICS
const char MQTT_SUB_TOPIC[] = "air_quality/" HOSTNAME "/out";
const char MQTT_PUB_TOPIC[] = "air_quality/" HOSTNAME "/in";

// Variables globals
time_t now;
const int capacity = JSON_OBJECT_SIZE(128);
StaticJsonDocument<capacity> doc;

// PMS5003 Configurations
constexpr auto PMS_RX = 16;
constexpr auto PMS_TX = 16;

SerialPM pms(PMS5003, PMS_RX, PMS_TX); // Inicialización del sensor y sus pines PMSx003, RX, TX

GAS_GMXXX<TwoWire> gas;

static uint8_t recv_cmd[8] = {};

// Métodos
// Captura de datos desde el sensor PMS5003
void getDataPmsSensor()
{
  pms.read(); // Lectura del sensor

  if (pms)
  { // Lectura realizada
    Serial.print(F("PM2.5 "));
    Serial.print(pms.pm25);
    doc["PM2_5"] = pms.pm25;
    Serial.print(F(" [ug/m3]"));
    Serial.print(F(", "));
    Serial.print(F("PM10 "));
    Serial.print(pms.pm10);
    doc["PM10"] = pms.pm10;
    Serial.print(F(" [ug/m3]"));
    Serial.print(F(", "));
  }
}

void getDataGroveMultichannelSensor()
{
  uint8_t len = 0;
  uint8_t addr = 0;
  uint8_t i;
  uint32_t val = 0;

  val = gas.measure_NO2();
  Serial.print(F("NO2: "));
  Serial.print(gas.calcVol(val));
  Serial.print(F(" [ppm]"));
  Serial.print(F(", "));
  doc["NO2"] = gas.calcVol(val);
  Serial.print(" ");
  val = gas.measure_CO();
  Serial.print("CO: ");
  Serial.print(gas.calcVol(val));
  Serial.println(F(" [ppm]."));
  doc["CO"] = gas.calcVol(val);
}

// Métodos para la conexion MQTT
void mqtt_connect()
{
  while (!client.connected())
  {
    Serial.print("Time: ");
    Serial.print(ctime(&now));
    Serial.println("MQTT connecting");
    if (client.connect(HOSTNAME, MQTT_USER, MQTT_PASS))
    {
      Serial.println("Connected");
    }
    else
    {
      Serial.print("failed to connect, status: ");
      Serial.println(client.state());
      Serial.println("Wait 5 seconds, We try to reconnect");
      delay(5000);
    }
  }
}

void setup()
{
  // Codigo de inicio
  Serial.begin(115200);
  Serial.print("Intentando conexión con el SSID: ");
  Serial.println(ssid);
  WiFi.setHostname(HOSTNAME);
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
  }
  Serial.println();
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.println("");
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  Serial.print("Current time: ");
  Serial.print(asctime(&timeinfo));
  client.setServer(MQTT_HOST, MQTT_PORT);
  mqtt_connect();
  Serial.println("Arranque de los sensores...");
  pms.init();
  gas.begin(Wire, 0x08);
}

void loop()
{
  // Validación de conexión con el cliente
  if (!client.connected())
  {
    Serial.print("Can't connect to server");
    delay(5000);
  };
  // Recolección de datos
  getDataPmsSensor();
  getDataGroveMultichannelSensor();

  client.loop();

  // Subcripción al Broker MQTT
  char buffer[128];
  size_t n = serializeJson(doc, buffer);
  client.publish("air_quality/environments/in", buffer, n);
  delay(1000);
}