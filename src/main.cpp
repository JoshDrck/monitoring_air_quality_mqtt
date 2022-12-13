#include <Arduino.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <Wire.h>
#include <PMserial.h>
#include <Multichannel_Gas_GMXXX.h>
#include <Adafruit_BME680.h>
#include <MQ131.h>
#define HOSTNAME "ESP32"
#define AOUT 35
WiFiClient espClient;
PubSubClient client(espClient);

// Wifi credentials

const char *ssid = "Josh";
const char *pass = "Lorejo1999";
const char *temp = "10";

// Wifi variables
const char *MQTT_HOST = "192.168.1.155";
const int MQTT_PORT = 1883;
const char *MQTT_USER = "homeassistant";
const char *MQTT_PASS = "iepo2veeshiedaafa4oshuachaiNgei9quuchi2zei0Kaiw0IeZoovi1xei3ei7Z";

// Variables globals
time_t now;
const int capacity = JSON_OBJECT_SIZE(128);
StaticJsonDocument<capacity> doc;

// PMS5003 Configurations
constexpr auto PMS_RX = 16;
constexpr auto PMS_TX = 17;

SerialPM pms(PMS5003, PMS_RX, PMS_TX); // Inicialización del sensor y sus pines PMS003, RX, TX

// Grove Configurations
GAS_GMXXX<TwoWire> gas;

static uint8_t recv_cmd[8] = {};

TwoWire I2CGROVE = TwoWire(1);

#define I2C_SDA 18
#define I2C_SCL 19

// BME688 Configurations
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME680 bme;

// Métodos
// Inicialización de los sensores
void initBME688()
{
  if (!bme.begin(0x77))
  {
    Serial.println("No se pudo encontrar un sensor valido BME688, check wiring!");
  }
  else
  {
    Serial.println("Sensor BME688 Inicio correctamente =)");
    // Set up oversampling and filter initialization
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms
  }
}

void initMQ131()
{
  MQ131.begin(2, AOUT, LOW_CONCENTRATION, 680000);

  MQ131.setR0(364740.87);

  MQ131.setTimeToRead(1);

  Serial.println("Calibration parameters");
  Serial.print("R0 = ");
  Serial.print(MQ131.getR0());
  Serial.println(" Ohms");
  Serial.print("Time to heat = ");
  Serial.print(MQ131.getTimeToRead());
  Serial.println(" s");
}

// Captura de datos desde el sensor PMS5003
void getDataPmsSensor()
{
  // Lectura del sensor
  pms.read();

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
  Serial.print(F(" [ppm]."));
  Serial.print(F(", "));
  doc["CO"] = gas.calcVol(val);
}

void getDataBME688()
{
  if (!bme.performReading())
  {
    Serial.println("Failed to perform reading :(");
  }
  else
  {
    Serial.print("Temp: ");
    Serial.print(bme.temperature);
    doc["TEMP"] = bme.temperature;
    Serial.print(F(", "));
    Serial.print("Hum: ");
    Serial.print(bme.humidity);
    doc["HUM"] = bme.humidity;
    Serial.print(F(", "));
  }
}

void getDataMQ131()
{
  MQ131.sample();
  Serial.print("O3: ");
  Serial.print(MQ131.getO3(PPM));
  doc["O3"] = MQ131.getO3(PPM);
  Serial.println(" ppm");
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
      delay(2000);
    }
  }
}

void setup()
{
  // Codigo de inicio
  Serial.begin(115200);

  Serial.print("Intentando conexión con el SSID: ");
  Serial.println(ssid);

  // Inicio de la conexión con al WiFi
  WiFi.setHostname(HOSTNAME);
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
  }
  Serial.println();

  // Resultado de la conección
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.println("");

  // Inicio de conexión con MQTT server
  client.setServer(MQTT_HOST, MQTT_PORT);
  mqtt_connect();

  // Inicioalizacion de los sensores
  Serial.println("Arranque de los sensores...");
  pms.init();
  I2CGROVE.begin(I2C_SDA, I2C_SCL);
  gas.begin(I2CGROVE, 0x08);
  initBME688();
  initMQ131();
}

void loop()
{
  // Validación de conexión con el cliente
  if (!client.connected())
  {
    Serial.print("Can't connect to server");
    reconnect();
  }
  client.loop();

  // Recolección de datos
  getDataPmsSensor();
  getDataGroveMultichannelSensor();
  getDataBME688();
  getDataMQ131();

  // Subcripción al Broker MQTT
  char buffer[128];
  size_t n = serializeJson(doc, buffer);
  client.publish("air_quality/environments/in", buffer, n);
  delay(1000);
}