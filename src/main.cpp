#include <Arduino.h>
#include <PMserial.h>
#include <Multichannel_Gas_GMXXX.h>
#include <Wire.h>

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
    Serial.print(F(", "));
    Serial.print(F("PM10 "));
    Serial.print(pms.pm10);
    Serial.println(F(" [ug/m3]"));
  }
  else
  { // Mensajes de error
    switch (pms.status)
    {
    case pms.OK:
      break;
    case pms.ERROR_TIMEOUT:
      Serial.println(F(PMS_ERROR_TIMEOUT));
      break;
    case pms.ERROR_MSG_UNKNOWN:
      Serial.println(F(PMS_ERROR_MSG_UNKNOWN));
      break;
    case pms.ERROR_MSG_HEADER:
      Serial.println(F(PMS_ERROR_MSG_HEADER));
      break;
    case pms.ERROR_MSG_BODY:
      Serial.println(F(PMS_ERROR_MSG_BODY));
      break;
    case pms.ERROR_MSG_START:
      Serial.println(F(PMS_ERROR_MSG_START));
      break;
    case pms.ERROR_MSG_LENGTH:
      Serial.println(F(PMS_ERROR_MSG_LENGTH));
      break;
    case pms.ERROR_MSG_CKSUM:
      Serial.println(F(PMS_ERROR_MSG_CKSUM));
      break;
    case pms.ERROR_PMS_TYPE:
      Serial.println(F(PMS_ERROR_PMS_TYPE));
      break;
    }
  }
}

void getDataGroveMultichannelSensor()
{
  uint8_t len = 0;
  uint8_t addr = 0;
  uint8_t i;
  uint32_t val = 0;

  val = gas.measure_NO2();
  Serial.print("NO2: ");
  Serial.print(val);
  Serial.print(" ");
  val = gas.measure_CO();
  Serial.print("CO: ");
  Serial.print(val);
  Serial.print(" | ");
}

void setup()
{
  // Codigo de inicio
  Serial.begin(115200);
  Serial.println("Arranque de los sensores...");
  pms.init();
  gas.begin(Wire, 0x08);
}

void loop()
{
  // put your main code here, to run repeatedly:
  getDataPmsSensor();
  getDataGroveMultichannelSensor();
  delay(100000);
}