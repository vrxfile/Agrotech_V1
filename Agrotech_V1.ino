#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750FVI.h>

// Точка доступа Wi-Fi
char ssid[] = "MGBot";
char pass[] = "Terminator812";

// Параметры IoT сервера
char auth[] = "180ab65a8b4c4c678cc9806744df443f";
IPAddress blynk_ip(139, 59, 206, 133);

// Датчик освещенности
BH1750FVI bh1750;

// Датчик температуры/влажности и атмосферного давления
Adafruit_BME280 bme280;

// Датчик влажности почвы емкостной
const float air_value    = 857.0;
const float water_value  = 573.0;
const float moisture_0   = 0.0;
const float moisture_100 = 100.0;

#define ALARM_TIMER    100   // Период для таймера сигнализации удара
#define DOOR_TIMER     1000  // Период для таймера открытия дверцы
#define MAIN_TIMER     10000 // Период для таймера обновления данных
#define INTERNET_TIMER 60000 // Период для таймера тестирования Интернета
#define RESET_TIMER    60000 // Период для таймера общей перезагрузки

// Таймеры
BlynkTimer timer_main;
BlynkTimer timer_door;
BlynkTimer timer_reset;
BlynkTimer timer_alarm;
BlynkTimer timer_internet;

void setup()
{
  // Инициализация последовательного порта
  Serial.begin(115200);
  delay(512);
  Serial.println();
  Serial.println();
  Serial.println();

  // Инициализация I2C
  Wire.begin();
  Wire.setClock(10000L);

  // Инициализация модуля 4-х реле

  // Включение реле управления Wi-Fi роутером

  // Инициализация LCD дисплея

  // Ожидание запуска Wi-Fi роутера (~ 2 минуты)
  delay(60000);
  delay(60000);

  // Инициализация Wi-Fi и поключение к серверу Blynk
  Serial.print("Connecting to ");
  Serial.println(ssid);
  Blynk.begin(auth, ssid, pass, blynk_ip, 8442);
  delay(1024);
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  // Поверка подключения к Интернету

  // Инициализация датчика BH1750
  bh1750.begin();
  bh1750.setMode(Continuously_High_Resolution_Mode);

  // Инициализация датчика BME280
  bool bme_status = bme280.begin();
  if (!bme_status)
    Serial.println("Could not find a valid BME280 sensor, check wiring!");

  // Инициализация датчика VEML6075

  // Инициализация датчика LSM9DS1

  // Инициализация датчика RGB модуля

  // Инициализация датчика BUZZER модуля

  // Инициализация таймеров
  timer_update.setInterval(UPDATE_TIMER, readSendData);
}

void loop()
{
  Blynk.run();
  timer_update.run();
}

// Считывание датчиков и отправка данных на сервер Blynk
void readSendData()
{
  // Считывание датчика света
  float light = bh1750.getAmbientLight();
  Serial.print("Light = ");
  Serial.println(light);
  Blynk.virtualWrite(V5, light); delay(25);             // Отправка данных на сервер Blynk

  // Считывание датчика температуры/влажности/давления
  float air_temp = bme280.readTemperature();
  float air_hum = bme280.readHumidity();
  float air_press = bme280.readPressure() / 100.0F;
  Serial.print("Air temperature = ");
  Serial.println(air_temp);
  Serial.print("Air humidity = ");
  Serial.println(air_hum);
  Serial.print("Air pressure = ");
  Serial.println(air_press);
  Blynk.virtualWrite(V0, air_temp); delay(25);        // Отправка данных на сервер Blynk
  Blynk.virtualWrite(V1, air_hum); delay(25);         // Отправка данных на сервер Blynk
  Blynk.virtualWrite(V4, air_press); delay(25);       // Отправка данных на сервер Blynk

  // Датчиков почвы
  float adc1_1 = (float)ads1015.readADC_SingleEnded(0);
  float adc1_2 = (float)ads1015.readADC_SingleEnded(1);
  float soil_hum = map(adc1_1, air_value, water_value, moisture_0, moisture_100);
  float soil_temp = adc1_2 / 10.0;
  Serial.print("Soil temperature = ");
  Serial.println(soil_temp);
  Serial.print("Soil moisture = ");
  Serial.println(soil_hum);
  Blynk.virtualWrite(V2, soil_temp); delay(25);        // Отправка данных на сервер Blynk
  Blynk.virtualWrite(V3, soil_hum); delay(25);         // Отправка данных на сервер Blynk
}

// Управление реле #1 с Blynk
BLYNK_WRITE(V100)
{
  // Получение управляющего сигнала с сервера
  int relay_ctl = param.asInt();
  Serial.print("Relay power 1: ");
  Serial.println(relay_ctl);
  // Управление реле #1
  digitalWrite(RELAY_PIN_1, relay_ctl);
}

// Управление реле #2 с Blynk
BLYNK_WRITE(V101)
{
  // Получение управляющего сигнала с сервера
  int relay_ctl = param.asInt();
  Serial.print("Relay power 2: ");
  Serial.println(relay_ctl);
  // Управление реле #2
  digitalWrite(RELAY_PIN_2, relay_ctl);
}
