#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MCP4725.h>
#include <SparkFunLSM9DS1.h>
#include <I2C_graphical_LCD_display.h>
#include <BH1750FVI.h>
#include <VEML6075.h>
#include <VL53L0X.h>
#include <PCA9536.h>
#include <TLC59108.h>

// Точка доступа Wi-Fi
char ssid[] = "MGBot";
char pass[] = "Terminator812";

// Параметры IoT сервера
char auth[] = "7782f301820d406396ee6202ec0d5c41";
IPAddress blynk_ip(139, 59, 206, 133);

// Датчик освещенности
BH1750FVI bh1750;

// Датчик температуры/влажности и атмосферного давления
Adafruit_BME280 bme;

// Датчик УФ излучения
VEML6075 veml;

// Датчик расстояния
VL53L0X lox;
//#define LONG_RANGE
//#define HIGH_SPEED
#define HIGH_ACCURACY

// Датчик положения
LSM9DS1 imu;
#define LSM9DS1_M   0x1E
#define LSM9DS1_AG  0x6B

// Модуль динамика
Adafruit_MCP4725 buzzer;

// Модуль RGB светодиодов
#define HW_RESET_PIN 0 // Только програмнный сброс
#define I2C_ADDR TLC59108::I2C_ADDR::BASE
TLC59108 leds(I2C_ADDR + 7); // Без перемычек добавляется 3 бита адреса

// Микросхема управления реле
PCA9536 pca9536;

// LCD дисплей
I2C_graphical_LCD_display lcd;

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
  delay(500);
  Serial.println();
  Serial.println();
  Serial.println();

  // Инициализация I2C
  Wire.begin();
  Wire.setClock(100000L);

  // Инициализация модуля 4-х реле
  pca9536.reset();
  pca9536.setState(IO_LOW);
  pca9536.setMode(IO_OUTPUT);
  delay(250);

  // Тестирование сирены
  pca9536.setState(IO1, IO_HIGH); delay(250);
  pca9536.setState(IO1, IO_LOW); delay(250);

  // Включение реле управления Wi-Fi роутером
  pca9536.setState(IO0, IO_HIGH); delay(250);

  // Инициализация LCD дисплея
  lcd.begin();
  lcd.clear (0, 0, 127, 63, 0x00);
  lcd.frameRect (0, 0, 127, 63, 1, 1);
  lcd.gotoxy (5, 5); lcd.string ("Waiting for WiFi...", false);

  // Ожидание запуска Wi-Fi роутера (45...60 секунд)
  // delay(30000);
  // delay(30000);

  // Инициализация Wi-Fi и поключение к серверу Blynk
  lcd.clear (1, 1, 126, 63, 0x00);
  lcd.gotoxy (5, 5); lcd.string ("Connecting WiFi...", false);
  lcd.gotoxy (5, 25); lcd.string ("Starting Blynk...", false);
  //  Serial.print("Connecting to ");
  //  Serial.println(ssid);
  //  Blynk.begin(auth, ssid, pass, blynk_ip, 8442);
  //  delay(1024);
  //  Serial.println("");
  //  Serial.println("WiFi connected");
  //  Serial.print("IP address: ");
  //  Serial.println(WiFi.localIP());
  //  Serial.println();

  // Поверка подключения к Интернету

  // Инициализация датчика BH1750
  bh1750.begin();
  bh1750.setMode(Continuously_High_Resolution_Mode);
  delay(250);

  // Инициализация датчика BME280
  bool bme_status = bme.begin();
  if (!bme_status)
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  delay(250);

  // Инициализация датчика VEML6075
  if (!veml.begin())
    Serial.println("VEML6075 not found!");
  delay(250);

  // Инициализация датчика LSM9DS1
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  bool imu_status = imu.begin();
  if (!imu_status)
    Serial.println("Failed to communicate with LSM9DS1!");

  // Инициализация датчика VL53L0x
  lox.init();
  lox.setTimeout(500);
#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  lox.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  lox.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  lox.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif
#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  lox.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  lox.setMeasurementTimingBudget(200000);
#endif

  // Инициализация RGB модуля
  leds.init(HW_RESET_PIN);
  leds.setLedOutputMode(TLC59108::LED_MODE::PWM_IND);
  byte pwm = 0x00; leds.setAllBrightness(pwm);
  pwm = 0xFE; leds.setBrightness(2, pwm);
  delay(250);

  // Инициализация датчика BUZZER модуля
  buzzer.begin(0x61); // С перемычкой адрес будет 0x60
  delay(250);

  // Инициализация таймеров
  // timer_main.setInterval(MAIN_TIMER, readSendData);
}

void loop()
{
  // Blynk.run();
  // timer_main.run();
  // timer_door.run();
  // timer_reset.run();
  // timer_alarm.run();
  // timer_internet.run();
}

/*
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
*/
