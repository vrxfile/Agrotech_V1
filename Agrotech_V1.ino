#define BLYNK_PRINT Serial

#include <rom/rtc.h>
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

#define RLED 3
#define GLED 2
#define BLED 4

void setup()
{
  // Инициализация последовательного порта
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println();
  Serial.println();

  // Вывод в терминал сообщения о причине сброса контроллера
  Serial.println("CPU0 reset reason:");
  print_reset_reason(rtc_get_reset_reason(0));
  Serial.println();
  verbose_print_reset_reason(rtc_get_reset_reason(0));
  Serial.println("CPU1 reset reason:");
  print_reset_reason(rtc_get_reset_reason(1));
  verbose_print_reset_reason(rtc_get_reset_reason(1));
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

  // Инициализация RGB модуля
  leds.init(HW_RESET_PIN);
  leds.setLedOutputMode(TLC59108::LED_MODE::PWM_IND);
  byte pwm = 0x00; leds.setAllBrightness(pwm);

  // Включение красного светодиода
  pwm = 0xFE; leds.setBrightness(RLED, pwm);
  delay(250);

  // Инициализация LCD дисплея
  lcd.begin();
  lcd.clear (0, 0, 127, 63, 0x00);
  lcd.gotoxy (5, 5); lcd.string ("Waiting for WiFi...", false);

  // Ожидание запуска Wi-Fi роутера (45...60 секунд)
  // delay(30000);
  // delay(30000);

  // Инициализация Wi-Fi и поключение к серверу Blynk
  lcd.clear (0, 0, 127, 63, 0x00);
  lcd.gotoxy (5, 10); lcd.string ("Connecting WiFi...", false);
  lcd.gotoxy (5, 35); lcd.string ("Starting Blynk...", false);
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

  // Включение синего светодиода
  pwm = 0xFE; leds.setBrightness(BLED, pwm);
  delay(250);

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

  // Инициализация датчика BUZZER модуля
  buzzer.begin(0x61); // С перемычкой адрес будет 0x60
  delay(250);

  // Включение зеленого светодиода
  pwm = 0xFE; leds.setBrightness(RLED, pwm);
  delay(250);

  // Инициализация таймеров
  timer_main.setInterval(MAIN_TIMER, readSendData);
  timer_door.setInterval(DOOR_TIMER, readDistSensor);
}

void loop()
{
  Blynk.run();
  timer_main.run();
  timer_door.run();
  // timer_reset.run();
  // timer_alarm.run();
  // timer_internet.run();
}

void readDistSensor()
{
  float dist = lox.readRangeSingleMillimeters();
  Serial.print("Distance = ");
  Serial.println(dist);
}

// Считывание датчиков и отправка данных на сервер Blynk
void readSendData()
{
  // Считывание датчика температуры/влажности/давления
  float air_temp = bme.readTemperature();
  float air_hum = bme.readHumidity();
  float air_press = bme.readPressure() * 7.5006 / 1000.0;
  Serial.print("Air temperature = ");
  Serial.println(air_temp);
  Serial.print("Air humidity = ");
  Serial.println(air_hum);
  Serial.print("Air pressure = ");
  Serial.println(air_press);
  Blynk.virtualWrite(V4, air_temp); delay(25);        // Отправка данных на сервер Blynk
  Blynk.virtualWrite(V5, air_hum); delay(25);         // Отправка данных на сервер Blynk
  Blynk.virtualWrite(V6, air_press); delay(25);       // Отправка данных на сервер Blynk

  // Считывание датчика света
  float light = bh1750.getAmbientLight();
  Serial.print("Light = ");
  Serial.println(light);
  Blynk.virtualWrite(V7, light); delay(25);             // Отправка данных на сервер Blynk

  // Датчиков почвы
  //  float adc1_1 = (float)ads1015.readADC_SingleEnded(0);
  //  float adc1_2 = (float)ads1015.readADC_SingleEnded(1);
  //  float soil_hum = map(adc1_1, air_value, water_value, moisture_0, moisture_100);
  //  float soil_temp = adc1_2 / 10.0;
  //  Serial.print("Soil temperature = ");
  //  Serial.println(soil_temp);
  //  Serial.print("Soil moisture = ");
  //  Serial.println(soil_hum);
  //  Blynk.virtualWrite(V2, soil_temp); delay(25);        // Отправка данных на сервер Blynk
  //  Blynk.virtualWrite(V3, soil_hum); delay(25);         // Отправка данных на сервер Blynk

  Serial.println();
}
/*
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

// Вывод сообщения о причине сброса контроллера
void print_reset_reason(RESET_REASON reason)
{
  switch ( reason)
  {
    case 1 : Serial.println ("POWERON_RESET"); break;         /**<1,  Vbat power on reset*/
    case 3 : Serial.println ("SW_RESET"); break;              /**<3,  Software reset digital core*/
    case 4 : Serial.println ("OWDT_RESET"); break;            /**<4,  Legacy watch dog reset digital core*/
    case 5 : Serial.println ("DEEPSLEEP_RESET"); break;       /**<5,  Deep Sleep reset digital core*/
    case 6 : Serial.println ("SDIO_RESET"); break;            /**<6,  Reset by SLC module, reset digital core*/
    case 7 : Serial.println ("TG0WDT_SYS_RESET"); break;      /**<7,  Timer Group0 Watch dog reset digital core*/
    case 8 : Serial.println ("TG1WDT_SYS_RESET"); break;      /**<8,  Timer Group1 Watch dog reset digital core*/
    case 9 : Serial.println ("RTCWDT_SYS_RESET"); break;      /**<9,  RTC Watch dog Reset digital core*/
    case 10 : Serial.println ("INTRUSION_RESET"); break;      /**<10, Instrusion tested to reset CPU*/
    case 11 : Serial.println ("TGWDT_CPU_RESET"); break;      /**<11, Time Group reset CPU*/
    case 12 : Serial.println ("SW_CPU_RESET"); break;         /**<12, Software reset CPU*/
    case 13 : Serial.println ("RTCWDT_CPU_RESET"); break;     /**<13, RTC Watch dog Reset CPU*/
    case 14 : Serial.println ("EXT_CPU_RESET"); break;        /**<14, for APP CPU, reseted by PRO CPU*/
    case 15 : Serial.println ("RTCWDT_BROWN_OUT_RESET"); break; /**<15, Reset when the vdd voltage is not stable*/
    case 16 : Serial.println ("RTCWDT_RTC_RESET"); break;     /**<16, RTC Watch dog reset digital core and rtc module*/
    default : Serial.println ("NO_MEAN");
  }
}

// Расшифровка сообщения о причине сброса контроллера
void verbose_print_reset_reason(RESET_REASON reason)
{
  switch ( reason)
  {
    case 1  : Serial.println ("Vbat power on reset"); break;
    case 3  : Serial.println ("Software reset digital core"); break;
    case 4  : Serial.println ("Legacy watch dog reset digital core"); break;
    case 5  : Serial.println ("Deep Sleep reset digital core"); break;
    case 6  : Serial.println ("Reset by SLC module, reset digital core"); break;
    case 7  : Serial.println ("Timer Group0 Watch dog reset digital core"); break;
    case 8  : Serial.println ("Timer Group1 Watch dog reset digital core"); break;
    case 9  : Serial.println ("RTC Watch dog Reset digital core"); break;
    case 10 : Serial.println ("Instrusion tested to reset CPU"); break;
    case 11 : Serial.println ("Time Group reset CPU"); break;
    case 12 : Serial.println ("Software reset CPU"); break;
    case 13 : Serial.println ("RTC Watch dog Reset CPU"); break;
    case 14 : Serial.println ("for APP CPU, reseted by PRO CPU"); break;
    case 15 : Serial.println ("Reset when the vdd voltage is not stable"); break;
    case 16 : Serial.println ("RTC Watch dog reset digital core and rtc module"); break;
    default : Serial.println ("NO_MEAN");
  }
}
