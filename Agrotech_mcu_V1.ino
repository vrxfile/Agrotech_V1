#define BLYNK_PRINT Serial

#include <rom/rtc.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <Preferences.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_APDS9960.h>
#include <I2C_graphical_LCD_display.h>
#include <BH1750FVI.h>
#include <VL53L0X.h>
#include <PCA9536.h>
#include <TLC59108.h>

// Точка доступа Wi-Fi
// char ssid[] = "MGBot";
char ssid[] = "AgroMGBOT";
char pass[] = "Terminator812";

// Параметры IoT сервера
char auth[] = "7782f301820d406396ee6202ec0d5c41";
IPAddress blynk_ip(139, 59, 206, 133);

// Датчик освещенности
BH1750FVI bh1750;

// Датчик температуры/влажности и атмосферного давления
Adafruit_BME280 bme;

// Датчик расстояния
VL53L0X lox;
//#define LONG_RANGE
#define HIGH_SPEED
// #define HIGH_ACCURACY

// Датчик положения
Adafruit_LSM9DS1 imu = Adafruit_LSM9DS1();

// Датчик приближения
Adafruit_APDS9960 apds9960;

// Модуль RGB светодиодов
#define HW_RESET_PIN 0 // Только програмнный сброс
#define I2C_ADDR TLC59108::I2C_ADDR::BASE
TLC59108 leds(I2C_ADDR + 7); // Без перемычек добавляется 3 бита адреса

// Микросхема управления реле
PCA9536 pca9536;

// LCD дисплей
I2C_graphical_LCD_display lcd;

// Датчик влажности почвы емкостной
#define SOIL_MOISTURE1      A4
#define SOIL_TEMPERATURE1   A5
#define SOIL_MOISTURE2      A6
#define SOIL_TEMPERATURE2   A7
const float air_value = 1590.0;
const float water_value = 830.0;
const float moisture_0 = 0.0;
const float moisture_100 = 100.0;

// Периоды таймеров
#define LCD_TIMER      10000 // Период для таймера LCD экрана
#define DOOR_TIMER     1000  // Период для таймера открытия дверцы
#define AUTO_TIMER     1000  // Период для таймера автоматических функций
#define MAIN_TIMER     10000 // Период для таймера обновления данных
#define ALARM_TIMER    100   // Период для таймера сигнализации удара
#define RESET_TIMER    60000 // Период для таймера общей перезагрузки
#define RELAY_TIMER    60000 // Период для таймеров управления реле

// Номера светодиодов RGB модуля
#define RLED   3
#define GLED   2
#define BLED   5
#define UVLED1 1
#define UVLED2 4

// Таймеры
BlynkTimer timer_lcd;
BlynkTimer timer_main;
BlynkTimer timer_door;
BlynkTimer timer_auto;
BlynkTimer timer_reset;
BlynkTimer timer_alarm;
BlynkTimer timer_relay;

// Настройки контроллера для счетчика включений
Preferences preferences;

// Данные с датчиков
static volatile float air_temp = 0.0;
static volatile float air_hum = 0.0;
static volatile float air_press = 0.0;
static volatile float light = 0.0;
static volatile float soil_hum1 = 0.0;
static volatile float soil_temp1 = 0.0;
static volatile float soil_hum2 = 0.0;
static volatile float soil_temp2 = 0.0;
static volatile float dist = 0.0;
static volatile float prox = 0.0;
static volatile float acc_xx = 0.0;
static volatile float acc_yy = 0.0;
static volatile float acc_zz = 0.0;
static volatile float acc_x0 = 0.0;
static volatile float acc_y0 = 0.0;
static volatile float acc_z0 = 0.0;
static volatile float sys_temp = 0.0;

// Статусы и флаги различных систем
static volatile int acc_status = 0x00;
static volatile int door_status = 0x00;
static volatile int relay1_status = 0x00;
static volatile int relay2_status = 0x00;
static volatile int wifi_status = 0x00;
static volatile int blynk_status = 0x00;
static volatile int i2c_status = 0x00;

// Счетчики событий различных систем
static volatile int acc_counter = 0x00;
static volatile int door_counter = 0x00;
static volatile int relay1_counter = 0x00;
static volatile int relay2_counter = 0x00;
static volatile int working_counter = 0x00;
static volatile int red_green_led = 0x00;
static volatile int pwr_counter = 0x00;

// Различные константы
const float acc_dd = 7.0;             // Пороги срабатывания по акселеромеру
const float door_dist = 4.0;          // Расстояние, при котором дверца закрыта
const int acc_max_counter = 30;       // Количество ударов до срабатывания сигнализации
const int max_reset_counter = 360;    // Количество минут до перезагрузки всей системы
const float min_soil_hum = 30;        // Минимальная влажность почвы

void setup()
{
  // Инициализация последовательного порта
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println();
  Serial.println();

  // Вывод в терминал сообщения о причине сброса контроллера
  Serial.println("CPU0 reset reason:");
  print_reset_reason(rtc_get_reset_reason(0));
  verbose_print_reset_reason(rtc_get_reset_reason(0));
  Serial.println();
  Serial.println("CPU1 reset reason:");
  print_reset_reason(rtc_get_reset_reason(1));
  verbose_print_reset_reason(rtc_get_reset_reason(1));
  Serial.println();

  // Счетчик включений контроллера
  preferences.begin("my-app", false);
  //preferences.clear();
  //preferences.remove("counter");
  pwr_counter = preferences.getUInt("counter", 0);
  pwr_counter++;
  preferences.putUInt("counter", pwr_counter);
  preferences.end();
  Serial.print("Power counter value: ");
  Serial.println(pwr_counter);
  Serial.println();

  // Инициализация I2C
  Wire.begin();
  Wire.setClock(20000L);

  // Инициализация модуля 4-х реле
  pca9536.reset();
  pca9536.setState(IO_LOW);
  pca9536.setMode(IO_OUTPUT);
  delay(250);

  // Инициализация RGB модуля
  leds.init(HW_RESET_PIN);
  leds.setLedOutputMode(TLC59108::LED_MODE::PWM_IND);

  // Включение красного светодиода
  byte pwm = 0x00; leds.setAllBrightness(pwm);
  pwm = 0x1F; leds.setBrightness(RLED, pwm);
  delay(250);

  // Инициализация LCD дисплея
  lcd.begin();
  lcd.clear (0, 0, 127, 63, 0x00);
  lcd.gotoxy (5, 10); lcd.string ("Waiting for WiFi...", false);

  // Ожидание запуска Wi-Fi роутера (45...60 секунд)
  Serial.print("Waiting for WiFi...");
  for (int i = 0; i < 10; i++)
  {
    delay(5000);
    Serial.print(i);
  }
  Serial.println();
  Serial.println();

  // Инициализация Wi-Fi и поключение к серверу Blynk
  lcd.clear (0, 0, 127, 63, 0x00);
  lcd.gotoxy (5, 10); lcd.string ("Connecting WiFi...", false);
  lcd.gotoxy (5, 25); lcd.string ("Starting Blynk...", false);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  Blynk.begin(auth, ssid, pass, blynk_ip, 8442);
  delay(1000);

  // Проверка подключения к Интернету
  lcd.clear (0, 0, 127, 63, 0x00);
  wifi_status = WiFi.status();
  blynk_status = Blynk.connected();
  if ((wifi_status == WL_CONNECTED) && (blynk_status))
  {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println();
    lcd.gotoxy (5, 10);
    lcd.string ("Connected", false);
    lcd.gotoxy (5, 25);
    lcd.string ("Internet is working", false);
  }
  else
  {
    Serial.println("");
    Serial.print("Error connection to Wi-Fi and Blynk");
    Serial.println("");
    lcd.gotoxy (5, 10);
    lcd.string ("Connection failed!", false);
  }

  // Включение синего светодиода
  pwm = 0x00; leds.setAllBrightness(pwm);
  pwm = 0x1F; leds.setBrightness(BLED, pwm);
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

  // Инициализация датчика LSM9DS1
  if (!imu.begin())
  {
    Serial.println("Unable to initialize the LSM9DS1. Check your wiring!");
  }
  else
  {
    imu.setupAccel(imu.LSM9DS1_ACCELRANGE_8G);
    imu.setupMag(imu.LSM9DS1_MAGGAIN_4GAUSS);
    imu.setupGyro(imu.LSM9DS1_GYROSCALE_245DPS);
    sensors_event_t a, m, g, t;
    imu.getEvent(&a, &m, &g, &t);
    acc_x0 = a.acceleration.x;
    acc_y0 = a.acceleration.y;
    acc_z0 = a.acceleration.z;
  }

  // Инициализация датчика VL53L0x
  /*
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
  */

  // Инициализация датчика приближения
  if (!apds9960.begin()) {
    Serial.println("Failed to initialize APDS-9960 device!");
  }
  apds9960.enableColor(true);
  apds9960.enableProximity(true);

  // Очистка всех индикаторов в Blynk
  Blynk.virtualWrite(V0, 0x00); delay(50);
  Blynk.virtualWrite(V1, 0x00); delay(50);
  Blynk.virtualWrite(V2, 0x00); delay(50);
  Blynk.virtualWrite(V3, 0x00); delay(50);
  Blynk.virtualWrite(V4, 0x00); delay(50);
  Blynk.virtualWrite(V5, 0x00); delay(50);
  Blynk.virtualWrite(V6, 0x00); delay(50);
  Blynk.virtualWrite(V7, 0x00); delay(50);
  Blynk.virtualWrite(V8, 0x00); delay(50);
  Blynk.virtualWrite(V9, 0x00); delay(50);
  Blynk.virtualWrite(V10, 0x00); delay(50);
  Blynk.virtualWrite(V11, 0x00); delay(50);
  Blynk.virtualWrite(V12, 0x00); delay(50);
  Blynk.virtualWrite(V13, 0x00); delay(50);
  Blynk.virtualWrite(V14, 0x00); delay(50);
  Blynk.virtualWrite(V15, 0x00); delay(50);
  Blynk.virtualWrite(V100, 0x00); delay(50);
  Blynk.virtualWrite(V101, 0x00); delay(50);
  Blynk.virtualWrite(V102, 0x00); delay(50);
  Blynk.virtualWrite(V103, 0x00); delay(50);
  Blynk.virtualWrite(V104, 0x00); delay(50);

  // Включение зеленого светодиода
  pwm = 0x00;
  leds.setAllBrightness(pwm);
  pwm = 0x1F;
  if ((wifi_status == WL_CONNECTED) && (blynk_status))
    leds.setBrightness(GLED, pwm);
  else
    leds.setBrightness(RLED, pwm);
  delay(250);

  // Очистка LCD экрана
  lcd.clear (0, 0, 127, 63, 0x00);

  // Инициализация таймеров
  timer_main.setInterval(MAIN_TIMER, readSendData);
  timer_door.setInterval(DOOR_TIMER, readDoorSensor);
  timer_auto.setInterval(AUTO_TIMER, doAutoFunctions);
  timer_alarm.setInterval(ALARM_TIMER, readIMUSensor);
  timer_relay.setInterval(RELAY_TIMER, doRelayFunctions);
  timer_reset.setInterval(RESET_TIMER, resetProcedure);
  timer_lcd.setInterval(LCD_TIMER, printLCDData);
}

void loop()
{
  Blynk.run();
  timer_lcd.run();
  timer_main.run();
  timer_door.run();
  timer_auto.run();
  timer_reset.run();
  timer_alarm.run();
  timer_relay.run();
}

// Датчик открытия дверцы
void readDoorSensor()
{
  uint16_t red_data   = 0;
  uint16_t green_data = 0;
  uint16_t blue_data  = 0;
  uint16_t clear_data = 0;
  uint16_t prox_data  = 0;
  if (apds9960.colorDataReady())
  {
    apds9960.getColorData(&red_data, &green_data, &blue_data, &clear_data);
    prox = apds9960.readProximity();
    // Serial.print("Door proximity = ");
    // Serial.println(prox);
    // Serial.println();
    if (prox <= door_dist)
    {
      if (door_status == 0x00)
        door_counter++;
      door_status = 0x01;
      Serial.println("Proximity sensor event");
      Serial.println();
    }
    else
    {
      door_status = 0x00;
    }
  }
}

// Акселерометр
void readIMUSensor()
{
  sensors_event_t a, m, g, t;
  imu.getEvent(&a, &m, &g, &t);
  sys_temp = t.temperature;
  acc_xx = fabs(a.acceleration.x - acc_x0);
  acc_yy = fabs(a.acceleration.y - acc_y0);
  acc_zz = fabs(a.acceleration.z - acc_z0);
  //  Serial.print("Accel X: "); Serial.print(acc_xx); Serial.print(" m/s^2");
  //  Serial.print("\tY: "); Serial.print(acc_yy); Serial.print(" m/s^2 ");
  //  Serial.print("\tZ: "); Serial.print(acc_zz); Serial.print(" m/s^2 ");
  //  Serial.print("\tTemp: "); Serial.print(sys_temp); Serial.println(" *C ");
  //  Serial.println();
  if ((acc_xx > acc_dd) || (acc_yy > acc_dd) || (acc_zz > acc_dd))
  {
    acc_counter++;
    Serial.print("Acceleromter event: ");
    Serial.println(acc_counter);
    Serial.println();
    if (acc_counter >= acc_max_counter)
    {
      acc_counter = 0x00;
      acc_status = 0x01;
    }
  }
}

// Считывание датчиков и отправка данных на сервер Blynk
void readSendData()
{
  // Считывание датчика температуры/влажности/давления
  air_temp = bme.readTemperature();
  air_hum = bme.readHumidity();
  air_press = bme.readPressure() * 7.5006 / 1000.0;
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
  light = bh1750.getAmbientLight();
  Serial.print("Light = ");
  Serial.println(light);
  Blynk.virtualWrite(V7, light); delay(25);             // Отправка данных на сервер Blynk

  // Считывание датчиков почвы
  float adc1_1 = (float)analogRead(SOIL_MOISTURE1);
  float adc1_2 = (float)analogRead(SOIL_TEMPERATURE1);
  float adc2_1 = (float)analogRead(SOIL_MOISTURE2);
  float adc2_2 = (float)analogRead(SOIL_TEMPERATURE2);
  soil_hum1 = map(adc1_1, air_value, water_value, moisture_0, moisture_100);
  soil_temp1 = ((adc1_2 / 4095.0 * 5.0) - 0.3) * 100.0;
  soil_hum2 = map(adc2_1, air_value, water_value, moisture_0, moisture_100);
  soil_temp2 = ((adc2_2 / 4095.0 * 5.0) - 0.3) * 100.0;
  if (soil_hum1 < 0)
    soil_hum1 = 0;
  if (soil_hum2 < 0)
    soil_hum2 = 0;
  if (soil_hum1 > 100)
    soil_hum1 = 100;
  if (soil_hum2 > 100)
    soil_hum2 = 100;
  Serial.print("Soil temperature 1 = ");
  Serial.println(soil_temp1);
  Serial.print("Soil moisture 1 = ");
  Serial.println(soil_hum1);
  Serial.print("Soil temperature 2 = ");
  Serial.println(soil_temp2);
  Serial.print("Soil moisture 2 = ");
  Serial.println(soil_hum2);
  Blynk.virtualWrite(V0, soil_hum1); delay(25);         // Отправка данных на сервер Blynk
  Blynk.virtualWrite(V1, soil_hum2); delay(25);         // Отправка данных на сервер Blynk
  Blynk.virtualWrite(V2, soil_temp1); delay(25);        // Отправка данных на сервер Blynk
  Blynk.virtualWrite(V3, soil_temp2); delay(25);        // Отправка данных на сервер Blynk

  // Отправка данных о состоянии различных систем
  Serial.print("Door open events: ");
  Serial.println(door_counter);
  Serial.print("Acceleromter events: ");
  Serial.println(acc_counter);
  Serial.print("Working counter: ");
  Serial.println(working_counter);
  Serial.print("Power counter value: ");
  Serial.println(pwr_counter);
  Serial.print("Inside temperature: ");
  Serial.println(sys_temp);
  Blynk.virtualWrite(V10, door_counter); delay(25);       // Отправка данных на сервер Blynk
  Blynk.virtualWrite(V11, acc_counter); delay(25);        // Отправка данных на сервер Blynk
  Blynk.virtualWrite(V13, working_counter); delay(25);    // Отправка данных на сервер Blynk
  Blynk.virtualWrite(V14, pwr_counter); delay(25);        // Отправка данных на сервер Blynk
  Blynk.virtualWrite(V15, sys_temp); delay(25);           // Отправка данных на сервер Blynk

  // Считывание лазерного датчика уровня воды
  // dist = lox.readRangeSingleMillimeters();
  dist = 0.0;
  Serial.print("Distance = ");
  Serial.println(dist);
  Blynk.virtualWrite(V12, dist); delay(25);        // Отправка данных на сервер Blynk

  Serial.println();
}

// Вывод некоторых данных на LCD экран
void printLCDData()
{
  char strtemp1[32] = "";
  char strtemp2[32] = "";
  dtostrf(air_temp, 4, 1, strtemp1);
  sprintf(strtemp2, "Air temp  = %s *C ", strtemp1);
  lcd.gotoxy(0, 0);
  lcd.string(strtemp2, false);
  dtostrf(air_hum, 4, 1, strtemp1);
  sprintf(strtemp2, "Air hum   = %s %% ", strtemp1);
  lcd.gotoxy(0, 10);
  lcd.string(strtemp2, false);
  dtostrf(air_press, 5, 1, strtemp1);
  sprintf(strtemp2, "Air press = %s mm ", strtemp1);
  lcd.gotoxy(0, 20);
  lcd.string(strtemp2, false);
  dtostrf(light, 5, 1, strtemp1);
  sprintf(strtemp2, "Light     = %s lx ", strtemp1);
  lcd.gotoxy(0, 30);
  lcd.string(strtemp2, false);
  dtostrf(soil_hum1, 4, 1, strtemp1);
  sprintf(strtemp2, "Soil hum1 = %s %% ", strtemp1);
  lcd.gotoxy(0, 40);
  lcd.string(strtemp2, false);
  dtostrf(soil_hum2, 4, 1, strtemp1);
  sprintf(strtemp2, "Soil hum2 = %s %% ", strtemp1);
  lcd.gotoxy(0, 50);
  lcd.string(strtemp2, false);
}

// Управление реле #1
BLYNK_WRITE(V100)
{
  // Получение управляющего сигнала с сервера
  relay1_status = param.asInt();
  Serial.print("Relay power 1: ");
  Serial.println(relay1_status);
  Serial.println();
  if (relay1_status)
    pca9536.setState(IO2, IO_HIGH);
  else
    pca9536.setState(IO2, IO_LOW);
}

// Управление реле #2
BLYNK_WRITE(V101)
{
  // Получение управляющего сигнала с сервера
  relay2_status = param.asInt();
  Serial.print("Relay power 2: ");
  Serial.println(relay2_status);
  Serial.println();
  if (relay2_status)
    pca9536.setState(IO3, IO_HIGH);
  else
    pca9536.setState(IO3, IO_LOW);
}

// Отключение сирены и статуса открытой двери
BLYNK_WRITE(V102)
{
  int signal_ctl = param.asInt();
  Serial.print("Signal status: ");
  Serial.println(signal_ctl);
  Serial.println();
  acc_status = 0x00;
  acc_counter = 0x00;
  door_status = 0x00;
  door_counter = 0x00;
  if (signal_ctl)
    pca9536.setState(IO1, IO_HIGH);
  else
    pca9536.setState(IO1, IO_LOW);
}

// Данные ползунка таймера реле #1
BLYNK_WRITE(V103)
{
  relay1_counter = param.asInt();
  Serial.print("Relay timer 1: ");
  Serial.println(relay1_counter);
  Serial.println();
}

// Данные ползунка таймера реле #2
BLYNK_WRITE(V104)
{
  relay2_counter = param.asInt();
  Serial.print("Relay timer 2: ");
  Serial.println(relay2_counter);
  Serial.println();
}

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

// Автоматические функции и различные проверки
void doAutoFunctions()
{
  byte pwm = 0x00;

  // Включение сирены по срабатыванию акселерометра
  if (acc_status)
    pca9536.setState(IO1, IO_HIGH);

  // Включение мигания светодиода при открытии дверцы
  if (door_counter)
  {
    red_green_led = 1 - red_green_led;
    pwm = 0x00;
    leds.setAllBrightness(pwm);
    pwm = 0x1F;
    if (red_green_led)
      leds.setBrightness(RLED, pwm);
    else
      leds.setBrightness(GLED, pwm);
  }
  else
  {
    if (red_green_led)
    {
      red_green_led = 0x00;
      pwm = 0x00;
      leds.setAllBrightness(pwm);
      pwm = 0x1F;
      leds.setBrightness(GLED, pwm);
    }
  }

  // Если влажность почвы ниже определенного порога - включение фиолетовых светодиодов
  if ((soil_hum1 < min_soil_hum) || (soil_hum2 < min_soil_hum))
  {
    pwm = 0x1F;
    leds.setBrightness(UVLED1, pwm);
    leds.setBrightness(UVLED2, pwm);
  }
  else
  {
    pwm = 0x00;
    leds.setBrightness(UVLED1, pwm);
    leds.setBrightness(UVLED2, pwm);
  }
}

// Функции и таймеры для управления реле
void doRelayFunctions()
{
  // Уменьшение счетчика работы реле 1
  if ((relay1_counter > 0) && (relay1_status))
  {
    relay1_counter--;
    Serial.print("Relay counter 1: ");
    Serial.println(relay1_counter);
    Blynk.virtualWrite(V103, relay1_counter);
    delay(25);
  }
  // Отключение реле 1 по окончании работы счетчика
  if ((relay1_counter <= 0) && (relay1_status))
  {
    relay1_status = 0x00;
    pca9536.setState(IO2, IO_LOW);
    Blynk.virtualWrite(V100, relay1_status);
    delay(25);
  }

  // Уменьшение счетчика работы реле 2
  if ((relay2_counter > 0) && (relay2_status))
  {
    relay2_counter--;
    Serial.print("Relay counter 2: ");
    Serial.println(relay2_counter);
    Blynk.virtualWrite(V104, relay2_counter);
    delay(25);
  }
  // Отключение реле 2 по окончании работы счетчика
  if ((relay2_counter <= 0) && (relay2_status))
  {
    relay2_status = 0x00;
    pca9536.setState(IO3, IO_LOW);
    Blynk.virtualWrite(V101, relay2_status);
    delay(25);
  }

  Serial.println();
}

// Процедура перезагрузки
void resetProcedure()
{
  working_counter++;
  Serial.print("Working counter: ");
  Serial.println(working_counter);
  Serial.println();
  if (working_counter >= max_reset_counter)
  {
    working_counter = 0;
    Serial.println("Reseting system!");
    Serial.println();
    delay(250);
    ESP.restart();
  }
}
