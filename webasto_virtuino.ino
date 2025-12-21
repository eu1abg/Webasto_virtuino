// ядро ЕСП32 3.00
#include <AutoOTA.h>
AutoOTA ota("3.0", "eu1abg/Webasto_virtuino"); // eu1abg/Webasto_virtuino   https://github.com/GyverLibs/AutoOTA
bool obn=0;       // флаг обновления
#define Kline 0  // берем данные из вебасты по Клинии 1. датчики внешнии 0.
#define Rele 1   //  1 используем реле для запуска вебасты.  0 по Клинии.
#define dshim1 5   //  5 используем для настройки нуля печки мотор

#define uS_TO_S_FACTOR 1000000ULL  /* преобразуем микросек в сек*/
#define TIME_TO_SLEEP  3       /* время сна */
#define TIME_POWER_SLEEP  6      /* время на приеме */
//===========================================================================================================================================
//#include <ESP8266WiFi.h>
#include <WiFi.h>
//==================================================================
#include "GyverPID.h"
GyverPID regulator1(0.5, 0.5, 0.1);  // Печка   коэф. П, коэф. И, коэф. Д, период дискретизации dt (мс)
GyverPID regulator2(2, 1, 0.1);  // Помпа
 //==================================================================
#include <SimplePortal.h>
#include <EEPROM.h>
//==================================================================
#include <PubSubClient.h>
#include <WiFiClient.h>
//===========================================================================================================================================
#include <NTPClient_Generic.h>          // https://github.com/khoih-prog/NTPClient_Generic
#include <WiFiUdp.h>
WiFiUDP ntpUDP;
 #define TIME_ZONE_OFFSET_HRS            (3)
#define SECS_IN_HR                (3600L)
NTPClient timeClient(ntpUDP);
//=========================================================================================================================================== 
#include "esp_bt.h"
#include "esp_chip_info.h"

//=============================================================================================================

#include <DS18B20.h>
DS18B20 sensor1(16); // температура салона
DS18B20 sensor2(17); // температура антифриза
//======================================================================
#include "esp_task_wdt.h"
//=========================================================================================================================================== 
#include <Wire.h>
#define I2C_SDA 32
#define I2C_SCL 25
#include <GyverOLED.h>
//GyverOLED<SSD1306_128x64, OLED_NO_BUFFER> oled;
GyverOLED<SSH1106_128x64> oled;
//===========================================================================================================================================
#define CLK 35
#define DT 34
#define SW 33
#include "GyverEncoder.h"
Encoder enc1(CLK, DT, SW);
//===========================================================================================================================================
#include <HardwareSerial.h>
HardwareSerial kLineSerial(1);

const int baudRate = 10400;  // Скорость для K-Line 10400
const int rxPin = 14;        // Пин для приема данных K-Line
const int txPin = 34;        // Пин для передачи данных K-Line

  byte Wakeup[] = {0x81, 0x51, 0xF1, 0xA1, 0x64};
   byte Init1[] = {0x81, 0x51, 0xF1, 0x81, 0x44};
   byte Init2[] = {0x82, 0x51, 0xF1, 0x3C, 0x00, 0x00};
byte Request1[] = {0x83, 0x51, 0xF1, 0x2A, 0x01, 0x01, 0xF1};  // получаем напр и темп
byte Request2[] = {0x83, 0x51, 0xF1, 0x2A, 0x01, 0x02, 0xF2};  //*2 запрос на статические данные ( текущее сост, ошибки, и т.д..)
byte Request3[] = {0x83, 0x51, 0xF1, 0x2A, 0x01, 0x05, 0xF5};  //*3 запрос на статические данные (версия прошивки, тип топлива, и т.д..)
byte Request4[] = {0x83, 0x51, 0xF1, 0x31, 0x22, 0xFF, 0x17};  //  вкл вебасты
byte Request5[] = {0x83, 0x51, 0xF1, 0x31, 0x22, 0x01, 0x19};
byte Request6[] = {0x83, 0x51, 0xF1, 0x31, 0x22, 0x00, 0x18};  //  выкл вебасты
byte Request7[] = {0x83, 0x51, 0xF1, 0x2A, 0x01, 0x02, 0xF2};  //  диагностика

// 88 F1 51 6A 02 00 00 00 00 00 02 38 (на выключенном котле). В нём: 6й байт — состояние нагнетателя, 7й — штифта накаливания, 8й — циркуляционного насоса,
//  9й — топливного насоса. 10 и 11 байты — состояние котла. При полностью выключенном котле это будет 00 02, в режиме опроса датчика пламени 11 12, разогрева — 17 32, 
//  подачи топлива 1F 32, рабочего цикла 1D 32, продувки 05 12.
// В принципе, этого достаточно. Индикация пламени — опрос 11 байта, если там 02 или 12, то пламени нет, 
// если 32, то пламя есть. Индикация выхода на рабочий режим — 10 байт, если там 1D, то вышли на рабочий режим, если что-то другое — то пока ещё нет (или уже всё).

byte Answer[18]; // вообще говоря, в ответе 11 байт. Но ещё 7 байт придут перед ответом, это сам запрос, т.к. в протоколе K-Line присутствует эхо


//===========================================================================================================================================

#include <TimerMs.h>
TimerMs tmr1(5000, 1, 0);   // отправляем топики 
TimerMs tmr2(7000, 1, 0);   // АКБ
TimerMs tmr3(180000, 1, 0);   // обновление
TimerMs tmr4;   //  ;
TimerMs tmr5(60000, 5, 0); 
TimerMs tmr6(60000, 1, 0);  // притухает экран
TimerMs tmr7(300000, 1, 0);   // экран откл
TimerMs tmr8;   // выход из меню
TimerMs tmr9((TIME_POWER_SLEEP*1000), 1, 0);   //  таймер перехода  в спящий режим
 TimerMs tmr10(30000, 1, 0); // опрос вебасты в ждущем режиме
// //=====================================================
//const char* ssid = "EPS-Minsk.by";
//const char* password = "13051973";

const char* mqtt_server = "m6.wqtt.ru"; // replace with your broker url
const char* mqtt_username = "eu1abg";  // you don't need the username and password for public connection
const char* mqtt_password = "13051973";
const int mqtt_port = 14516;

WiFiClient espClient;
PubSubClient client(espClient);
//==============================================================================================================================
//#include "GyverTimers.h"   //  https://alexgyver.ru/gyvertimers/

//==============================================================================================================================
#include "esp_adc_cal.h"
#define VREF 3.37
#define DIV_R1 4600
#define DIV_R2 980
#define AKB_PIN 36
esp_adc_cal_characteristics_t adc_chars;
//==============================================================================================================================
int tonePin = 4;
unsigned long lastMsg = 0; uint32_t sec; uint32_t timer; uint32_t minutes; uint32_t seconds; uint32_t hours; uint32_t timerwifi;
int shim1 = 0; //  авто шим отопитель
int shim2 = 0; // авто шим помпа
int shim21 = 0;
float ts = 0;  // температура салон
float ta = 0; // темпер антифриз
int n=0;      // перекл экрана
int n1 = 0;    // счетчик 
int n2 = 0;  // счетчик 
float vakb ;   //  измеряемое напряжение
float v1;
float v = 10.7; // порог напряжения 10.710
float v2 = 11.7; // верхний порог напряжения 11.7
int batlow ; int vklweb; int vklpomp; int switch1; String web_time; String stroka; int vkl1; int tz=0;
//--------------------------------------------------
float R1=0; float R2=0;   int mode=0;
//--------------------------------------------------
int shift; int nagn; int pomp; int tpomp; int ign; int sostweb;
//--------------------------------------------------
//int timer = 900; // время авто выключения
//int timer1 = 120; // время подготовки
char buffer[100]; bool x=0; bool x1=0; int x6=0; int tust; bool ventil;
int portal=0; uint32_t timerwifi33;
int RRSI; String nagrev="Откл.Нагр."; 
bool ekrON=0; int m=0; int m1=0; int m2=18;
const unsigned long sleepp = 5; // 30 секунд в миллисекундах
bool slepper=0; bool menu=0,ob;
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//#define LED 2 //светодиод на gpio2
bool on = false; //флаг состояния светодиода
//-----------------------------------------------------------------------------------------------------------------


//--------------------------------ТОПИКИ и АйДи ---------------------------------------------------------------------------------
String incommingMessage=""; String numar = ""; uint32_t chipId = 0; 
String  RRSI_top;        const char* RRSI_topic;
String ts_top;           const char* ts_topic;                    
String ta_top;           const char* ta_topic;                  
String vakb_top;         const char* vakb_topic;                
String shim21_top;       const char* shim21_topic;                      
String vklweb_top;       const char* vklweb_topic;
String vklpomp_top;      const char* vklpomp_topic;
String tust_top;         const char* tust_topic;
String shim1_top;        const char* shim1_topic;
String stroka_top;       const char* stroka_topic;
String switch12_top;     const char* switch12_topic;
String ventil_top;       const char* ventil_topic;
//-----------------------------------------------------------------------------------------------------------------
String switch1_top;      const char* switch1_topic;
String shim2_top;        const char* shim2_topic;
String tust1_top;        const char* tust1_topic;
//String edit11_topic="fen_edit11";
//------------------------------------------------------------------------------------------------------
void preSetupChipId() {
  uint64_t mac = ESP.getEfuseMac();
  chipId = (uint32_t)(mac >> 24);  // Берем 4 байта MAC-адреса
  
 RRSI_top = String(chipId)+"/"+"web_RRSI";        RRSI_topic = RRSI_top.c_str();
 ts_top = String(chipId)+"/"+"web_ts";            ts_topic = ts_top.c_str() ;                    
 ta_top = String(chipId)+"/"+"web_ta";            ta_topic =  ta_top.c_str();                  
 vakb_top = String(chipId)+"/"+"web_vakb";        vakb_topic = vakb_top.c_str();                
 shim21_top = String(chipId)+"/"+"web_shim21";    shim21_topic = shim21_top.c_str();                      
 vklweb_top = String(chipId)+"/"+"web_vklweb";    vklweb_topic = vklweb_top .c_str();
 vklpomp_top = String(chipId)+"/"+"web_vklpomp";  vklpomp_topic = vklpomp_top.c_str();
 tust_top = String(chipId)+"/"+"web_tust";        tust_topic = tust_top.c_str();
 shim1_top = String(chipId)+"/"+"web_shim1";      shim1_topic = shim1_top.c_str();
 stroka_top = String(chipId)+"/"+"web_stroka";    stroka_topic = stroka_top.c_str();
 switch12_top =String(chipId)+"/"+"web_vkl1";     switch12_topic = switch12_top.c_str();
 ventil_top = String(chipId)+"/"+"web_ventil";    ventil_topic = ventil_top.c_str();
//-----------------------------------------------------------------------------------------------------------------
switch1_top = String(chipId)+"/"+"web_vkl";       switch1_topic = switch1_top.c_str();
shim2_top = String(chipId)+"/"+"web_shim2";       shim2_topic = shim2_top.c_str();
tust1_top = String(chipId)+"/"+"web_tust1";       tust1_topic = tust1_top.c_str();

}
//===================================================================
void __attribute__((constructor)) beforeSetup() {preSetupChipId();}
//=====================================================================



//=======================ПРЕРЫВАНИЕ апаратное по 12 ноге ================================================
IRAM_ATTR void myIsr() {
enc1.tick();  // отработка в прерывании

}
//==============================================================================================================================

void setup() {
 Serial.begin(115200);Wire.begin(I2C_SDA, I2C_SCL, 100000); EEPROM.begin(500); oled.init(); oled.clear(); oled.setScale(1); oled.setContrast(200); 
 //=============================================================================================================================
//+++++++++++++++++++++++++++++++++++++++++ Кнопки ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 enc1.setTickMode(AUTO);
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
randomSeed(micros());
  Serial.println("\nWiFi connected\nIP address: ");
  Serial.println(WiFi.localIP());
  oled.setCursor(15, 0);oled.print(" WiFi.localIP ");oled.setCursor(15, 1);oled.print(WiFi.localIP());
  oled.setCursor(0, 3);oled.print(" Labadasto Version "); oled.setCursor(45, 4);  oled.invertText(1); oled.print(ota.version());oled.invertText(0);
    oled.setCursor(10, 6);oled.print(" Chip ID:"); oled.print(chipId);
  oled.update(); delay(5000); oled.clear(); oled.update();
//========================================================================================
  pinMode(36, INPUT);  // напряжение
  analogReadResolution(12);
  analogSetPinAttenuation(AKB_PIN, ADC_11db);

  esp_adc_cal_characterize(
    ADC_UNIT_1,
    ADC_ATTEN_DB_11,
    ADC_WIDTH_BIT_12,
    1100,           // стандарт для Mini
    &adc_chars);
//------------------------------------------------------
  ini(1); 
  pinMode(22, OUTPUT); // вентилятор отопителя
  pinMode(19, OUTPUT); // вкл вебасты
  pinMode(18, OUTPUT);  // выкл климат и вкл шим климат
  pinMode(21, OUTPUT); //ledcAttachPin(21, 1);  // шим помпа
  pinMode(23, OUTPUT); //ledcAttachPin(23, 2);  // шим вентиль отопителя 400Гц

  pinMode(33, INPUT_PULLUP);  // кнопка вкл ручное управление
  //pinMode(16, INPUT);  // t1
  //pinMode(17, INPUT);  // t2
  pinMode(2, OUTPUT);  //  светодиод режим

  //ledcSetup(1, 1000, 8); ledcSetup(2, 400, 8);
  ledcAttach(21, 1000, 8); ledcAttach(23, 400, 8);
  
  regulator1.setDirection(NORMAL); // ПЕЧКА   направление регулирования (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
  regulator1.setLimits(20, 100);    // пределы (ставим для 8 битного ШИМ). ПО УМОЛЧАНИЮ СТОЯТ 0 И 255
  regulator1.setpoint = 22;        // сообщаем регулятору температуру, которую он должен поддерживать 

  regulator2.setDirection(NORMAL); // ПОМПА     направление регулирования (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
  regulator2.setLimits(75, 250);    // пределы (ставим для 8 битного ШИМ). ПО УМОЛЧАНИЮ СТОЯТ 0 И 255
  regulator2.setpoint = 22;        // сообщаем регулятору температуру, которую он должен поддерживать 
//===========================================================================================================================================  
  WIFISEL();
//==============================================================================================================================
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
  //==============================================================================================================================
  tone(tonePin, 783, 200);
//===========================================================================================================================================          
kLineSerial.begin(baudRate, SERIAL_8N1, rxPin, txPin);
//=========================================================================================================================================
 EEPROM.get(300, tust); 
//-------------------------- SLEEP --------------------------------------------------------------------------
 //esp_bluedroid_disable();  https://microsin.net/programming/arm/esp32-sleep-modes.html
 //esp_bt_controller_disable();
 //esp_sleep_enable_wifi_wakeup(); // Разрешает пробуждение от WiFi MAC.

 attachInterrupt(33, myIsr, CHANGE); // прерывание по кнопке
 esp_sleep_enable_ext0_wakeup(GPIO_NUM_33,0); //1 = High, 0 = Low  пробуждение от сна    по кнопке
//-------------------------- прерывание по таймеру встроеному не зависит от проца  ---------------------------------------------------------------------
//  noInterrupts();
//  timer_isr_init(); //
//  timer_attachInterrupt(Flag); //настраиваем прерывание (привязка к функции)
//  timer_write(ESP.getCycleCount() + 160000000L); //Тактовая частота 80MHz, получаем секунду
//  interrupts();
/////////////// Настройка конфигурации watchdog////////////////////////
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 30000,           // 10 секунд
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, // Все ядра
        .trigger_panic = true,         // Сброс при зависании
    };
    
    esp_task_wdt_init(&twdt_config);
    esp_task_wdt_add(NULL);

 //----------------------------------------------------------------------------------------------------
}
void loop() {  
esp_task_wdt_reset(); // Сбрасываем watchdog 
client.loop(); obnovl();
//==================================================================== 
   if(digitalRead(19) ==0) vklweb=0; else vklweb=1;
//======================================================================   
   if (tmr3.tick() ) ob=1;   //  проверяем обнову

//====================================================================
 if (!client.connected() ) reconnect();
   
 //oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo 
 if (tmr2.tick() ) {if (Kline==0) akb(); }//timeClient.update(); menu();
 //oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
//============================================ МЕНЮ =============================================================================================
if (enc1.isPress() and (m1==0)) { ekrON=1; slepper=0; tone(tonePin, 783, 165.441); } 
//if (butt1.isDouble()  and (m1==0)) { ekrON=1; tone(16, 1000, 165.441); } 

if(m1==0){
if (enc1.isHolded()){ n=1; m1=1; ekrON=1; tone(tonePin, 1000, 1000); oled.clear();oled.setScale(2); oled.setCursor(10, 0);oled.invertText(1); oled.print("  Меню.  ");menu=1; oled.invertText(0);oled.setScale(1); }
 tmr8.setTime(30000);tmr8.start();}
else if (m1==1) { 
   if (enc1.isRelease())  { m=m+1; oled.setContrast(200); tone(tonePin, 1500, 100);tmr8.start();if (m>7) {m=0;} }

switch (m) { 
  case 0: oled.setCursor(15, 4); oled.print(String("")+(m+1)+"."+" Все выкл.       "); if (enc1.isHolded() and (m1=1))  {switch1=0; m1=0; n=0; tone(tonePin, 1000, 1000);tone(tonePin, 2500, 100);oled.clear();} break;
  case 1: oled.setCursor(15, 4); oled.print(String("")+(m+1)+"."+" Подготовка.     "); if (enc1.isHolded() and (m1=1))  {switch1=1;  m1=0; n=0;tone(tonePin, 1000, 1000);tone(tonePin, 2500, 100);oled.clear();} break;
  case 2: oled.setCursor(15, 4); oled.print(String("")+(m+1)+"."+" Прогр.двигла    "); if (enc1.isHolded() and (m1=1))  {switch1=2;  m1=0; n=0;tone(tonePin, 1000, 1000);tone(tonePin, 2500, 100); oled.clear();} break;
  case 3: oled.setCursor(15, 4); oled.print(String("")+(m+1)+"."+" Ручное упр.     "); if (enc1.isHolded() and (m1=1))  {switch1=3;  m1=0; n=0;tone(tonePin, 1000, 1000);tone(tonePin, 2500, 100); oled.clear();} break;
  case 4: oled.setCursor(15, 4); oled.print(String("")+(m+1)+"."+" Авто.           "); if (enc1.isHolded() and (m1=1))  {switch1=4;  m1=0; n=0;tone(tonePin, 1000, 1000);tone(tonePin, 2500, 100); oled.clear();} break;
  case 5: oled.setCursor(15, 4); oled.print(String("")+(m+1)+"."+" Уст.темп.АВто.  "); if (enc1.isHolded() and (m1=1))  { m1=2; n=1; tone(tonePin, 2500, 200); } break;
  case 6: oled.setCursor(15, 4); oled.print(String("")+(m+1)+"."+" ID "+ String(chipId)+"     ");break;
  case 7: oled.setCursor(15, 4); oled.print(String("")+(m+1)+"."+" Reset.          "); if (enc1.isHolded() and (m1=1))  {  tone(tonePin, 2500, 200);esp_restart(); } break;
 }
 }
if (m1==2) {
oled.setCursor(20, 4); oled.print("Уст.темп."); oled.setCursor(20, 6); oled.print("Температура = ");oled.print(m2);
if (enc1.isRelease())  { m2=m2+1; tone(tonePin, 1500, 100); tmr8.start(); if (m2>25) {m2=18;} }
if (enc1.isHolded() and (m1==2))  { EEPROM.put(300,m2); EEPROM.commit(); EEPROM.get(300, tust);  m1=0; n=0; tone(tonePin, 1000, 1000);tone(tonePin, 2500, 100); oled.clear();}

}
if (tmr8.tick()) {m1=0;n=0; menu=0; tone(tonePin, 3000, 100);tone(tonePin, 300, 500);oled.clear();}
//oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
// Реже выполняем тяжелые операции
   
  if (Kline ==0 ) {
  //  if (sensor1.ready()) ts = sensor1.getTemp();
  //  if (sensor2.ready()) ts = sensor2.getTemp();
   ts = sensor1.getTempC();  ta = sensor2.getTempC();
    }
      else {  //if (sensor1.ready()) ts = sensor1.getTemp();
        ts = sensor1.getTempC();
          if (slepper==0)  data(1); else if (tmr10.tick()) data(1); }     // в ждущем режиме опрос кдинии по таймеру 10
//oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
  
//---------------------------------------------------------------------------------------------------------
 if (tmr1.tick() ) { digitalWrite(2,HIGH);
   RRSI= WiFi.RSSI(); EEPROM.get(300, tust); 
 publishMessage(RRSI_topic,String(RRSI),true); 
 publishMessage(ts_topic,String(ts),true);    
 publishMessage(ta_topic,String(ta),true);
 publishMessage(vakb_topic,String(vakb),true);
 publishMessage(shim21_topic,String(shim1-dshim1),true);  //  
 publishMessage(vklweb_topic,String(vklweb),true);
 publishMessage(vklpomp_topic,String(vklpomp),true);
 publishMessage(tust_topic,String(EEPROM.read(300)),true);     //publishMessage(tust_topic,String(tust),true); 
 publishMessage(shim2_topic,String(shim2*0.39),true); //  
 publishMessage(stroka_topic,stroka.c_str(),true);
 publishMessage(switch12_topic,String(switch1),true);  
 publishMessage(ventil_topic,String(ventil),true);
 digitalWrite(2,LOW);
 //Serial.print("SLEEPER = ");Serial.println(slepper);
//============================================================================

//===========================================================================
 }
//---------------------------------------------------------------------------------------------------------
 
          //regulator1.input = sensor1.getTemp();regulator2.input = sensor1.getTemp();    
 //oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
 if (Kline ==0 ) {
    if (digitalRead(19)==0) {nagrev="Откл.Нагр."; }
     else nagrev=" Нагрев! ";
 }
 else {  if (sostweb == 0) {nagrev="Откл.Нагр."; }
  else nagrev=" Нагрев! "; }                       // нужен анализ битов из вебасты

 //ooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo-- 
if (switch1 == 0)   // все выкл
   {  vklpomp =0;  shim2 = 0; shim1=0; stop(); stroka ="Ver. "+ ota.version() +".  "+"  Сделайте выбор "; // tust = 22; stroka ="Сделайте выбор";
    //vkl1 =0;
    }
//--------------------------------------------------------------------------------------------------------- 
 if ((switch1 == 1) && (batlow == 0))  // вкл подготовка к запуску
{  digitalWrite(18,HIGH); timers(); stroka = "Подготовка 5 мин.  " + web_time;
    vklpomp =0; shim1 = 0;  shim2 = 0; ///tust = 22; //vkl1 = 1;
   if( minutes == 5 && switch1==1) {stop(); switch1 = 0; x1=0; stroka ="Ver. "+ ota.version() +".  "+"  Сделайте выбор "; }
  }

//-----------------------------ПРОГРЕВ---------------------------------------------------------------------------- 
if ((switch1 == 2) && (batlow == 0))  // вкл только нагрев и помпа 80пр
  { 
    shim1 = 0; // печка
    shim2 = 220; // помпа
    //oooooooooooooooooooooooooooooooooooooooooooooooooooooooo
   if (Rele==1) digitalWrite(19, HIGH); else webstart();// вкл вебасты
   //ooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
      ledcWrite(21,shim2); // шим pompa
      ledcWrite(23,shim1);  //шим вентиль отопителя 400Гц
   digitalWrite(22,LOW); // вентилятор отопителя
   digitalWrite(18,LOW);
    
   timers(); 
    stroka = "Прогр. 15 мин. " + web_time;  vklpomp = 1;
   if( minutes == 15 && switch1==2) {stop(); switch1 = 0; x1=0; stroka ="Ver. "+ ota.version() +".  "+"  Сделайте выбор ";  }
   }
//-----------------------------------РУЧНОЕ---------------------------------------------------------------------- 
if ((switch1== 3) && (batlow == 0))  //  вкл нагрев  помпа на регулятор и ручное управление салонным вентилятором
  {  stroka = " Ручное управление " + web_time;  vklpomp = 1;

    if ( (ta < 69) && (tz==0) ) {shim2 = 250; if (Rele==1) {digitalWrite(19,HIGH);}else webstart(); }
            
      else { if (Rele==1) {digitalWrite(19,LOW); } else {webstop();}  tz=1; regulator2.setpoint = tust+3; regulator2.input = ts; shim2 = regulator2.getResultTimer();}
       timers();
   if (ta < 35 ) tz=0; 
    //oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo 
    //if (Rele==1) digitalWrite(19,HIGH); else webstart(1);
    //oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
      ledcWrite(21,shim2);  // шим помпа
      ledcWrite(23,(shim1*1.55));  // шим вентиль отопителя 400Гц
    digitalWrite(22,HIGH);
    digitalWrite(18,HIGH);
   
    
if( hours == 15 && switch1==3) {stop(); switch1 = 0; x1=0; stroka ="Ver. "+ ota.version() +".  "+"  Сделайте выбор ";  }  
  }
//------------------------------------AVTO--------------------------------------------------------------------- 
if ((switch1 == 4) && (batlow == 0)) // авто
   {   stroka = " Автоматика " + web_time;  vklpomp = 1; regulator2.input = ts;   regulator2.setpoint = tust; 
 
  if (ta < 30 ) {shim1 = 25;}
    else {   regulator1.input = ts; regulator1.setpoint = tust; shim1 = regulator1.getResultTimer();}

  if ( (ta < 69) && (tz==0) ) {shim2 = 250; if (Rele==1) {digitalWrite(19,HIGH);}else webstart(); }
            else { if (Rele==1) {digitalWrite(19,LOW); } else {webstop();}  tz=1; regulator2.setpoint = tust+3; regulator2.input = ts; shim2 = regulator2.getResultTimer();}
       
   if (ta < 35 ) tz=0; 
   timers();
    
if( hours == 24 && switch1==4) {stop(); switch1 = 0; x1=0; stroka ="Ver. "+ ota.version() +".  "+"  Сделайте выбор ";  }
//............................................................................

      ledcWrite(21,shim2);  // шим помпа
      ledcWrite(23,shim1);   // шим вентиль отопителя 400Гц
    digitalWrite(22,HIGH);  // отопитель
    digitalWrite(18,HIGH); // климат
    }
//--------------------------------------------------------------------------------------------------------- 
if(shim1 > dshim1 ) ventil=1; else ventil=0;

 if (tmr5.tick()) {rekonektt();}
 ekr(); 
 oled.update(); 
 //sleep(); 
  client.loop(); 
  esp_task_wdt_reset();
}
//====================================================================================================
void sleep(){ 
  if(menu) return;
  if( (slepper == 1) && (switch1 == 0) ) {
if (tmr9.tick()) { 
  Serial.println(" ");Serial.println("Sleep"); //Serial.println(WiFi.getSleep() ? "Modem Sleep ВКЛЮЧЕН" : "Modem Sleep ВЫКЛЮЧЕН");  // вызывает завис  связи
//esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR); 
//esp_light_sleep_start(); // Режим легкого сна составляет около 0,8 мА.
//esp_deep_sleep_start(); // глубокого сна Чип потребляет от 0,15 мА
//WiFi.setSleep(true);
//WiFi.setSleep(false);  // ОТКЛЮЧАЕМ режим сна модема
  }}}
//==============================================================================================================================
void ekr(){ 
  if (vakb>13.7) {oled.setPower(1);slepper=0; tmr1.setTime(1000); tmr2.setTime(2000);}
  if (ekrON==0) { if (tmr6.tick()) 
     {oled.setContrast(1);} 
 if (tmr7.tick() and (vakb < 13.7)) {oled.setPower(0); slepper=1; tmr1.setTime(5000); tmr2.setTime(7000);}  //  включение ждущего режима
 }
  if (ekrON==1) {oled.setPower(1); oled.setContrast(200); ekrON=0; }
     
  if (n==0){
  oled.setCursorXY(5, 7);oled.print(vakb); oled.print(" Vbat"); oled.setCursorXY(73, 7);oled.print(RRSI); oled.print(" dB");
  oled.setCursorXY(5, 27);oled.print(ta); oled.print(" T.a"); oled.setCursorXY(73, 27);oled.print(ts); oled.print(" T.s");
  oled.setCursorXY(5, 48);oled.print(tust); oled.print(" T.ust"); oled.setCursorXY(73, 48); if( nagrev==" Нагрев! "){oled.invertText(1);} oled.print(nagrev); oled.invertText(0);
    oled.roundRect(0, 0, 62, 19,OLED_STROKE); oled.roundRect(64, 0, 127, 19,OLED_STROKE);
    oled.roundRect(0, 21, 62, 40,OLED_STROKE); oled.roundRect(64, 21, 127, 40,OLED_STROKE);
    oled.roundRect(0, 42, 62, 61,OLED_STROKE); oled.roundRect(64, 42, 127, 61,OLED_STROKE);
    //oled.update();
    }
    else { 

    
    
   
    }
}
//==============================================================================================================================
void rekonektt() { 
  
if (WiFi.status() != WL_CONNECTED) { WiFi.disconnect(); WiFi.reconnect(); }
}
//==============================================================================================================================
 void obnovl() { 
  if(!ob) return;
  String ver, notes;
if (ota.checkUpdate(&ver, &notes)) { 
  oled.clear(); oled.setCursor(10, 0);oled.print(" Update Version "); oled.setCursor(45, 1);  oled.invertText(1); oled.print(ver);oled.invertText(0); 
  oled.setCursor(0, 2);oled.println(" Notes:  "); oled.print(notes); oled.update(); delay(5000); oled.clear();
  oled.setCursor(10, 0);oled.print(" Update Begin !!!! "); oled.update();ota.updateNow();}
 ob=0;
  }
//==============================================================================================================================
void callback(char* topic, byte* payload, unsigned int length) { 
   
  String incommingMessage = "";

      for (int i = 0; i < length; i++) incommingMessage+=(char)payload[i];
       Serial.println("Message arrived ["+String(topic)+"]"+incommingMessage);
    
     if( strcmp(topic,switch1_topic) == 0) {
      if (incommingMessage.equals("0")) {switch1=0; ekrON=1;slepper=0; tone(tonePin, 783, 165.441);}
      if (incommingMessage.equals("1")) {switch1=1; ekrON=1;slepper=0; tone(tonePin, 783, 165.441);}
      if (incommingMessage.equals("2")) {switch1=2; ekrON=1;slepper=0; tone(tonePin, 783, 165.441);}
      if (incommingMessage.equals("3")) {switch1=3; ekrON=1;slepper=0; tone(tonePin, 783, 165.441);}
      if (incommingMessage.equals("4")) {switch1=4; ekrON=1;slepper=0; tone(tonePin, 783, 165.441);} 
          
     }
     if (strcmp(topic,shim1_topic) == 0) {shim1 =dshim1+incommingMessage.toInt();  }
     if (strcmp(topic,tust1_topic) == 0) { tust = incommingMessage.toInt(); EEPROM.put(300,tust); EEPROM.commit(); }  ////tust = incommingMessage.toInt();
  //Serial.print("switch1 =  "); Serial.println(switch1);
  //Serial.print("edit1 =  "); Serial.println(edit1);
  }
//==============================================================================================================================
void publishMessage(const char* topic, String payload , boolean retained){
   
  if (client.publish(topic, payload.c_str(), true))
      Serial.println("Message publised ["+String(topic)+"]: "+payload);
}
//==============================================================================================================================
void reconnect(){ 
   static unsigned long lastAttempt = 0;
    
    if(millis() - lastAttempt < 5000) return; // ждем 5 секунд
    
    
    
        Serial.print("Attempting MQTT connection...");
    String clientId = "EClient-";   // Create a random client ID
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {Serial.println("connected");
    client.subscribe(switch1_topic); 
    client.subscribe(shim1_topic); 
    client.subscribe(tust1_topic);
    //subscribe the topics here
      //client.subscribe(command2_topic);   
      } else {
      Serial.print("failed, rc=");Serial.print(client.state());Serial.println(" try again in 5 seconds"); 
      
      } 
 lastAttempt = millis();

}
//==============================================================================================================================
void akb() {  
   vakb = readAKB(); batlow = 0;
if (vakb < v) {batlow = 1;stroka = " Low Power STOP! ";switch1 = 0;}
    else if (vakb > 15.0) {batlow = 1;stroka = " HIGH Power STOP! ";switch1 = 0;}
}
//==============================================================================================================================
float readAKB() {
  uint32_t sum = 0;
  for (int i = 0; i < 20; i++) {sum += analogRead(AKB_PIN);delay(2);}
uint32_t raw = sum / 20;
uint32_t mv  = esp_adc_cal_raw_to_voltage(raw, &adc_chars);

  float vadc = mv / 1000.0;
  float vakb = vadc * ((DIV_R1 + DIV_R2) / (float)DIV_R2);
return vakb;
}
//======================================================
void timers() { 
  
  if(x1==0){ x1=1;  timer = millis();}
   sec = (millis() - timer) / 1000ul; seconds = (sec % 3600ul) % 60ul; minutes = (sec % 3600ul) / 60ul; hours = (sec / 3600ul);
  sprintf (buffer, "%02d:%02d:%02d", hours, minutes,seconds ); web_time = buffer;}
//============================================================================================================================== 
void stop(){ 
  //oooooooooooooooooooooooooooooooooooooooooooooooooooooo
 if  (Rele==1)  digitalWrite(19,LOW); else { if (sostweb != 0) {webstop();}  } // котел будет всегда просыпаться нужен флаг о не работающим котле надо разобрать биты 
 //ooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
 ledcWrite(21,0); // шим помпа
 ledcWrite(23,0); // шим вентиль отопителя 400Гц
 digitalWrite(22,LOW);  // отопитель
 digitalWrite(18,LOW); // климат
   
 }
//============================================================================================================================== 
//=======================================================================================================================
void webstop(){ ini(1);
  if (n2==1){
   kLineSerial.write(Init1, sizeof(Init1)); recive(sizeof(Init1));
   kLineSerial.write(Init2, sizeof(Init2)); recive(sizeof(Init2));
   kLineSerial.write(Request1, sizeof(Request1));recive(sizeof(Request1));
   kLineSerial.write(Request6, sizeof(Request6));recive(sizeof(Request6));
 n2=0;}
}

//=======================================================================================================================
void webstart(){ ini(1);
if (n2==0){
   kLineSerial.write(Init1, sizeof(Init1)); recive(sizeof(Init1));
   kLineSerial.write(Init2, sizeof(Init2)); recive(sizeof(Init2));
   kLineSerial.write(Request1, sizeof(Request1));recive(sizeof(Request1));
   kLineSerial.write(Request4, sizeof(Request4));recive(sizeof(Request4));
n2=1;}
}
//=======================================================================================================================
void data(int n3){ 
if ( n3==1){  ini(1);
  kLineSerial.write(Init1, sizeof(Init1));       recive(sizeof(Init1));
  kLineSerial.write(Init2, sizeof(Init2));       recive(sizeof(Init2)); 
  //kLineSerial.write(Wakeup, sizeof(Wakeup));     recive(sizeof(Wakeup));     
  kLineSerial.write(Request1, sizeof(Request1)); recive(sizeof(Request1));
  n3 =0;}
   n1 = kLineSerial.available(); Serial.print("n1 = "); Serial.println(n1); //Функция получает количество байт(символов)
 while(kLineSerial.available()) { for (int i=0;i<n1;i++) Answer[i]=kLineSerial.read();}
   Serial.println("DATA -------------------------------------------------------"); 
   for (int i=sizeof(Request1);i<n1;i++) {Serial.print(Answer[i],HEX);  Serial.print(" "); } Serial.println("");
   Serial.println("-------------------------------------------------------------"); 

  Serial.print( "R2-- ");  R2 = (Answer[sizeof(Request1)+6]);  Serial.print(R2/124);  
if (Answer[sizeof(Request1)] == 0x88 ){  
ta = Answer[sizeof(Request1)+5]/2.481; R2 = Answer[sizeof(Request1)+6];  R1=R2/124;
vakb = Answer[sizeof(Request1)+7]/14.65; v = Answer[sizeof(Request1)+8]/14.65; 
mode = Answer[sizeof(Request1)+9];
 
 }}
//=======================================================================================
void recive(int r){


   n = kLineSerial.available(); oled.setCursor(0, 6); oled.print("Rx n = "); oled.print(n);   oled.print("   r = "); oled.print(r);
 while(kLineSerial.available()) { for (int i=0;i<n;i++) Answer[i]=kLineSerial.read(); }
  //oled.setCursor(0, 7);oled.print("                                            ");oled.update();

      for (int i=r;i<n;i++) {Serial.print(Answer[i],HEX);  Serial.print(" "); } Serial.println("");

  //oled.setCursor(0, 7);  for (int i=r; i<n; i++) oled.print(Answer[i],HEX);
  //oled.update();
}
//=======================================================================================================================
void data1(int n4) {
  if (n4==1){
   kLineSerial.write(Init1, sizeof(Init1)); recive(sizeof(Init1));
   kLineSerial.write(Init2, sizeof(Init2)); recive(sizeof(Init2));
   //kLineSerial.write(Wakeup, sizeof(Wakeup));recive(sizeof(Wakeup));
   kLineSerial.write(Request7, sizeof(Request7));recive(sizeof(Request7));
n4=0;}
   n = kLineSerial.available(); Serial.print("n = "); Serial.println(n1);     //Функция получает количество байт(символов)
 while(kLineSerial.available()) { for (int i=0;i<n;i++) Answer[i]=kLineSerial.read();}
  
  Serial.println("DATA1 -------------------------------------------------------"); 
  for (int i=sizeof(Request7);i<n;i++) {Serial.print(Answer[i],HEX);  Serial.print(" "); } Serial.println("");
  Serial.println("-------------------------------------------------------------"); 
  
if (Answer[sizeof(Request7)] == 0x88 ){
    nagn = Answer[sizeof(Request7)+6];
    shift = Answer[sizeof(Request7)+7];
    pomp= Answer[sizeof(Request7)+8];
    tpomp = Answer[sizeof(Request7)+9];
    ign = Answer[sizeof(Request7)+10];
    sostweb = nagn+shift+pomp+tpomp+ign;
 }
Serial.print("nagn--");Serial.print(nagn);Serial.print("  shift---");Serial.print(shift);Serial.print("  pomp--"); Serial.print( pomp);
Serial.print("  tpomp---");Serial.print(tpomp);Serial.print("  ign---");Serial.print(ign);
}
//=======================================================================================================================
void ini(int n5){
if(n5==1){
 pinMode(txPin, OUTPUT);digitalWrite(txPin, LOW);delay(300);digitalWrite(txPin, HIGH);delay(25);digitalWrite(txPin, LOW);delay(25);digitalWrite(txPin, HIGH);//delay(3025);
delay(500);
kLineSerial.begin(baudRate, SERIAL_8N1, rxPin, txPin);
n5=0;} 
}
//=======================================================================================================================
//============================= Прерывание по таймеру функция ===========================================
void WIFISEL () {
 //portalRun(); 
label0:
 if(portal==0){
   EEPROM.get(0, portalCfg.SSID); EEPROM.get(150, portalCfg.pass); WiFi.mode(WIFI_STA); WiFi.begin(portalCfg.SSID, portalCfg.pass);
      oled.setCursor(0, 0);oled.println(portalCfg.SSID); oled.print(portalCfg.pass); oled.update(); delay(3000); oled.clear();
      oled.setCursor(0, 3);oled.println("Подключение.");oled.update(); 

  timerwifi33 = millis(); 
  while (WiFi.status() != WL_CONNECTED) {Serial.print("."); oled.print(".");oled.update(); delay(500);
    if((millis()-timerwifi33) > 25000) { portal=1; WiFi.disconnect(); 
    oled.clear(); oled.setCursor(0, 0);oled.invertText(1);oled.print("  ESP-conf Start !  "); oled.update(); oled.invertText(0);goto label0;} }
    
  }
if (portal==1) {
 
  portalRun(180000); 
   
 

 switch (portalStatus()) { 
   
    

        case SP_SUBMIT: portal=0; EEPROM.put(0,portalCfg.SSID);EEPROM.put(150,portalCfg.pass); EEPROM.commit();
        oled.clear();oled.setCursor(0, 0);oled.println(portalCfg.SSID);oled.print(portalCfg.pass);oled.update(); delay(3000);
        char SSI[32]; 
  EEPROM.get(0, SSI);oled.setCursor(0, 3);oled.println(SSI);oled.update();
  EEPROM.get(150, SSI);oled.print(SSI);oled.update(); delay(3000);                              goto label0;  break;
        case SP_SWITCH_AP: portal=2;WiFi.mode(WIFI_AP); WiFi.softAP("LabadaSto", "12345678");                 break;  
        case SP_SWITCH_LOCAL: portal=0;                                                                       break;
        case SP_EXIT:  portal=0; goto label0;                                                                 break;
        case SP_TIMEOUT: portal=2; portal=1; WiFi.mode(WIFI_AP); WiFi.softAP("LabadaSto", "12345678");        break;
        case SP_ERROR:   portal=1; goto label0;                                                               break;
 }
}
char SSI[32]; 
  EEPROM.get(0, SSI);Serial.println(SSI);
  EEPROM.get(150, SSI);Serial.print(SSI);  
    

}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//==============================================================
