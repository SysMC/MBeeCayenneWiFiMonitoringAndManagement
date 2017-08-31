
/**
  * Скетч предназначен для вывода данных о температуре, измеряемой бортовым датчиком на модуле MBee,
  * и влажности, получаемой модулем MBee с датчика HIH4000, в Cayenne MyDevises, а также, для вкл/выкл электропотребителей.
  * Максимальное число датчиков - 10 шт.
  * Принимающий и передающий модули MBee-868-x.0 работают под управлением ПО Serial Star.
  * 
**/
    
#include <MBee.h>
#include <SoftwareSerial.h>
#include <CayenneMQTTESP8266.h>
#include <SimpleTimer.h>  

#define LED 2  //Вывод контрольного светодиода.
#define ACTIVITY_CHECK_PERIOD  3600000 //Период проверик актуальности данных
#define SENSOR_OFF 0
#define SENSOR_ON 1
#define SENSORS_MAX_COUNT 10 //Максимальное число автономных устройств, передающих показания датчиков температуры и влажности

#define CMD_PARAMETER_LEN 1


// WiFi network info.
char ssid[] = "ssid";
char wifiPassword[] = "wifiPassword";

// Cayenne authentication info. This should be obtained from the Cayenne Dashboard.
char username[] = "username";
char password[] = "password";
char clientID[] = "clientID";
const uint16_t sensor[SENSORS_MAX_COUNT] = {0x0002, 0x0003, 0x0004, 0x0005, 0x0006, 0x0007, 0x0008, 0x0009, 0x000A, 0x000B}; //Адреса автономных устройств, передающих показания датчиков температуры и влажности
uint8_t flagActivityCheck [SENSORS_MAX_COUNT];

typedef struct
{
  uint8_t sof = 0x7E;
  uint8_t lengthMsb = 0x00;
  uint8_t lengthLsb = 0x08;
  uint8_t frameType = 0x17;
  uint8_t frameID = 0x00;
  uint8_t MSBDestination = 0x00;
  uint8_t LSBDestination = 0x0C;
  uint8_t remoteCommandOptions = 0x0D;
  unsigned char code[2] = {'L','5'};
  uint8_t commandParameter; 
  uint8_t checkSum;
}TxFrame_t;

TxFrame_t frame;
SerialStar mbee;
RxIoSampleResponse ioSample;
SimpleTimer timer;

enum 
{
 NO_EVENTS,
 ACTUATOR_ON,
 ACTUATOR_OFF
}events = NO_EVENTS;

void setup() 
{ 
  Serial.begin(9600);
  mbee.begin(Serial);
  Cayenne.begin(username, password, clientID, ssid, wifiPassword);
  pinMode(LED, OUTPUT);  //Настраиваем вывод контрольного светодиода.
  digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  timer.setInterval(ACTIVITY_CHECK_PERIOD, sensorActivityCheck);
}

void loop() 
{
  Cayenne.loop();
  timer.run();

  switch(events)
  {
    case ACTUATOR_ON: 
      frame.commandParameter = 0x05; //Команда установки на L5 высокого уровня.
      frame.checkSum = 0x49;
      sendCommand();
      events = NO_EVENTS;
      break;
    
    case ACTUATOR_OFF: 
      frame.commandParameter = 0x04; //Переводим L5 в низкий уровень.
      frame.checkSum = 0x4A;
      sendCommand();
      events = NO_EVENTS;
      break;

    default:
      break;
  }
  mbee.readPacket(); //Проверяем, если в буфере пакет.
  if(mbee.getResponse().isAvailable()) 
  {
    if(mbee.getResponse().getApiId() == IO_DATA_SAMPLE_API_FRAME) //Является ли принятый пакет, пакетом с данными о состоянии датчиков удаленного модема?
    {
      /**********************************************************************************************************/
      mbee.getResponse().getRxIoSampleResponse(ioSample); //Получаем пакет с данными.

      for(uint8_t i = 0; i < SENSORS_MAX_COUNT; i++)
      {
        if(sensor[i] == ioSample.getRemoteAddress())
        {
          if(ioSample.getTemperature() < 128) //Переводим число из дополнительного кода в прямой.
            {
              Cayenne.celsiusWrite(i, ioSample.getTemperature());
            }
          else
            {
              Cayenne.celsiusWrite(i, 256 - ioSample.getTemperature());
            }  
          Cayenne.virtualWrite(i + 10, getHIH4000Humidity(ioSample.getAnalog(1)));
          Cayenne.virtualWrite(i + 20, float(ioSample.getVbatt()) / 51);
          flagActivityCheck[i] = SENSOR_ON;
        }
      }
    } 
  }
}

/*Функция пересчета влажности для датчика HIH4000**********************************************/
float getHIH4000Humidity(uint16_t adcData)
{
  float humidity;
  humidity = ((adcData *1.9881158) / 50) - 31.20521173;       //формула расчета влажности для данного датчика: (Vout*2-0.958)/0.0307. Умножаем на 2 из-за делителя на плате. Все остальное из datasheet на датчик
  return humidity;
} 

/*Функция проверки актуальности данных от датчиков*********************************************/
void sensorActivityCheck(void)
{
  for(uint8_t i = 0; i < SENSORS_MAX_COUNT; i++)
  {
    if(flagActivityCheck[i] == SENSOR_OFF)
    {
      Cayenne.celsiusWrite(i, 0);
      Cayenne.virtualWrite(i + 10, 0);
      Cayenne.virtualWrite(i + 20, 0);
    }
    else
    {
       flagActivityCheck[i] = SENSOR_OFF;
    }
  }
}

//Функция считывания из Cayenne*********************************************
CAYENNE_IN(31)
{
    if(getValue.asInt() == 1)
    {
      events = ACTUATOR_ON;
     }
    else if(getValue.asInt() == 0)
    {
      events = ACTUATOR_OFF;
    }
}

//Функция отправки данных модулю MBee*********************************************
void sendCommand()
{
  Serial.write((uint8_t*)&frame, sizeof(TxFrame_t)); 
}
