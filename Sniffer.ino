// Версия 1.0
#include <SPI.h>
#include "mcp_can.h"

// Шилд подключен следущим образом:
// CS   - DI10
// MISO - DI12
// MOSI - DI11
// SCK  - DI13
// INT  - DI2

#define SPI_CS_PIN 10 // сигнал CS на DI10

bool canDataReceived = false; // флаг для обработки прерывания поступления данных из CAN-шины

#define DATA_LIMIT 8

// струтура хранения данных CAN-пакета
struct CANFrame
{
  uint32_t ID;                // идентификатор пакета
  uint8_t  Length;            // длина данных
  uint8_t  Data[DATA_LIMIT];  // сами данные
};

// структура хранения данных отправляемых в COM-порт
struct ComOutBuffer
{
  uint8_t Prefix0 = 0xAA;
  uint8_t Prefix1 = 0x55;
  uint8_t Prefix2 = 0xAA;
  uint8_t Prefix3 = 0x55;
  CANFrame OutFrame;
};

ComOutBuffer comOutBuffer;      // буфер отправки данных в COM-порт и ниже указатель на него
uint8_t*     comOutBufferPtr = (void*)&comOutBuffer;

// переменные для обработки входщих данных последовательного порта для формирования тестового пакета
CANFrame testFrame = { 0 };
uint8_t* idBytes = (void*)&testFrame.ID;
uint8_t  comFrameStep = 0;    // переменная стадии обработки входищих данных из COM-порта

// переменные для замера различных скоростных параметров
uint16_t counterFPS  = 0;      // CAN-пакеты в секунду
uint16_t counterBPS  = 0;      // байты в секунду
uint32_t counterTime = millis();

//###################################################################################################################################
// инициализировать CAN-адаптер
MCP_CAN CAN(SPI_CS_PIN);

//###################################################################################################################################
// Инициализация
//###################################################################################################################################
void setup()
{
  // очистка пакета CAN
  comOutBuffer.OutFrame.ID = 0;
  comOutBuffer.OutFrame.Length = 0;
  for (uint8_t iData = 0; iData < DATA_LIMIT; iData++)
  {
    comOutBuffer.OutFrame.Data[iData] = 0;
  }

  Serial.begin(500000);

  // настройка скорости обмена и частоты кварца
  while (CAN.begin(CAN_500KBPS, MCP_8MHz) != CAN_OK)
  {
    delay(100);
  }

  // очистка фильтров
  /*byte std = 0;
    byte ext = 1;
    int ulMask = 0x00, ulFilt = 0x00;

    // очистка масок
    CAN.init_Mask(0, ext, ulMask);
    CAN.init_Mask(1, ext, ulMask);

    // сброс фильтров
    CAN.init_Filt(0, ext, ulFilt);
    CAN.init_Filt(1, std, ulFilt);
    CAN.init_Filt(2, ext, ulFilt);
    CAN.init_Filt(3, std, ulFilt);
    CAN.init_Filt(4, ext, ulFilt);
    CAN.init_Filt(5, std, ulFilt);*/

  // подключить обработчик прерывания о поступлении данных по спадающему уровню сигнала
  attachInterrupt(0, CANInterrupt, FALLING);
}

//###################################################################################################################################
// Обработчик прерывания поступивших данных
//###################################################################################################################################
void CANInterrupt()
{
  // установить флаг наличия данных в буфере
  canDataReceived = true;
}

//###################################################################################################################################
// Основной цикл программы
//###################################################################################################################################
void loop()
{
  // если в CAN-буфере есть данные...
  if (canDataReceived)
  {
    // сбросить флаг данных в буфере
    canDataReceived = false;

    while (CAN.checkReceive() == CAN_MSGAVAIL)
    {
      // чтение данных в буфер
      if (CAN.readMsgBuf(&comOutBuffer.OutFrame.Length, comOutBuffer.OutFrame.Data) == CAN_OK)
      {
        // получение идентификатора пакета (достоверный только после чтения сообщения в буфер)
        comOutBuffer.OutFrame.ID = CAN.getCanId();
        Serial.write(comOutBufferPtr, 9 + comOutBuffer.OutFrame.Length);

        // замеры скоростных парамтеров
        if (millis() - counterTime >= 1000)
        {
          comOutBuffer.OutFrame.ID = 0;
          comOutBuffer.OutFrame.Length = 4;
          // а при копировании little-endian, вместо big-endian, поэтому надо побайтово делать
          // количество пакетов в секунду, примерная скорость около 745 пакетов в секунду
          comOutBuffer.OutFrame.Data[0] = highByte(counterFPS);  // counterFPS >> 8 & 0xFF;
          comOutBuffer.OutFrame.Data[1] = lowByte(counterFPS);   // counterFPS & 0xFF;
          // количество байтов в секунду, примерная скорость около 9200 байтов в секунду
          comOutBuffer.OutFrame.Data[2] = highByte(counterBPS);  // counterBPS >> 8 & 0xFF;
          comOutBuffer.OutFrame.Data[3] = lowByte(counterBPS);   // counterBPS & 0xFF;
          Serial.write(comOutBufferPtr, 9 + comOutBuffer.OutFrame.Length);
          counterFPS = 0;
          counterBPS = 0;
          counterTime = millis();
        }

        counterFPS++;
        counterBPS += (5 + comOutBuffer.OutFrame.Length);
      }
    }
  }
}

//###################################################################################################################################
// Прерывание обработки данных из последовательного порта
//###################################################################################################################################
void serialEvent()
{
  int b;

  // чтение из COM-порта, пока там есть данные
  while (Serial.available())
  {
    /*
      идея такая: последовательно читая каждый байт, найти сигнатуру
      потом заполнить ID-пакета и далее длину пакета
      далее заполнить входной массив количеством байтов до длины пакета
    */
    switch (comFrameStep)
    {
      // поиск заголовка
      case 0: case 2:
        if (Serial.read() == 0xAA)
          comFrameStep++;
        else
          comFrameStep = 0;
        break;

      // поиск заголовка
      case 1: case 3:
        if (Serial.read() == 0x55)
          comFrameStep++;
        else
          comFrameStep = 0;
        break;

      // получение ID-пакета
      case 4 ... 7:
        b = Serial.read();
        *(idBytes + comFrameStep - 4) = (byte)b;
        comFrameStep++;
        break;

      // получение длины данных
      case 8:
        b = Serial.read();
        if (b > 0 && b <= 8)
        {
          testFrame.Length = (byte)b;
          comFrameStep++;
        }
        else
          comFrameStep = 0;
        break;

      // получение непосредственно самих данных пакета
      case 9 ... 16:
        if (comFrameStep - 9 < testFrame.Length)
        {
          b = Serial.read();
          testFrame.Data[comFrameStep - 9] = (byte)b;
          comFrameStep++;
        }
        else
          comFrameStep = 17;
        break;
    }

    // пакет собран и будет отправлен через SendCANFrame()
    if (comFrameStep == 17)
    {
      if (CAN.sendMsgBuf(testFrame.ID, 0, testFrame.Length, testFrame.Data, true) == CAN_OK)
      {
        // отправилось без ошибок
        comFrameStep = 0;
      }
    }
  }
}
