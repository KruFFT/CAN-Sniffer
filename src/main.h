// Версия 2.3

// 2024.05.04 версия 2.3 - перевод статистики на little-endian, команды управления подключением
// 2024.08.19 версия 2.2 - отказ от использования ArduinoSTL, переход на обычный массив
// 2024.06.15 версия 2.1 - используется библиотека CAN-BUS Shield by Seeed Studio версии 2.3.3
//                       - используется библиотека ArduinoSTL by Mike Matera 1.3.3 (необходимо исправление: https://github.com/mike-matera/ArduinoSTL/issues/95)
//                       - добавлен интервал времени между пакетами в мс
// 2022.07.02 версия 2.0 - добавлена поддержка ESP и Wi-Fi
// 2022.03.12 версия 1.2 - обновление библиотеки CAN-BUS Shield by Seeed Studio до версии 2.3.1
// 2020.06.13 версия 1.1 - добавил очистку фильтров в setup()

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515_can.h>        // CAN-BUS Shield by Seeed Studio

// Параметры подключения к AVR
// Для AVR (Arduino) шилд подключен следующим образом:
// INT  - D2
// CS   - D10
// SCK  - D13
// MISO - D12
// MOSI - D11
#define CAN_INT 2   // сигнал INT на D2
#define CAN_CS 10   // сигнал CS на D10

// Сигнатура и различные структуры, буферы и счётчики
#define SERIAL_SPEED            500000      // скорость последовательного порта
#define SIGNATURE               0x55AA55AA  // сигнатура передаваемых пакетов (big-endian)
#define SERVICE_ID              0x7FF       // идентификатор пакета статистики и управления
#define FRAME_INTERVALS_LIMIT   100         // размер массива подсчёта интервалов между CAN-пакетами
#define PARSING_TIMEOUT         1000        // длительность разбора пакета до тайм-аута

// Структура данных CAN-пакета
struct CANFrame
{
    uint32_t id;        // идентификатор пакета
    uint16_t interval;  // интервал между пакетами
    uint8_t  length;    // длина данных
    uint8_t  data[8];   // сами данные
};

// Структура хранения данных отправляемых в последовательный порт или через Wi-Fi
struct OutCANFrame
{
    uint32_t signature = SIGNATURE;
    CANFrame frame;
};

uint8_t  parsingStep = 0;           // номер стадии разбора пришедших из последовательного порта данных
uint32_t parsingTime = millis();    // время начала разбора данных (для сброса по тайм-ауту)
CANFrame inCANFrame = { 0 };        // принятый из последовательного порта пакет для отправки в CAN-шину
OutCANFrame outCANFrame;            // буфер отправки данных в последовательный порт

// Переменные для замера различных скоростных параметров
uint16_t counterFPS  = 0;           // CAN-пакеты в секунду
uint16_t counterBPS  = 0;           // байты в секунду
uint32_t counterTime = millis();    // счётчик времени для подсчёта количества CAN-пакетов и байтов в секунду

// Описание номера пакета и его времени для массива интервалов между пакетами
struct FrameTime
{
    uint32_t ID;
    uint32_t ms;
};

// Массив интервалов между CAN-пакетами
FrameTime frameTimes[FRAME_INTERVALS_LIMIT + 1] = {{ 0, 0 }};
uint16_t frameTimesAmount = 0;

// Флаг для обработки прерывания поступления данных из CAN-шины и инициализация CAN
volatile bool canDataReceived = false;
mcp2515_can CAN(CAN_CS);

// Объявление всех функций
void setup();
void loop();
void calculateSpeed(uint32_t currentTime);
void sendPacketToPC();
void receivePacketFromPC();
void processCommand();
uint16_t getAndSaveIdInterval(uint32_t id, uint32_t currentTime);
void CANConnect(uint16_t speed);
void CANDisconnect();
void CANInterrupt();

// Пара
#ifdef MODE_SIMULATOR
    #define  TEST_INTERVAL      5
    #define  TEST_LONG_INTERVAL 10000
    uint32_t testTime = millis();
    uint32_t testLongTime = millis();
    uint8_t  testLongInc = 0x00;
    uint8_t  testInc = 0x00;
    uint8_t  testLongDec = 0xFF;
    uint8_t  testDec = 0xFF;
    bool     extended = false;
#endif
