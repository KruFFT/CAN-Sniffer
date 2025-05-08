// Версия 2.3

// 2024.05.08 версия 2.3 - перевод статистики на little-endian, команды управления подключением, переход на VSCode + PlatformIO, рефакторинг кода
// 2024.08.19 версия 2.2 - отказ от использования ArduinoSTL, переход на обычный массив
// 2024.06.15 версия 2.1 - используется библиотека CAN-BUS Shield by Seeed Studio версии 2.3.3
//                       - используется библиотека ArduinoSTL by Mike Matera 1.3.3 (необходимо исправление: https://github.com/mike-matera/ArduinoSTL/issues/95)
//                       - добавлен интервал времени между пакетами в мс
// 2022.07.02 версия 2.0 - добавлена поддержка ESP и Wi-Fi
// 2022.03.12 версия 1.2 - обновление библиотеки CAN-BUS Shield by Seeed Studio до версии 2.3.1
// 2020.06.13 версия 1.1 - добавил очистку фильтров в setup()

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515_can.h>

// Параметры подключения к AVR
#ifdef MODE_AVR
    // Для AVR шилд подключен следующим образом:
    // INT  - D2
    // CS   - D10
    // SCK  - D13
    // MISO - D12
    // MOSI - D11
    #define CAN_INT 2   // сигнал INT на D2
    #define CAN_CS 10   // сигнал CS на D10
#endif // MODE_AVR

// Параметры подключения к ESP
#ifdef MODE_ESP
    // Для ESP шилд подключен следующим образом:
    // INT  - GPIO5  (D1)
    // CS   - GPIO15 (D8)
    // SCK  - GPIO14 (D5)
    // MISO - GPIO12 (D6)
    // MOSI - GPIO13 (D7)
    #define CAN_INT 5   // сигнал INT на GPIO5 (D1)
    #define CAN_CS  15  // сигнал CS на GPIO15 (D8)
    // Используется микросхема сдвига логических уровней 5В <-> 3,3В, сигнал управления EN на GPIO4 (D2)
    #define LEVEL_SHIFTER 4
#endif // MODE_ESP

#ifdef MODE_SERIAL
    #define SERIAL_SPEED        500000      // скорость последовательного порта
    #define PARSING_TIMEOUT     1000        // длительность разбора пакета до тайм-аута
    uint32_t parsingTime = 0;               // время начала разбора пакета от компьютера
#endif // MODE_SERIAL

// Настройки беспроводного режима работы
#ifdef MODE_WIRELESS
    #include <ESP8266WiFi.h>
    #include <WiFiUdp.h>

    // настройки точки доступа Wi-Fi
    const char* ssid       = "CAR";                 // название беспроводной сети
    const char* password   = "1357924680";          // пароль к сети
    const IPAddress pcIp(192, 168, 1, 100);         // IP-адрес компьютера
    const uint16_t udpPort = 0xAA55;                // номер порта для обмена данными
    WiFiUDP UDP;                                    // UDP-сокет

    #define  UDP_BUFFER_SIZE 200                    // размер буфера пакетов
    uint8_t  udpOutBuffer[UDP_BUFFER_SIZE] = { 0 }; // буфер накопления пакетов на отправку на компьютер
    uint8_t* udpOutBufferPosition = udpOutBuffer;   // указатель на этот буфер
    uint8_t  udpInBuffer[UDP_BUFFER_SIZE] = { 0 };  // буфер приёма пакета от компьютера
#endif // MODE_WIRELESS

// Сигнатура и различные структуры, буферы и счётчики
#define SIGNATURE                   0xAA55AA55u // сигнатура передаваемых пакетов
#define SERVICE_ID                  0x7FFu      // идентификатор пакета статистики и управления
#define FRAME_INTERVALS_SIZE        100         // размер массива подсчёта интервалов между CAN-пакетами
#define PARSING_COMPLETE            19          // состояние: пакет получен и удачно декодирован
#define SWAP_BYTES_UINT32(value)    ((((value) >> 24) & 0x000000FF) | (((value) >> 8) & 0x0000FF00) | (((value) << 8) & 0x00FF0000) | (((value) << 24) & 0xFF000000))

// Структура данных CAN-пакета
struct CANFrame
{
    uint32_t id;        // идентификатор пакета
    uint16_t interval;  // интервал между пакетами
    uint8_t  length;    // длина данных
    uint8_t  data[8];   // данные пакета
};

// Структура хранения данных отправляемых в последовательный порт или через Wi-Fi
struct OutCANFrame
{
    uint32_t signature = SWAP_BYTES_UINT32(SIGNATURE);
    CANFrame frame;
};

uint8_t  parsingStep = 0;           // номер стадии разбора пришедших из последовательного порта данных
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
FrameTime frameTimes[FRAME_INTERVALS_SIZE] = {{ 0, 0 }};
uint16_t frameTimesAmount = 0;

// Флаг для обработки прерывания поступления данных из CAN-шины и инициализация объекта CAN
volatile bool canDataReceived = false;
mcp2515_can CAN(CAN_CS);

// Объявление всех функций
void setup();
void loop();
void calculateSpeed(uint32_t currentTime);
uint16_t getAndSaveIdInterval(uint32_t id, uint32_t currentTime);
void sendPacketToPC();
void receivePacketFromPC();
void processCommand();
void CANConnect(uint16_t speed);
void CANDisconnect();
void CANInterrupt();

// Параметры симулятора
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
#endif // MODE_SIMULATOR
