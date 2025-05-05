// Версия 2.3

// 2024.05.04 версия 2.3 - перевод статистики на little-endian
// 2024.08.19 версия 2.2 - отказ от использования ArduinoSTL, переход на обычный массив
// 2024.06.15 версия 2.1 - используется библиотека CAN-BUS Shield by Seeed Studio версии 2.3.3
//                       - используется библиотека ArduinoSTL by Mike Matera 1.3.3 (необходимо исправление: https://github.com/mike-matera/ArduinoSTL/issues/95)
//                       - добавлен интервал времени между пакетами в мс
// 2022.07.02 версия 2.0 - добавлена поддержка ESP и Wi-Fi
// 2022.03.12 версия 1.2 - обновление библиотеки CAN-BUS Shield by Seeed Studio до версии 2.3.1
// 2020.06.13 версия 1.1 - добавил очистку фильтров в setup()

// Перед сборкой необходимо выбрать конфигурацию убрав соответствующий комментарий:
// AVR - для Arduino
// ESP - для ESP
// Так же выбрать режим: для Arduino будет работать SERIAL, для ESP оба варианта SERIAL/WIRELESS

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

// Включение отправки пакета для тестирования без подключения к CAN-шине
//#define TEST

#ifdef TEST
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

// Обработчик прерывания поступивших данных
void CANInterrupt()
{
    canDataReceived = true;     // установить флаг наличия данных в буфере
}

// Инициализация
void setup()
{
    Serial.begin(SERIAL_SPEED);
    pinMode(CAN_INT, INPUT);
}

// Основной цикл программы
void loop()
{
    #ifdef TEST
        uint32_t currentTime = millis();
        
        if (currentTime - testLongTime >= TEST_LONG_INTERVAL)
        {
            testLongTime = currentTime;
            testLongInc++;
            testLongDec--;
        }

        if (currentTime - testTime >= TEST_INTERVAL)
        {
            testTime = currentTime;
            testInc++;
            testDec--;
            // заполнение пакета случайными данными
            outCANFrame.frame.id       = 0xABCu;
            if (extended)
            {
                outCANFrame.frame.id  |= 0x80000000u;
            }
            outCANFrame.frame.interval = TEST_INTERVAL;
            outCANFrame.frame.length   = 8;
            outCANFrame.frame.data[0]  = testLongInc;
            outCANFrame.frame.data[1]  = testInc;
            outCANFrame.frame.data[2]  = testLongDec;
            outCANFrame.frame.data[3]  = testDec;
            outCANFrame.frame.data[4]  = testLongInc;
            outCANFrame.frame.data[5]  = testInc;
            outCANFrame.frame.data[6]  = testLongDec;
            outCANFrame.frame.data[7]  = testDec;
            sendPacketToPC();

            // замеры скоростных параметров
            calculateSpeed(currentTime);

            extended = !extended;
        }
    #else
        // если в CAN-буфере есть данные...
        if (canDataReceived)
        {
            // сбросить флаг данных в буфере
            canDataReceived = false;

            while (CAN.checkReceive() == CAN_MSGAVAIL)
            {
                // чтение данных из буфера
                if (CAN.readMsgBuf(&outCANFrame.frame.length, outCANFrame.frame.data) == CAN_OK)
                {
                    // получение идентификатора пакета (достоверный только после чтения сообщения из буфера)
                    outCANFrame.frame.id = CAN.getCanId();
                    // обработка интервала между CAN-пакетами
                    uint32_t currentTime = millis();
                    outCANFrame.frame.interval = getAndSaveIdInterval(outCANFrame.frame.id, currentTime);
                    // для расширенного ID установить старший бит
                    if (CAN.isExtendedFrame())
                    {
                        outCANFrame.frame.id |= 0x80000000u;
                    }

                    sendPacketToPC();

                    // замеры скоростных параметров
                    calculateSpeed(currentTime);
                }
            }
        }
    #endif

    receivePacketFromPC();
}

// Получение интервала для ID
uint16_t getAndSaveIdInterval(uint32_t id, uint32_t currentTime)
{
    // поиск по массиву
    for (size_t index = 0; index < frameTimesAmount; index++)
    {
        if (frameTimes[index].ID == id)
        {
            uint32_t interval = currentTime - frameTimes[index].ms;
            frameTimes[index].ms = currentTime;
            return (interval < 65536) ? (uint16_t)interval : 0;
        }
    }
    // пакет не найден - надо добавить
    if (frameTimesAmount < FRAME_INTERVALS_LIMIT)
    {
        frameTimes[frameTimesAmount].ID = id;
        frameTimes[frameTimesAmount].ms = currentTime;
        frameTimesAmount++;
    }
    return 0;
}

// Подсчёт скоростных показателей CAN-пакетов
void calculateSpeed(uint32_t currentTime)
{
    if (currentTime - counterTime >= 1000)
    {
        counterTime = currentTime;
        outCANFrame.frame.id = SERVICE_ID;
        outCANFrame.frame.length = 4;
        // количество пакетов в секунду, примерная скорость около 745 пакетов в секунду
        *(uint16_t*)(&(outCANFrame.frame.data[0])) = counterFPS;
        // количество байтов в секунду, примерная скорость около 9200 байтов в секунду
        *(uint16_t*)(&(outCANFrame.frame.data[2])) = counterBPS;

        sendPacketToPC();
        
        counterFPS = 0;
        counterBPS = 0;
    }

    counterFPS++;
    counterBPS += (5 + outCANFrame.frame.length);
}

// Отправка полученного CAN-пакета в последовательный порт
void sendPacketToPC()
{
    // сигнатура (4 байта) + ID пакета (4 байта) + интервал (2 байта) + длина (1 байт) = 11 байт
    size_t dataLength = 11 + outCANFrame.frame.length;    
    Serial.write((uint8_t*)&outCANFrame, dataLength);
}

// Разбор и приём данных из последовательного порта
void receivePacketFromPC()
{
    //uint32_t currentTime = millis();

    if (Serial.available() && parsingStep >= 0 && parsingStep <= 18)
    {
        int nextByte = Serial.read();
        // идея такая: последовательно читая каждый байт, найти сигнатуру
        // потом заполнить ID-пакета и далее длину пакета
        // далее заполнить входной массив количеством байтов до длины пакета
        switch (parsingStep)
        {
            // поиск сигнатуры
            case 0:
                if (nextByte == (SIGNATURE & 0xFFu)) parsingStep++; else parsingStep = 0;
                break;
    
            // поиск сигнатуры
            case 1:
                if (nextByte == ((SIGNATURE >> 8) & 0xFFu)) parsingStep++; else parsingStep = 0;
                break;
    
            // поиск сигнатуры
            case 2:
                if (nextByte == ((SIGNATURE >> 16) & 0xFFu)) parsingStep++; else parsingStep = 0;
                break;

            // поиск сигнатуры
            case 3:
                if (nextByte == ((SIGNATURE >> 24) & 0xFFu)) parsingStep++; else parsingStep = 0;
                break;

            // получение ID-пакета
            case 4 ... 7:
                *((uint8_t*)&inCANFrame.id + parsingStep - 4) = (uint8_t)nextByte;
                parsingStep++;
                break;

            // пропуск зарезервированного слова
            case 8 ... 9:
                parsingStep++;
                break;

            // получение длины данных
            case 10:
                if (nextByte >= 0 && nextByte <= 8)
                {
                    inCANFrame.length = (uint8_t)nextByte;
                    parsingStep++;
                }
                else
                    parsingStep = 0;
                break;
    
            // получение непосредственно самих данных пакета
            case 11 ... 18:
                if (parsingStep - 11 < inCANFrame.length)
                {
                    inCANFrame.data[parsingStep - 11] = (uint8_t)nextByte;
                    parsingStep++;
                }
                else
                    parsingStep = 19;
                break;
        }
    }
    else if (parsingStep == 19)
    {
        // получен CAN-пакет от компьютера
        parsingStep = 0;
        // проверка управляющих пакетов
        if (inCANFrame.id == SERVICE_ID)
        {
            processCommand();
        }
        else
        {
            #ifdef TEST
                outCANFrame.frame = inCANFrame;
                outCANFrame.frame.id += 8;
                outCANFrame.frame.interval = 123;
                for (size_t index = 0; index < inCANFrame.length; index++)
                {
                    outCANFrame.frame.data[index]++;
                }
                sendPacketToPC();
            #else
                // отправка пакета в CAN-шину
                CAN.sendMsgBuf(inCANFrame.id, (inCANFrame.id & 0x80000000u) ? 1 : 0, inCANFrame.length, inCANFrame.data);
            #endif
        }
    }
}

// Обработка управляющих пакетов
void processCommand()
{
    switch (inCANFrame.data[0])
    {
        // отключиться от CAN-шины
        case 0:
            CANDisconnect();
            break;

        // подключиться к CAN-шине на указанной скорости
        case 1:
            uint16_t speed = *(uint16_t*)(&(inCANFrame.data[1]));
            CANConnect(speed);
            break;
    }
}

// Подключиться к CAN-шине
void CANConnect(uint16_t speed)
{
    // подключить обработчик прерывания о поступлении данных по спадающему уровню сигнала
    attachInterrupt(digitalPinToInterrupt(CAN_INT), CANInterrupt, FALLING);

    MCP_BITTIME_SETUP canSpeed;
    switch (speed)
    {
        case 100:  canSpeed = CAN_100KBPS;  break;
        case 125:  canSpeed = CAN_125KBPS;  break;
        case 200:  canSpeed = CAN_200KBPS;  break;
        case 250:  canSpeed = CAN_250KBPS;  break;
        case 500:  canSpeed = CAN_500KBPS;  break;
        case 1000: canSpeed = CAN_1000KBPS; break;
        default:   canSpeed = CAN_500KBPS;  break;
    }

    // настройка скорости обмена и частоты кварца
    while (CAN.begin(canSpeed, MCP_8MHz) != CAN_OK)
    {
        delay(100);
    }

    // типы фильтров
    #define STD 0
    #define EXT 1
    
    // сброс фильтров
    CAN.init_Filt(0, STD, 0);
    CAN.init_Filt(1, STD, 0);
    CAN.init_Filt(2, STD, 0);
    CAN.init_Filt(3, STD, 0);
    CAN.init_Filt(4, STD, 0);
    CAN.init_Filt(5, STD, 0);
    
    // очистка масок
    CAN.init_Mask(0, STD, 0);
    CAN.init_Mask(1, STD, 0);
}

// Отключиться от CAN-шины
void CANDisconnect()
{
    detachInterrupt(digitalPinToInterrupt(CAN_INT));
}
