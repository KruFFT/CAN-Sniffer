// Версия 2.2

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

//////////////////////////////////////////////////////////////////////////////////////////////////
// Выбор типа микроконтроллера                                                                  //
#define AVR                                                                                     //
//#define ESP                                                                                   //
//////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////
// Выбор режима работы                                                                          //
#define SERIAL                                                                                  //
//#define WIRELESS                                                                              //
//////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////
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
    bool     testSend = true;
#endif

#include <SPI.h>
#include <mcp2515_can.h>

//////////////////////////////////////////////////////////////////////////////////////////////////
// Параметры подключения к AVR
#ifdef AVR
    // Для AVR (Arduino) шилд подключен следующим образом:
    // CS   - D10
    // MISO - D12
    // MOSI - D11
    // SCK  - D13
    // INT  - D2
    
    #define CAN_INT 2   // сигнал INT на D2
    #define CAN_CS 10   // сигнал CS на D10
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////
// Параметры подключения к ESP
#ifdef ESP
    // Для ESP шилд подключен следующим образом:
    // INT  - GPIO5  (D1)
    // SCK  - GPIO14 (D5)
    // MOSI - GPIO13 (D7)
    // MISO - GPIO12 (D6)
    // CS   - GPIO15 (D8)
    
    #define CAN_INT 5   // сигнал INT на GPIO5 (D1)
    #define CAN_CS  15  // сигнал CS на GPIO15 (D8)
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////
// Настройки беспроводного режима работы
#ifdef WIRELESS
    #include <ESPWiFi.h>
    #include <WiFiUdp.h>

    // настройки точки доступа Wi-Fi
    const char* ssid       = "CAR";                 // название беспроводной сети
    const char* password   = "1357924680";          // пароль к сети
    const uint16_t udpPort = 0xAA55;                // номер порта для обмена данными
    WiFiUDP UDP;                                    // UDP-сокет

    #define  UDP_BUFFER_SIZE 1000                   // размер буфера пакетов
    uint8_t  udpOutBuffer[UDP_BUFFER_SIZE] = { 0 }; // буфер накопления пакетов на отправку на компьютер
    uint8_t* udpOutBufferPosition = udpOutBuffer;   // указатель на этот буфер
    uint8_t  udpInBuffer[UDP_BUFFER_SIZE] = { 0 };  // буфер приёма пакета от компьютера
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////
// Структура данных CAN-пакета
struct CANFrame
{
    uint32_t id;        // идентификатор пакета
    uint16_t interval;  // интервал между пакетами
    uint8_t  length;    // длина данных
    uint8_t  data[8];   // сами данные
};

//////////////////////////////////////////////////////////////////////////////////////////////////
// Сигнатура и различные структуры, буферы и счётчики
#define SIGNATURE 0x55AA55AA        // сигнатура передаваемых пакетов

// структура хранения данных отправляемых в последовательный порт или через Wi-Fi
struct OutCANFrame
{
    uint32_t signature = SIGNATURE;
    CANFrame frame;
};

OutCANFrame outCANFrame;            // буфер отправки данных в SERIAL-порт или в буфер передачи через Wi-Fi
CANFrame    inCANFrame  = { 0 };    // принятый из последовательного порта или через Wi-Fi пакет для отправки в CAN-шину

#ifdef SERIAL
    CANFrame comInFrame = { 0 };    // буфер приёма пакета от компьютера
    uint8_t  comInFrameStep = 0;    // переменная стадии обработки входящих данных из SERIAL-порта
#endif

// переменные для замера различных скоростных параметров
uint16_t counterFPS  = 0;           // CAN-пакеты в секунду
uint16_t counterBPS  = 0;           // байты в секунду
uint32_t counterTime = millis();    // счётчик времени для подсчёта количества CAN-пакетов и байтов в секунду

//////////////////////////////////////////////////////////////////////////////////////////////////
// Массив хранения ID пакетов и их временных интервалов
#define FRAME_TIMES_LIMIT 100

// описание номера пакета и его времени
struct FrameTime
{
    uint32_t ID;
    uint32_t ms;
};

// массив интервалов между CAN-пакетами
FrameTime frameTimes[FRAME_TIMES_LIMIT + 1] = {{ 0, 0 }};
uint16_t frameTimesAmount = 0;

uint16_t getAndSaveIdInterval(uint32_t id, uint32_t currentTime)
{
    // поиск по массиву
    for (auto index = 0; index < frameTimesAmount; index++)
    {
        if (frameTimes[index].ID == id)
        {
            uint32_t interval = currentTime - frameTimes[index].ms;
            frameTimes[index].ms = currentTime;
            return (interval < 65536) ? (uint16_t)interval : 0;
        }
    }
    // пакет не найден - надо добавить
    if (frameTimesAmount < FRAME_TIMES_LIMIT)
    {
        frameTimes[frameTimesAmount].ID = id;
        frameTimes[frameTimesAmount].ms = currentTime;
        frameTimesAmount++;
    }
    return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// Флаг для обработки прерывания поступления данных из CAN-шины и инициализация CAN
volatile bool canDataReceived = false;
mcp2515_can CAN(CAN_CS);

//////////////////////////////////////////////////////////////////////////////////////////////////
// Обработчик прерывания поступивших данных
//////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef ESP
    ICACHE_RAM_ATTR
#endif
void CANInterrupt()
{
    canDataReceived = true;     // установить флаг наличия данных в буфере
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// Инициализация
//////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
    #ifdef SERIAL
        Serial.begin(500000);
    #endif

    // подключить обработчик прерывания о поступлении данных по спадающему уровню сигнала
    pinMode(CAN_INT, INPUT);
    attachInterrupt(digitalPinToInterrupt(CAN_INT), CANInterrupt, FALLING);

    // настройка скорости обмена и частоты кварца
    while (CAN.begin(CAN_500KBPS, MCP_8MHz) != CAN_OK)
    {
        delay(100);
    }
    // типы фильтров
    #define STD 0
    #define EXT 1
    
    // сброс фильтров
    CAN.init_Filt(0, STD, 0x0);
    CAN.init_Filt(1, STD, 0x0);
    CAN.init_Filt(2, STD, 0x0);
    CAN.init_Filt(3, STD, 0x0);
    CAN.init_Filt(4, STD, 0x0);
    CAN.init_Filt(5, STD, 0x0);
    
    // очистка масок
    CAN.init_Mask(0, STD, 0x0);
    CAN.init_Mask(1, STD, 0x0);

    #ifdef WIRELESS
        // включение точки доступа
        WiFi.softAPConfig(IPAddress(192, 168, 1, 1),
                          IPAddress(192, 168, 1, 1),
                          IPAddress(255, 255, 255, 0));
        
        WiFi.mode(WIFI_AP);
        while (!WiFi.softAP(ssid, password))
        {
            delay(100);
        }
        UDP.begin(udpPort);
    #endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// Основной цикл программы
//////////////////////////////////////////////////////////////////////////////////////////////////
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
            outCANFrame.frame.id       = 0xABC;
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

            if (testSend)
            {
                sendPacketToPC();
            }

            // замеры скоростных параметров
            calculateSpeed(currentTime);
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

                    sendPacketToPC();

                    // замеры скоростных параметров
                    calculateSpeed(currentTime);
                }
            }
        }
    #endif

    #ifdef WIRELESS
        receivePacket();
    #endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// Подсчёт скоростных показателей CAN-пакетов
//////////////////////////////////////////////////////////////////////////////////////////////////
void calculateSpeed(uint32_t currentTime)
{
    if (currentTime - counterTime >= 1000)
    {
        counterTime = currentTime;
        outCANFrame.frame.id = 0;
        outCANFrame.frame.length = 4;
        // при копировании little-endian, вместо big-endian, поэтому надо побайтово делать
        // количество пакетов в секунду, примерная скорость около 745 пакетов в секунду
        outCANFrame.frame.data[0] = highByte(counterFPS);  // counterFPS >> 8 & 0xFF;
        outCANFrame.frame.data[1] = lowByte(counterFPS);   // counterFPS & 0xFF;
        // количество байтов в секунду, примерная скорость около 9200 байтов в секунду
        outCANFrame.frame.data[2] = highByte(counterBPS);  // counterBPS >> 8 & 0xFF;
        outCANFrame.frame.data[3] = lowByte(counterBPS);   // counterBPS & 0xFF;

        sendPacketToPC();
        
        counterFPS = 0;
        counterBPS = 0;
    }

    counterFPS++;
    counterBPS += (5 + outCANFrame.frame.length);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// Отправка полученного CAN-пакета в последовательный порт или Wi-Fi
//////////////////////////////////////////////////////////////////////////////////////////////////
void sendPacketToPC()
{
    // сигнатура (4 байта) + ID пакета (4 байта) + интервал (2 байта) + длина (1 байт) = 11 байт
    size_t dataLength = 11 + outCANFrame.frame.length;
    
    #ifdef SERIAL
        Serial.write((uint8_t*)&outCANFrame, dataLength);
    #endif

    #ifdef WIRELESS
        // накопление данных в буфере, это необходимо для увеличения пропускной способности
        if (UDP_BUFFER_SIZE - (udpOutBufferPosition - udpOutBuffer) >= 17)
        {
            // добавить данные в буфер
            memcpy(udpOutBufferPosition, &outCANFrame, dataLength);
            udpOutBufferPosition += dataLength;
        }
        else
        {
            // отправить данные
            UDP.beginPacket(IPAddress(192, 168, 1, 255), udpPort);
            UDP.write(udpOutBuffer, udpOutBufferPosition - udpOutBuffer);
            UDP.endPacket();

            udpOutBufferPosition = udpOutBuffer;
        }
    #endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// Проверка и приём данных из Wi-Fi
//////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef WIRELESS
void receivePacket()
{
    int packetSize = UDP.parsePacket();
    if (packetSize)
    {
        int length = UDP.read(udpInBuffer, UDP_BUFFER_SIZE);
        if (length > 0)
        {
            // если принятые данные распарсились, то в inCANFrame будет принятый пакет
            if (parseBuffer(udpInBuffer, length))
            {
                if (CAN.sendMsgBuf(inCANFrame.id, 0, inCANFrame.length, inCANFrame.data) == CAN_OK)
                {
                    // отправилось без ошибок
                }
            }
        }
    }
}
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////
// Парсер принятых данных
//////////////////////////////////////////////////////////////////////////////////////////////////
bool parseBuffer(uint8_t* receivedData, size_t length)
{
    if (length >= 10 && length <= 17)
    {
        // проверка сигнатуры 
        if (*(uint32_t*)receivedData == SIGNATURE)
        {
            receivedData += 4;
            inCANFrame.id = *(uint32_t*)receivedData;
            receivedData += 4;
            inCANFrame.length = *receivedData;
            receivedData++;
            for (size_t i = 0; i < inCANFrame.length; i++)
            {
                inCANFrame.data[i] = *receivedData++;
            }
            return true;
        }
    }
    return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// Прерывание обработки данных из последовательного порта
//////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef SERIAL
void serialEvent()
{
    // так как пакет данных через последовательный порт приходит генерируя несколько прерываний,
    // то не стал менять этот код и оставил из старой версии

    int nextByte;

    // чтение из SERIAL-порта, пока там есть данные
    while (Serial.available())
    {
        /*
        идея такая: последовательно читая каждый байт, найти сигнатуру
        потом заполнить ID-пакета и далее длину пакета
        далее заполнить входной массив количеством байтов до длины пакета
        */
        switch (comInFrameStep)
        {
            // поиск заголовка
            case 0: case 2:
                if (Serial.read() == 0xAA)
                    comInFrameStep++;
                else
                    comInFrameStep = 0;
                break;
    
            // поиск заголовка
            case 1: case 3:
                if (Serial.read() == 0x55)
                    comInFrameStep++;
                else
                    comInFrameStep = 0;
                break;
    
            // получение ID-пакета
            case 4 ... 7:
                nextByte = Serial.read();
                *((uint8_t*)&comInFrame.id + comInFrameStep - 4) = (uint8_t)nextByte;
                comInFrameStep++;
                break;
    
            // получение длины данных
            case 8:
                nextByte = Serial.read();
                if (nextByte > 0 && nextByte <= 8)
                {
                    comInFrame.length = (uint8_t)nextByte;
                    comInFrameStep++;
                }
                else
                    comInFrameStep = 0;
                break;
    
            // получение непосредственно самих данных пакета
            case 9 ... 16:
                if (comInFrameStep - 9 < comInFrame.length)
                {
                    nextByte = Serial.read();
                    comInFrame.data[comInFrameStep - 9] = (uint8_t)nextByte;
                    comInFrameStep++;
                }
                else
                    comInFrameStep = 17;
                break;
        }

        // отправка собранного пакета в CAN-шину
        if (comInFrameStep == 17)
        {
            comInFrameStep = 0;
            if (CAN.sendMsgBuf(comInFrame.id, 0, 0, comInFrame.length, comInFrame.data, true) == CAN_OK)
            {
                // отправилось без ошибок
            }
            #ifdef TEST
                outCANFrame.frame = comInFrame;
                outCANFrame.frame.id = 0x7E8;
                outCANFrame.frame.interval = 0;
                outCANFrame.frame.length = 8;
                outCANFrame.frame.data[0]++;
                outCANFrame.frame.data[1]++;
                outCANFrame.frame.data[2]++;
                outCANFrame.frame.data[3]++;
                outCANFrame.frame.data[4]++;
                outCANFrame.frame.data[5]++;
                outCANFrame.frame.data[6]++;
                outCANFrame.frame.data[7]++;
                sendPacketToPC();
                testSend = !testSend;
            #endif
        }
    }
}
#endif
