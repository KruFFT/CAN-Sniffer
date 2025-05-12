// Версия 2.3.1

// 2024.05.12 версия 2.3.1  - добавление команд управления в симулятор
//                          - исправлена потеря одного CAN-пакета при отправки данных в сеть
// 2024.05.08 версия 2.3    - перевод статистики на little-endian
//                          - команды управления подключением
//                          - переход на VSCode + PlatformIO
//                          - рефакторинг кода
// 2024.08.19 версия 2.2    - отказ от использования ArduinoSTL, переход на обычный массив
// 2024.06.15 версия 2.1    - используется библиотека CAN-BUS Shield by Seeed Studio версии 2.3.3
//                          - используется библиотека ArduinoSTL by Mike Matera 1.3.3 (необходимо исправление: https://github.com/mike-matera/ArduinoSTL/issues/95)
//                          - добавлен интервал времени между пакетами в мс
// 2022.07.02 версия 2.0    - добавлена поддержка ESP и Wi-Fi
// 2022.03.12 версия 1.2    - обновление библиотеки CAN-BUS Shield by Seeed Studio до версии 2.3.1
// 2020.06.13 версия 1.1    - добавил очистку фильтров в setup()

// AVR или ESP
#define MODE_AVR
//#define MODE_ESP

// режим работы последовательный, Wi-Fi
#define MODE_SERIAL
//#define MODE_WIRELESS

// включение симулятора для отладки
//#define MODE_SIMULATOR

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
    #define  UDP_BUFFER_MINIMUM 19                  // минимальный остаток буфера
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

// Параметры симулятора
#ifdef MODE_SIMULATOR
    #define  SIMULATOR_INTERVAL      5
    #define  SIMULATOR_LONG_INTERVAL 10000
    uint32_t simulatorTime = millis();
    uint32_t simulatorLongTime = millis();
    uint8_t  simulatorLongInc = 0x00;
    uint8_t  simulatorInc = 0x00;
    uint8_t  simulatorLongDec = 0xFF;
    uint8_t  simulatorDec = 0xFF;
    bool     simulatorExtended = false;
    bool     simulatorConnected = false;
#endif // MODE_SIMULATOR

// Инициализация
void setup()
{
    pinMode(CAN_INT, INPUT);

    #ifdef MODE_SERIAL
        Serial.begin(SERIAL_SPEED);
    #endif // MODE_SERIAL

    #ifdef MODE_WIRELESS
        // включение точки доступа
        WiFi.mode(WIFI_AP);
        WiFi.softAPConfig(IPAddress(192, 168, 1, 1), IPAddress(0, 0, 0, 0), IPAddress(255, 255, 255, 0));        
        WiFi.setSleep(false);
        while (!WiFi.softAP(ssid, password))
        {
            delay(100);
        }

        // параметры DHCP - блок не работает
        /*auto& dhcp = WiFi.softAPDhcpServer();
        dhcp.end();
        dhcps_lease dhcpParameters;
        dhcpParameters.enable = true;
        IP4_ADDR(&dhcpParameters.start_ip, 192, 168, 1, 100);
        IP4_ADDR(&dhcpParameters.end_ip, 192, 168, 1, 254);
        dhcp.set_dhcps_lease(&dhcpParameters);
        dhcp.begin();*/

        UDP.begin(udpPort);
    #endif // MODE_WIRELESS

    // включение микросхемы сдвига логических уровней 5В <-> 3,3В
    #ifdef LEVEL_SHIFTER
        pinMode(LEVEL_SHIFTER, OUTPUT);
        delay(100);
        digitalWrite(LEVEL_SHIFTER, HIGH);
    #endif // LEVEL_SHIFTER
}

// Основной цикл программы
void loop()
{
    #ifdef MODE_SIMULATOR
        if (simulatorConnected)
        {
            uint32_t currentTime = millis();
            
            if (currentTime - simulatorLongTime >= SIMULATOR_LONG_INTERVAL)
            {
                simulatorLongTime = currentTime;
                simulatorLongInc++;
                simulatorLongDec--;
            }

            if (currentTime - simulatorTime >= SIMULATOR_INTERVAL)
            {
                simulatorTime = currentTime;
                simulatorInc++;
                simulatorDec--;
                // заполнение пакета сгенерированными данными
                outCANFrame.frame.id       = 0xABCu;
                if (simulatorExtended)
                {
                    outCANFrame.frame.id  |= 0x80000000u;
                }
                outCANFrame.frame.interval = SIMULATOR_INTERVAL;
                outCANFrame.frame.length   = 8;
                outCANFrame.frame.data[0]  = simulatorLongInc;
                outCANFrame.frame.data[1]  = simulatorInc;
                outCANFrame.frame.data[2]  = simulatorLongDec;
                outCANFrame.frame.data[3]  = simulatorDec;
                outCANFrame.frame.data[4]  = simulatorLongInc;
                outCANFrame.frame.data[5]  = simulatorInc;
                outCANFrame.frame.data[6]  = simulatorLongDec;
                outCANFrame.frame.data[7]  = simulatorDec;
                sendPacketToPC();

                // замеры скоростных параметров
                calculateSpeed(currentTime);

                simulatorExtended = !simulatorExtended;
            }
        }
    #else // MODE_SIMULATOR
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

// Вычисление и получение интервала для ID
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
    if (frameTimesAmount < FRAME_INTERVALS_SIZE)
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
    // ID пакета (4 байта) + длина (1 байт) + данные
    counterBPS += (5 + outCANFrame.frame.length);
}

// Отправка полученного CAN-пакета в последовательный порт или Wi-Fi
void sendPacketToPC()
{
    // сигнатура (4 байта) + ID пакета (4 байта) + интервал (2 байта) + длина (1 байт) = 11 байт
    size_t dataLength = 11 + outCANFrame.frame.length;

    #ifdef MODE_SERIAL
        Serial.write((uint8_t*)&outCANFrame, dataLength);
    #endif // MODE_SERIAL

    #ifdef MODE_WIRELESS
        // накопление данных в буфере, это необходимо для увеличения пропускной способности
        memcpy(udpOutBufferPosition, &outCANFrame, dataLength);
        udpOutBufferPosition += dataLength;
        // если буфер заполнен - отправить порцию данных в сеть
        if (UDP_BUFFER_SIZE - (udpOutBufferPosition - udpOutBuffer) < UDP_BUFFER_MINIMUM)
        {
            // отправить данные
            UDP.beginPacket(pcIp, udpPort);
            UDP.write(udpOutBuffer, udpOutBufferPosition - udpOutBuffer);
            UDP.endPacket();
            // сброс буфера отправки пакетов
            udpOutBufferPosition = udpOutBuffer;
        }
    #endif // MODE_WIRELESS
}

// Приём и разбор данных из последовательного порта или Wi-Fi
void receivePacketFromPC()
{
    #ifdef MODE_SERIAL
        if (Serial.available())
        {
            if (parsingStep >= 0 && parsingStep <= 18)
            {
                int nextByte = Serial.read();
                // сделан поочерёдный побайтовый разбор данных из последовательного порта,
                // так как парсинг всего буфера сразу после .readBytes(..) работал нестабильно.
                // идея такая: последовательно читая каждый байт, найти сигнатуру
                // потом заполнить ID-пакета и далее длину пакета
                // далее заполнить входной массив количеством байтов до длины пакета
                switch (parsingStep)
                {
                    // поиск сигнатуры с учётом того, что придёт она с обратным порядком байтов (big-endian)
                    case 0:
                        parsingTime = millis();
                        if (nextByte == ((SIGNATURE >> 24) & 0xFFu)) parsingStep++; else parsingStep = 0;
                        break;
            
                    // поиск сигнатуры
                    case 1:
                        if (nextByte == ((SIGNATURE >> 16) & 0xFFu)) parsingStep++; else parsingStep = 0;
                        break;
            
                    // поиск сигнатуры
                    case 2:
                        if (nextByte == ((SIGNATURE >> 8) & 0xFFu)) parsingStep++; else parsingStep = 0;
                        break;

                    // поиск сигнатуры
                    case 3:
                        if (nextByte == ((SIGNATURE) & 0xFFu)) parsingStep++; else parsingStep = 0;
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
                            parsingStep = PARSING_COMPLETE;
                        break;
                }
            }
        }
        else
        {
            // если шла одна из стадий разбора пакета от компьютера, данных нет и время истекло - прекратить разбор
            if (parsingStep && (millis() - parsingTime) > PARSING_TIMEOUT)
            {
                parsingStep = 0;
            }
        }
    #endif //MODE_SERIAL

    #ifdef MODE_WIRELESS
        int packetSize = UDP.parsePacket();
        if (packetSize)// && parsingStep == 0)
        {
            int length = UDP.read(udpInBuffer, UDP_BUFFER_SIZE);
            if (length >= 11 && length <= 19)
            {
                uint8_t* receivedData = udpInBuffer;
                // проверка сигнатуры
                uint32_t signature = *(uint32_t*)receivedData;
                if (signature == SWAP_BYTES_UINT32(SIGNATURE))
                {
                    // пропустить сигнатуру и скопировать все остальные данные пакета
                    receivedData += 4;      
                    memcpy(&inCANFrame, receivedData, 15);
                    parsingStep = PARSING_COMPLETE;
                }
            }
        }
    #endif // MODE_WIRELESS

    // есть удачно разобранный пакет
    if (parsingStep == PARSING_COMPLETE)
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
            #ifdef MODE_SIMULATOR
                // немного изенить данные и отправить обратно в компьютер
                outCANFrame.frame = inCANFrame;
                outCANFrame.frame.id += 8;
                outCANFrame.frame.interval = 123;
                for (size_t index = 0; index < inCANFrame.length; index++)
                {
                    outCANFrame.frame.data[index]++;
                }
                sendPacketToPC();
            #else // MODE_SIMULATOR
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
    #ifdef MODE_SIMULATOR
        simulatorConnected = true;
    #else // MODE_SIMULATOR
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
    #endif
}

// Отключиться от CAN-шины
void CANDisconnect()
{
    #ifdef MODE_SIMULATOR
        simulatorConnected = false;
    #else // MODE_SIMULATOR
        detachInterrupt(digitalPinToInterrupt(CAN_INT));
    #endif
}

// Обработчик прерывания поступивших данных
#ifdef MODE_ESP
    IRAM_ATTR
#endif
void CANInterrupt()
{
    canDataReceived = true;     // установить флаг наличия данных в буфере
}
