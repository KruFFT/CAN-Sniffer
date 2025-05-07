#include "main.h"

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
    #ifdef MODE_SIMULATOR
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
            #ifdef MODE_SIMULATOR
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
