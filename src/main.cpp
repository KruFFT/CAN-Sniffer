#include "main.h"

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
