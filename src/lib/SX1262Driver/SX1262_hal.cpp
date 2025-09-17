/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2016 Semtech

Description: Handling of the node configuration protocol

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Matthieu Verdy

Modified and adapted by Alessandro Carcione for ELRS project
*/

#ifndef UNIT_TEST
#include "SX1262_Regs.h"
#include "SX1262_hal.h"
#include <SPI.h>
#include "logging.h"

SX1262Hal *SX1262Hal::instance = NULL;

SX1262Hal::SX1262Hal()
{
    instance = this;
}

void SX1262Hal::end()
{
    TXRXdisable(); // make sure the RX/TX amp pins are disabled
    detachInterrupt(GPIO_PIN_DIO1);

    SPI.end();
    IsrCallback_1 = nullptr; // remove callbacks
    IsrCallback_2 = nullptr; // remove callbacks
}

void SX1262Hal::init()
{
    DBGLN("Hal Init");

    if (GPIO_PIN_BUSY != UNDEF_PIN)
    {
        pinMode(GPIO_PIN_BUSY, INPUT);
    }
    if (GPIO_PIN_BUSY_2 != UNDEF_PIN)
    {
        pinMode(GPIO_PIN_BUSY_2, INPUT);
    }

    pinMode(GPIO_PIN_DIO1, INPUT);
    if (GPIO_PIN_DIO1_2 != UNDEF_PIN)
    {
        pinMode(GPIO_PIN_DIO1_2, INPUT);
    }

    pinMode(GPIO_PIN_NSS, OUTPUT);
    if (GPIO_PIN_NSS_2 != UNDEF_PIN)
    {
        pinMode(GPIO_PIN_NSS_2, OUTPUT);
        digitalWrite(GPIO_PIN_NSS_2, HIGH);
    }

    if (GPIO_PIN_PA_ENABLE != UNDEF_PIN)
    {
        DBGLN("Use PA enable pin: %d", GPIO_PIN_PA_ENABLE);
        pinMode(GPIO_PIN_PA_ENABLE, OUTPUT);
        digitalWrite(GPIO_PIN_PA_ENABLE, LOW);
    }

    if (GPIO_PIN_TX_ENABLE != UNDEF_PIN)
    {
        DBGLN("Use TX pin: %d", GPIO_PIN_TX_ENABLE);
        pinMode(GPIO_PIN_TX_ENABLE, OUTPUT);
        digitalWrite(GPIO_PIN_TX_ENABLE, LOW);
    }

    if (GPIO_PIN_RX_ENABLE != UNDEF_PIN)
    {
        DBGLN("Use RX pin: %d", GPIO_PIN_RX_ENABLE);
        pinMode(GPIO_PIN_RX_ENABLE, OUTPUT);
        digitalWrite(GPIO_PIN_RX_ENABLE, LOW);
    }

    if (GPIO_PIN_TX_ENABLE_2 != UNDEF_PIN)
    {
        DBGLN("Use TX_2 pin: %d", GPIO_PIN_TX_ENABLE_2);
        pinMode(GPIO_PIN_TX_ENABLE_2, OUTPUT);
        digitalWrite(GPIO_PIN_TX_ENABLE_2, LOW);
    }

    if (GPIO_PIN_RX_ENABLE_2 != UNDEF_PIN)
    {
        DBGLN("Use RX_2 pin: %d", GPIO_PIN_RX_ENABLE_2);
        pinMode(GPIO_PIN_RX_ENABLE_2, OUTPUT);
        digitalWrite(GPIO_PIN_RX_ENABLE_2, LOW);
    }

#ifdef PLATFORM_ESP32
	DBGLN("SPI pins SCK=%d MISO=%d MOSI=%d NSS=%d", GPIO_PIN_SCK,GPIO_PIN_MISO,GPIO_PIN_MOSI,-1);
    SPI.begin(GPIO_PIN_SCK, GPIO_PIN_MISO, GPIO_PIN_MOSI); // sck, miso, mosi, ss (ss can be any GPIO)
	//SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
	//SPI.endTransaction();
    //gpio_pullup_en((gpio_num_t)GPIO_PIN_NSS);
	//gpio_pullup_en((gpio_num_t)5);
    SPI.setFrequency(10000000);
    //SPI.setHwCs(true);
    //if (GPIO_PIN_NSS_2 != UNDEF_PIN) spiAttachSS(SPI.bus(), 1, GPIO_PIN_NSS_2);
    //spiEnableSSPins(SPI.bus(), SX12XX_Radio_All);
#elif defined(PLATFORM_ESP8266)
    DBGLN("PLATFORM_ESP8266");
    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.setFrequency(10000000);
#elif defined(PLATFORM_STM32)
    DBGLN("Config SPI");
    SPI.setMOSI(GPIO_PIN_MOSI);
    SPI.setMISO(GPIO_PIN_MISO);
    SPI.setSCLK(GPIO_PIN_SCK);
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV4); // 72 / 8 = 9 MHz
#endif

    //attachInterrupt(digitalPinToInterrupt(GPIO_PIN_BUSY), this->busyISR, CHANGE); //not used atm
    attachInterrupt(digitalPinToInterrupt(GPIO_PIN_DIO1), this->dioISR_1, RISING);
}

void ICACHE_RAM_ATTR SX1262Hal::setNss(uint8_t radioNumber, bool state)
{
    digitalWrite(GPIO_PIN_NSS, state);
}

void SX1262Hal::reset(void)
{
    DBGLN("SX1262 Reset");

    if (GPIO_PIN_RST != UNDEF_PIN)
    {
		
        pinMode(GPIO_PIN_RST, OUTPUT);
        delay(50); // Safety buffer. Busy takes longer to go low than the 1ms timeout in WaitOnBusy().
        digitalWrite(GPIO_PIN_RST, LOW);
        delay(50);
        digitalWrite(GPIO_PIN_RST, HIGH);
        delay(50); // Safety buffer. Busy takes longer to go low than the 1ms timeout in WaitOnBusy().
		DBGLN("SX1262 Reseting good");
    }
	
	digitalWrite(GPIO_PIN_NSS, HIGH);
    WaitOnBusy(SX12XX_Radio_1);

    //this->BusyState = SX1262_NOT_BUSY;
    DBGLN("SX1262 Ready!");
}

void ICACHE_RAM_ATTR SX1262Hal::WriteCommand(SX1262_RadioCommands_t command, uint8_t val, SX12XX_Radio_Number_t radioNumber, uint32_t busyDelay)
{
    WriteCommand(command, &val, 1, radioNumber, busyDelay);
}

void ICACHE_RAM_ATTR SX1262Hal::WriteCommand(SX1262_RadioCommands_t command, uint8_t *buffer, uint8_t size, SX12XX_Radio_Number_t radioNumber, uint32_t busyDelay)
{
    uint8_t OutBuffer[size + 1];

    OutBuffer[0] = (uint8_t)command;
    memcpy(OutBuffer + 1, buffer, size);

    WaitOnBusy(radioNumber);
    setNss(radioNumber, LOW);
    SPI.transfer(OutBuffer, (uint8_t)sizeof(OutBuffer));
    setNss(radioNumber, HIGH);
}

void ICACHE_RAM_ATTR SX1262Hal::ReadCommand(SX1262_RadioCommands_t command, uint8_t *buffer, uint8_t size, SX12XX_Radio_Number_t radioNumber)
{
     uint8_t OutBuffer[size + 2];

    WaitOnBusy(radioNumber);
    setNss(radioNumber, LOW);

    OutBuffer[0] = (uint8_t)command;
    OutBuffer[1] = 0x00;
    memcpy(OutBuffer + 2, buffer, size);
    SPI.transfer(OutBuffer, sizeof(OutBuffer));
    memcpy(buffer, OutBuffer + 2, size);
    setNss(radioNumber, HIGH);
}

void ICACHE_RAM_ATTR SX1262Hal::WriteRegister(uint16_t address, uint8_t *buffer, uint8_t size, SX12XX_Radio_Number_t radioNumber)
{
     uint8_t OutBuffer[size + 3];

    OutBuffer[0] = (SX1262_RADIO_WRITE_REGISTER);
    OutBuffer[1] = ((address & 0xFF00) >> 8);
    OutBuffer[2] = (address & 0x00FF);

    memcpy(OutBuffer + 3, buffer, size);

    WaitOnBusy(radioNumber);
    setNss(radioNumber, LOW);
    SPI.transfer(OutBuffer, (uint8_t)sizeof(OutBuffer));
    setNss(radioNumber, HIGH);
}

void ICACHE_RAM_ATTR SX1262Hal::WriteRegister(uint16_t address, uint8_t value, SX12XX_Radio_Number_t radioNumber)
{
    WriteRegister(address, &value, 1, radioNumber);
}

void ICACHE_RAM_ATTR SX1262Hal::ReadRegister(uint16_t address, uint8_t *buffer, uint8_t size, SX12XX_Radio_Number_t radioNumber)
{
    uint8_t OutBuffer[size + 4];

    OutBuffer[0] = (SX1262_RADIO_READ_REGISTER);
    OutBuffer[1] = ((address & 0xFF00) >> 8);
    OutBuffer[2] = (address & 0x00FF);
    OutBuffer[3] = 0x00;

    WaitOnBusy(radioNumber);
    setNss(radioNumber, LOW);
    SPI.transfer(OutBuffer, uint8_t(sizeof(OutBuffer)));
    memcpy(buffer, OutBuffer + 4, size);

    setNss(radioNumber, HIGH);
}

uint8_t ICACHE_RAM_ATTR SX1262Hal::ReadRegister(uint16_t address, SX12XX_Radio_Number_t radioNumber)
{
    uint8_t data;
    ReadRegister(address, &data, 1, radioNumber);
    return data;
}

void ICACHE_RAM_ATTR SX1262Hal::WriteBuffer(uint8_t offset, uint8_t *buffer, uint8_t size, SX12XX_Radio_Number_t radioNumber)
{
    uint8_t localbuf[size];

    for (int i = 0; i < size; i++) // todo check if this is the right want to handle volatiles
    {
        localbuf[i] = buffer[i];
    }

    uint8_t OutBuffer[size + 2];

    OutBuffer[0] = SX1262_RADIO_WRITE_BUFFER;
    OutBuffer[1] = offset;

    memcpy(OutBuffer + 2, localbuf, size);

    WaitOnBusy(radioNumber);

    setNss(radioNumber, LOW);
    SPI.transfer(OutBuffer, (uint8_t)sizeof(OutBuffer));
    setNss(radioNumber, HIGH);
}

void ICACHE_RAM_ATTR SX1262Hal::ReadBuffer(uint8_t offset, uint8_t *buffer, uint8_t size, SX12XX_Radio_Number_t radioNumber)
{
    uint8_t OutBuffer[size + 3];
    uint8_t localbuf[size];

    OutBuffer[0] = SX1262_RADIO_READ_BUFFER;
    OutBuffer[1] = offset;
    OutBuffer[2] = 0x00;

    WaitOnBusy(radioNumber);

    setNss(radioNumber, LOW);
    SPI.transfer(OutBuffer, uint8_t(sizeof(OutBuffer)));
    setNss(radioNumber, HIGH);

    memcpy(localbuf, OutBuffer + 3, size);

    for (int i = 0; i < size; i++) // todo check if this is the right wany to handle volatiles
    {
        buffer[i] = localbuf[i];
    }
}

bool ICACHE_RAM_ATTR SX1262Hal::WaitOnBusy(SX12XX_Radio_Number_t radioNumber)
{
	//return true;
    #define wtimeoutUS 10000
    uint32_t startTime = micros();

        while (digitalRead(GPIO_PIN_BUSY))
        {
        if ((micros() - startTime) > wtimeoutUS)
        {
            return false;
        }
        else
        {
           NOP();
        }
		}
    return true;
}

void ICACHE_RAM_ATTR SX1262Hal::dioISR_1()
{
    if (instance->IsrCallback_1)
        instance->IsrCallback_1();
}

void ICACHE_RAM_ATTR SX1262Hal::dioISR_2()
{
    if (instance->IsrCallback_2)
        instance->IsrCallback_2();
}

void ICACHE_RAM_ATTR SX1262Hal::TXenable(SX12XX_Radio_Number_t radioNumber)
{
}

void ICACHE_RAM_ATTR SX1262Hal::RXenable()
{
}

void ICACHE_RAM_ATTR SX1262Hal::TXRXdisable()
{
}

#endif // UNIT_TEST
