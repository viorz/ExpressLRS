#pragma once

/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: Handling of the node configuration protocol

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian

Heavily modified/simplified by Alessandro Carcione 2020 for ELRS project
*/

#include "SX1262_Regs.h"
#include "SX1262.h"

enum SX1262_BusyState_
{
    SX1262_NOT_BUSY = true,
    SX1262_BUSY = false,
};

class SX1262Hal
{
public:
    static SX1262Hal *instance;

    SX1262Hal();

    void init();
    void end();
    void reset();

    void IRAM_ATTR  setNss(uint8_t radioNumber, bool state);

    void IRAM_ATTR  WriteCommand(SX1262_RadioCommands_t command, uint8_t val, SX12XX_Radio_Number_t radioNumber = SX12XX_Radio_1, uint32_t busyDelay = 15);
    void IRAM_ATTR  WriteCommand(SX1262_RadioCommands_t opcode, uint8_t *buffer, uint8_t size, SX12XX_Radio_Number_t radioNumber = SX12XX_Radio_1, uint32_t busyDelay = 15);
    void IRAM_ATTR  WriteRegister(uint16_t address, uint8_t *buffer, uint8_t size, SX12XX_Radio_Number_t radioNumber = SX12XX_Radio_1);
    void IRAM_ATTR  WriteRegister(uint16_t address, uint8_t value, SX12XX_Radio_Number_t radioNumber = SX12XX_Radio_1);

    void IRAM_ATTR  ReadCommand(SX1262_RadioCommands_t opcode, uint8_t *buffer, uint8_t size, SX12XX_Radio_Number_t radioNumber = SX12XX_Radio_1);
    void IRAM_ATTR  ReadRegister(uint16_t address, uint8_t *buffer, uint8_t size, SX12XX_Radio_Number_t radioNumber = SX12XX_Radio_1);
    uint8_t IRAM_ATTR  ReadRegister(uint16_t address, SX12XX_Radio_Number_t radioNumber = SX12XX_Radio_1);

    void IRAM_ATTR  WriteBuffer(uint8_t offset, uint8_t *buffer, uint8_t size, SX12XX_Radio_Number_t radioNumber = SX12XX_Radio_1); // Writes and Reads to FIFO
    void IRAM_ATTR  ReadBuffer(uint8_t offset, uint8_t *buffer, uint8_t size, SX12XX_Radio_Number_t radioNumber = SX12XX_Radio_1);

    bool IRAM_ATTR  WaitOnBusy(SX12XX_Radio_Number_t radioNumber = SX12XX_Radio_1);

    void IRAM_ATTR  TXenable(SX12XX_Radio_Number_t radioNumber = SX12XX_Radio_1);
    void IRAM_ATTR  RXenable();
    void IRAM_ATTR  TXRXdisable();
	
	void IRAM_ATTR  WakeUP();

    static IRAM_ATTR  void dioISR_1();
    static IRAM_ATTR  void dioISR_2();
    void (*IsrCallback_1)(); //function pointer for callback
    void (*IsrCallback_2)(); //function pointer for callback

    uint32_t BusyDelayStart;
    uint32_t BusyDelayDuration;
    void BusyDelay(uint32_t duration)
    {
        if (GPIO_PIN_BUSY == UNDEF_PIN)
        {
            BusyDelayStart = micros();
            BusyDelayDuration = duration;
        }
    }

private:
#if defined(PLATFORM_ESP32)

#else
    bool rx_enabled;
    bool tx1_enabled;
    bool tx2_enabled;
#endif
};
