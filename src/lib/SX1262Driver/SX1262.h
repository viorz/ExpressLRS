#pragma once

#include "targets.h"
#include "SX1262_Regs.h"
#include "SX1262_hal.h"
#include "SX12xxDriverCommon.h"

#define RADIO_SNR_SCALE 4 // Units for LastPacketSNRRaw
#define SX1262_POWER_MIN 0
#define SX1262_POWER_MAX 22
#define RX_TIMEOUT_PERIOD_BASE SX126X_RADIO_TICK_SIZE_0015_US
#define RX_TIMEOUT_PERIOD_BASE_NANOS 15625

class SX1262Driver: public SX12xxDriverCommon
{
public:
    static SX1262Driver *instance;

    ///////////Radio Variables////////
    //#define TXRXBuffSize 16
    //volatile uint8_t TXdataBuffer[TXRXBuffSize];
    //volatile uint8_t RXdataBuffer[TXRXBuffSize];


    ///////////Radio Variables////////
    uint32_t timeout;
	uint8_t currBW;

    ///////////////////////////////////

    ////////////////Configuration Functions/////////////
    SX1262Driver();
    bool Begin();
    void End();
    void SetTxIdleMode() { SetMode(SX1262_MODE_FS, SX12XX_Radio_All); }; // set Idle mode used when switching from RX to TX
    void Config(uint8_t bw, uint8_t sf, uint8_t cr, uint32_t freq,
                uint8_t PreambleLength, bool InvertIQ, uint8_t PayloadLength, uint32_t interval,
                uint32_t flrcSyncWord=0, uint16_t flrcCrcSeed=0, uint8_t flrc=0);
    void SetFrequencyHz(uint32_t freq, SX12XX_Radio_Number_t radioNumber);
    void SetFrequencyReg(uint32_t freq, SX12XX_Radio_Number_t radioNumber = SX12XX_Radio_1);
    void SetRxTimeoutUs(uint32_t interval);
    void SetOutputPower(int8_t power);
    void startCWTest(uint32_t freq, SX12XX_Radio_Number_t radioNumber);
	void Calibrate( CalibrationParams_t calibParam);
	void SetSyncWord(uint8_t syncWord);


    bool GetFrequencyErrorbool();
    bool FrequencyErrorAvailable() const { return modeSupportsFei && (LastPacketSNRRaw > 0); }

    void TXnb(uint8_t * data, uint8_t size, SX12XX_Radio_Number_t radioNumber = SX12XX_Radio_1);
    void RXnb(SX1262_RadioOperatingModes_t rxMode = SX1262_MODE_RX);

    uint16_t GetIrqStatus(SX12XX_Radio_Number_t radioNumber = SX12XX_Radio_1);
    void ClearIrqStatus(uint16_t irqMask, SX12XX_Radio_Number_t radioNumber = SX12XX_Radio_1);

    uint8_t GetStatus(SX12XX_Radio_Number_t radioNumber = SX12XX_Radio_1);
	void SetDio3AsTcxoCtrl(SX1262_RadioTcxoCtrlVoltage_t voltage, uint32_t delay);
	uint16_t getDeviceErrors();
	void clearDeviceErrors();

    uint8_t GetRxBufferAddr(SX12XX_Radio_Number_t radioNumber = SX12XX_Radio_1);
    int8_t GetRssiInst(SX12XX_Radio_Number_t radioNumber = SX12XX_Radio_1);
    void GetLastPacketStats();
    SX12XX_Radio_Number_t GetProcessingPacketRadio() { return processingPacketRadio; }
    SX12XX_Radio_Number_t GetLastSuccessfulPacketRadio() { return lastSuccessfulPacketRadio; }

private:
    // constant used for no power change pending
    // must not be a valid power register value
    static const uint8_t PWRPENDING_NONE = 0xff;
	
	uint8_t currSyncWord = SX127X_SYNC_WORD;

    SX1262_RadioOperatingModes_t currOpmode;
    uint8_t packet_mode;
    bool modeSupportsFei;
    SX12XX_Radio_Number_t processingPacketRadio;
    SX12XX_Radio_Number_t lastSuccessfulPacketRadio;
    uint8_t pwrCurrent;
    uint8_t pwrPending;

    void SetMode(SX1262_RadioOperatingModes_t OPmode, SX12XX_Radio_Number_t radioNumber = SX12XX_Radio_1);
    void SetFIFOaddr(uint8_t txBaseAddr, uint8_t rxBaseAddr);
	void CalibrateImage( uint32_t freq );

    // LoRa functions
    void ConfigModParamsLoRa(uint8_t bw, uint8_t sf, uint8_t cr);
    void SetPacketParamsLoRa(uint8_t PreambleLength, SX1262_RadioLoRaPacketLengthsModes_t HeaderType,
                             uint8_t PayloadLength, uint8_t InvertIQ);
	void setWorkaround500Khz();
	void setWorkaroundIQPolarity();
	void setWorkaroundRXTimeout();

    void SetDioIrqParams(uint16_t irqMask,
                         uint16_t dio1Mask=SX1262_IRQ_RADIO_NONE,
                         uint16_t dio2Mask=SX1262_IRQ_RADIO_NONE,
                         uint16_t dio3Mask=SX1262_IRQ_RADIO_NONE);

    static void IsrCallback_1();
    static void IsrCallback_2();
    static void IsrCallback(SX12XX_Radio_Number_t radioNumber = SX12XX_Radio_1);
    bool RXnbISR(uint16_t irqStatus, SX12XX_Radio_Number_t radioNumber = SX12XX_Radio_1); // ISR for non-blocking RX routine
    void TXnbISR(); // ISR for non-blocking TX routine
    void CommitOutputPower();
};
