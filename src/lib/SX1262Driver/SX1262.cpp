#include "SX1262_Regs.h"
#include "SX1262_hal.h"
#include "SX1262.h"
#include "logging.h"

SX1262Hal hal;
SX1262Driver *SX1262Driver::instance = NULL;
static bool ImageCalibrated = false;

#if defined(DEBUG_SX1262_OTA_TIMING)
static uint32_t beginTX;
static uint32_t endTX;
#endif

const uint8_t AllowedSyncwords[105] =
    {0, 5, 6, 7, 11, 12, 13, 15, 18,
     21, 23, 26, 29, 30, 31, 33, 34,
     37, 38, 39, 40, 42, 44, 50, 51,
     54, 55, 57, 58, 59, 61, 63, 65,
     67, 68, 71, 77, 78, 79, 80, 82,
     84, 86, 89, 92, 94, 96, 97, 99,
     101, 102, 105, 106, 109, 111, 113, 115,
     117, 118, 119, 121, 122, 124, 126, 127,
     129, 130, 138, 143, 161, 170, 172, 173,
     175, 180, 181, 182, 187, 190, 191, 192,
     193, 196, 199, 201, 204, 205, 208, 209,
     212, 213, 219, 220, 221, 223, 227, 229,
     235, 239, 240, 242, 243, 246, 247, 255};

SX1262Driver::SX1262Driver(): SX12xxDriverCommon()
{
    instance = this;
    timeout = 0xffff;
    currOpmode = SX1262_MODE_SLEEP;
    lastSuccessfulPacketRadio = SX12XX_Radio_1;
}

void SX1262Driver::End()
{
    if (currOpmode != SX1262_MODE_SLEEP)
    {
        SetMode(SX1262_MODE_SLEEP, SX12XX_Radio_All);
    }
    hal.end();
    RemoveCallbacks();
}

bool SX1262Driver::Begin()
{
    hal.IsrCallback_1 = &SX1262Driver::IsrCallback_1;
    hal.IsrCallback_2 = &SX1262Driver::IsrCallback_2;

    hal.init();
    hal.reset();
    DBGLN("SX1262 Begin");
	
	DBGLN("Status = %x", GetStatus(SX12XX_Radio_1));
	
    SetMode(SX1262_MODE_STDBY_RC, SX12XX_Radio_1);

    CalibrationParams_t calibParam;
    SetDio3AsTcxoCtrl( SX1262_TCXO_CTRL_3_3V, 5 << 6); // convert from ms to SX126x time base
    calibParam.Value = 0x7F;
    Calibrate( calibParam );
	CalibrateImage(868000000);

    hal.WriteCommand( SX1262_RADIO_SET_RFSWITCHMODE, (uint8_t*)&(uint8_t){1}, 1); //dio2 as rf switch
	hal.WriteRegister( SX1262_REG_TX_CLAMP_CONFIG, hal.ReadRegister( SX1262_REG_TX_CLAMP_CONFIG ) | ( 0x1E ) );
	hal.WriteCommand( SX1262_RADIO_SET_STOPRXTIMERONPREAMBLE, (uint8_t*)&(uint8_t){false}, 1);
	SetSyncWord(currSyncWord);
    SetDioIrqParams(SX1262_IRQ_RADIO_NONE);

    // Force the next power update, and the lowest power
    pwrCurrent = PWRPENDING_NONE;
    SetOutputPower(SX1262_POWER_MIN);
    CommitOutputPower();

    return true;
}

uint16_t SX1262Driver::getDeviceErrors() {
  uint8_t data[2] = {0, 0};
  hal.ReadCommand(SX1262_RADIO_GET_ERROR, data, 2);
  uint16_t opError = (((uint16_t)data[0] & 0xFF) << 8) & ((uint16_t)data[1]);
  return(opError);
}

void SX1262Driver::clearDeviceErrors() {
  uint8_t data[2] = {0, 0};
  hal.WriteCommand(SX1262_RADIO_CLR_ERROR, data, 2);
  return;
}


void SX1262Driver::Calibrate( CalibrationParams_t calibParam )
{
    hal.WriteCommand(SX1262_RADIO_CALIBRATE, ( uint8_t* )&calibParam, 1 );
	delayMicroseconds(3500);
}

 void SX1262Driver::SetDio3AsTcxoCtrl(SX1262_RadioTcxoCtrlVoltage_t voltage, uint32_t delay)
{

  SetMode(SX1262_MODE_STDBY_RC);

  // check alowed voltage values
  uint8_t data[4];

  data[0] = voltage;
  data[1] = 0;
  data[2] = 0;
  data[3] = 10;

  hal.WriteCommand( SX1262_RADIO_SET_TCXOMODE, data, 4 );
  
}



void SX1262Driver::CalibrateImage( uint32_t freq )
{
    uint8_t calFreq[2];

    if( freq > 900000000 )
    {
        calFreq[0] = 0xE1;
        calFreq[1] = 0xE9;
    }
    else if( freq > 850000000 )
    {
        calFreq[0] = 0xD7;
        calFreq[1] = 0xDB;
    }
    else if( freq > 770000000 )
    {
        calFreq[0] = 0xC1;
        calFreq[1] = 0xC5;
    }
    else if( freq > 460000000 )
    {
        calFreq[0] = 0x75;
        calFreq[1] = 0x81;
    }
    else if( freq > 425000000 )
    {
        calFreq[0] = 0x6B;
        calFreq[1] = 0x6F;
    }
    hal.WriteCommand(SX1262_RADIO_CALIBRATEIMAGE, calFreq, 2 );
}

void SX1262Driver::Config(uint8_t bw, uint8_t sf, uint8_t cr, uint32_t regfreq,
                          uint8_t PreambleLength, bool InvertIQ, uint8_t _PayloadLength, uint32_t interval,
                          uint32_t flrcSyncWord, uint16_t flrcCrcSeed, uint8_t flrc)
{
    uint8_t irqs = SX1262_IRQ_TX_DONE | SX1262_IRQ_RX_DONE;
    uint8_t const mode = SX1262_PACKET_TYPE_LORA;

    PayloadLength = _PayloadLength;
    IQinverted = InvertIQ;
	currBW = bw;
    SetMode(SX1262_MODE_STDBY_RC);
    hal.WriteCommand(SX1262_RADIO_SET_PACKETTYPE, (uint8_t*)&mode, 1);
    ConfigModParamsLoRa(bw, sf, cr);
    SetPacketParamsLoRa(PreambleLength, SX1262_LORA_PACKET_IMPLICIT,_PayloadLength, InvertIQ);
	setWorkaroundIQPolarity();
    SetFrequencyReg(regfreq);
    SetDioIrqParams(irqs, irqs);
    SetRxTimeoutUs(interval);
	hal.WriteCommand( SX1262_RADIO_SET_LORASYMBTIMEOUT, (uint8_t*)&(uint8_t){0}, 1);
}

void SX1262Driver::setWorkaround500Khz()
{
	if (currBW == SX1262_LORA_BW_500)
		hal.WriteRegister( SX1262_REG_TX_MODULATION, hal.ReadRegister( SX1262_REG_TX_MODULATION ) & (0xFB) );
}

void SX1262Driver::setWorkaroundRXTimeout()
{
	hal.WriteRegister( SX1262_REG_RTC_CTRL, 0x00 );
	hal.WriteRegister( SX1262_REG_EVT_CLR, hal.ReadRegister( SX1262_REG_EVT_CLR ) & (0x02) );
}

void SX1262Driver::setWorkaroundIQPolarity()
{
	if (IQinverted)
		hal.WriteRegister( SX1262_REG_IQ_CONFIG, hal.ReadRegister( SX1262_REG_IQ_CONFIG ) | (0x04) );
	else
		hal.WriteRegister( SX1262_REG_IQ_CONFIG, hal.ReadRegister( SX1262_REG_IQ_CONFIG ) & (0xFB) );
}

void SX1262Driver::SetRxTimeoutUs(uint32_t interval)
{
    if (interval)
    {
        timeout = interval * 1000 / RX_TIMEOUT_PERIOD_BASE_NANOS; // number of periods for the SX1262 to timeout
    }
    else
    {
        timeout = 0xFFFFFF;   // no timeout, continuous mode
    }
}

bool SyncWordOk(uint8_t syncWord)
{
  for (unsigned int i = 0; i < sizeof(AllowedSyncwords); i++)
  {
    if (syncWord == AllowedSyncwords[i])
    {
      return true;
    }
  }
  return false;
}

void SX1262Driver::SetSyncWord(uint8_t syncWord)
{
  uint8_t _syncWord = syncWord;

  while (SyncWordOk(_syncWord) == false)
  {
    _syncWord++;
  }

  if(syncWord != _syncWord){
    DBGLN("Using syncword: %d instead of: %d", _syncWord, syncWord);
  }

  hal.WriteRegister(SX1262_REG_LR_SYNCWORD, _syncWord);
  currSyncWord = _syncWord;
}

/***
 * @brief: Schedule an output power change after the next transmit
 ***/
void SX1262Driver::SetOutputPower(int8_t power)
{
    uint8_t pwrNew = power;//constrain(power, SX1262_POWER_MIN, SX1262_POWER_MAX) + (-SX1262_POWER_MIN);

    if ((pwrPending == PWRPENDING_NONE && pwrCurrent != pwrNew) || pwrPending != pwrNew)
    {
        pwrPending = pwrNew;
        DBGLN("SetPower: %u", pwrPending);
    }
}

void ICACHE_RAM_ATTR SX1262Driver::CommitOutputPower()
{

	uint8_t buf[4];

    if (pwrPending == PWRPENDING_NONE)
        return;

    pwrCurrent = pwrPending;
    pwrPending = PWRPENDING_NONE;

    buf[0] = 0x04;
    buf[1] = 0x07;
    buf[2] = 0x00;
    buf[3] = 0x01;
    hal.WriteCommand( SX1262_RADIO_SET_PACONFIG, buf, 4);
	
	hal.WriteRegister( SX1262_REG_OCP, 0x38, SX12XX_Radio_1); //SET OCP max to 140ma
	
    buf[0] = pwrCurrent;//map(power, 0,15,0,22);
    buf[1] = ( uint8_t )SX1262_RADIO_RAMP_10_US;
    hal.WriteCommand(SX1262_RADIO_SET_TXPARAMS, buf, 2);
    //DBGLN("SetPower: %d", buf[0]);
}


void SX1262Driver::SetMode(SX1262_RadioOperatingModes_t OPmode, SX12XX_Radio_Number_t radioNumber)
{
    /*
    Comment out since it is difficult to keep track of dual radios.
    When checking SPI it is also useful to see every possible SPI transaction to make sure it fits when required.
    */
     //if (OPmode == currOpmode)
     //{
     //   return;
     //}

    uint8_t buf[3];
	
    switch (OPmode)
    {

    case SX1262_MODE_SLEEP:
        hal.WriteCommand(SX1262_RADIO_SET_SLEEP, (uint8_t*)&(uint8_t){0x01}, 1, radioNumber);
        break;

    case SX1262_MODE_STDBY_RC:
        hal.WriteCommand(SX1262_RADIO_SET_STANDBY, (uint8_t*)&(uint8_t){SX1262_STDBY_RC}, 1, radioNumber, 1500);
        break;

    // The DC-DC supply regulation is automatically powered in STDBY_XOSC mode.
    case SX1262_MODE_STDBY_XOSC:
        hal.WriteCommand(SX1262_RADIO_SET_STANDBY, (uint8_t*)&(uint8_t){SX1262_STDBY_XOSC}, 1, radioNumber, 50);
        break;

    case SX1262_MODE_FS:
        hal.WriteCommand(SX1262_RADIO_SET_FS, (uint8_t*)&(uint8_t){0x00}, 1, radioNumber, 70);
        break;

    case SX1262_MODE_RX:
        buf[0] = ( uint8_t )( ( timeout >> 16 ) & 0xFF );
        buf[1] = ( uint8_t )( ( timeout >> 8 ) & 0xFF );
        buf[2] = ( uint8_t )( timeout & 0xFF );
		hal.WriteRegister(SX1262_REG_RX_GAIN, 0x96 ); 
        hal.WriteCommand(SX1262_RADIO_SET_RX, buf, sizeof(buf), radioNumber, 100);
        break;

    case SX1262_MODE_TX:
		setWorkaround500Khz();
        buf[0] = ( uint8_t )( ( 0 >> 16 ) & 0xFF );
        buf[1] = ( uint8_t )( ( 0 >> 8 ) & 0xFF );
        buf[2] = ( uint8_t )( 0 & 0xFF );
        hal.WriteCommand(SX1262_RADIO_SET_TX, buf, sizeof(buf), radioNumber, 100);
        break;

    case SX1262_MODE_CAD:
        break;

    default:
        break;
    }

    currOpmode = OPmode;
}

void SX1262Driver::ConfigModParamsLoRa(uint8_t bw, uint8_t sf, uint8_t cr)
{
    // Care must therefore be taken to ensure that modulation parameters are set using the command
    // SetModulationParam() only after defining the packet type SetPacketType() to be used

    uint8_t buf[4] = { 0x00, 0x00, 0x00, 0x00};
    buf[0] = sf;
    buf[1] = bw;
    buf[2] = cr;
    buf[3] = 0; //low data rate optimise

    hal.WriteCommand(SX1262_RADIO_SET_MODULATIONPARAMS, buf, 4);
}

void SX1262Driver::SetPacketParamsLoRa(uint8_t PreambleLength, SX1262_RadioLoRaPacketLengthsModes_t HeaderType,
                                       uint8_t PayloadLength, uint8_t InvertIQ)
{
    uint8_t buf[9];

    buf[0] = ( PreambleLength >> 8 ) & 0xFF;
    buf[1] = PreambleLength;
    buf[2] = HeaderType;
    buf[3] = PayloadLength;
    buf[4] = SX1262_LORA_CRC_OFF;
    buf[5] = InvertIQ ? SX1262_LORA_IQ_NORMAL: SX1262_LORA_IQ_INVERTED;
	buf[6] = 0xff;
	buf[7] = 0xff;
	buf[8] = 0xff;
	
    hal.WriteCommand(SX1262_RADIO_SET_PACKETPARAMS, buf, 9);
}


void ICACHE_RAM_ATTR SX1262Driver::SetFrequencyHz(uint32_t Reqfreq, SX12XX_Radio_Number_t radioNumber)
{
    uint8_t buf[4] = {0};
	
	SetMode(SX1262_MODE_STDBY_RC);

    if( ImageCalibrated == false )
    {
        CalibrateImage( Reqfreq );
        ImageCalibrated = true;
    }

    uint32_t freq = (uint32_t)((double)Reqfreq / (double)FREQ_STEP);
    buf[0] = ( uint8_t )( ( freq >> 24 ) & 0xFF );
    buf[1] = ( uint8_t )( ( freq >> 16 ) & 0xFF );
    buf[2] = ( uint8_t )( ( freq >> 8 ) & 0xFF );
    buf[3] = ( uint8_t )( freq & 0xFF );
	
    hal.WriteCommand(SX1262_RADIO_SET_RFFREQUENCY, buf, 4);
    currFreq = Reqfreq;
}

void ICACHE_RAM_ATTR SX1262Driver::SetFrequencyReg(uint32_t freq, SX12XX_Radio_Number_t radioNumber)
{
    uint8_t buf[4] = {0};
	
	SetMode(SX1262_MODE_STDBY_RC);
	
    if( ImageCalibrated == false )
    {
        CalibrateImage((uint32_t)((double)freq * (double)FREQ_STEP));
        ImageCalibrated = true;
    }

    buf[0] = ( uint8_t )( ( freq >> 24 ) & 0xFF );
    buf[1] = ( uint8_t )( ( freq >> 16 ) & 0xFF );
    buf[2] = ( uint8_t )( ( freq >> 8 ) & 0xFF );
    buf[3] = ( uint8_t )( freq & 0xFF );

    hal.WriteCommand(SX1262_RADIO_SET_RFFREQUENCY, buf, 4);
    currFreq = freq;
}

void SX1262Driver::SetFIFOaddr(uint8_t txBaseAddr, uint8_t rxBaseAddr)
{
    uint8_t buf[2];

    buf[0] = txBaseAddr;
    buf[1] = rxBaseAddr;
    hal.WriteCommand(SX1262_RADIO_SET_BUFFERBASEADDRESS, buf, sizeof(buf), SX12XX_Radio_All);
}

void SX1262Driver::SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask)
{
    uint8_t buf[8];

    buf[0] = (uint8_t)((irqMask >> 8) & 0x00FF);
    buf[1] = (uint8_t)(irqMask & 0x00FF);
    buf[2] = (uint8_t)((dio1Mask >> 8) & 0x00FF);
    buf[3] = (uint8_t)(dio1Mask & 0x00FF);
    buf[4] = (uint8_t)((dio2Mask >> 8) & 0x00FF);
    buf[5] = (uint8_t)(dio2Mask & 0x00FF);
    buf[6] = (uint8_t)((dio3Mask >> 8) & 0x00FF);
    buf[7] = (uint8_t)(dio3Mask & 0x00FF);

    hal.WriteCommand(SX1262_RADIO_SET_DIOIRQPARAMS, buf, sizeof(buf), SX12XX_Radio_All);
}

uint16_t ICACHE_RAM_ATTR SX1262Driver::GetIrqStatus(SX12XX_Radio_Number_t radioNumber)
{
    uint8_t status[2];

    hal.ReadCommand(SX1262_RADIO_GET_IRQSTATUS, status, 2, radioNumber);
    return status[0] << 8 | status[1];
}

void ICACHE_RAM_ATTR SX1262Driver::ClearIrqStatus(uint16_t irqMask, SX12XX_Radio_Number_t radioNumber)
{
    uint8_t buf[2];

    buf[0] = (uint8_t)(((uint16_t)irqMask >> 8) & 0x00FF);
    buf[1] = (uint8_t)((uint16_t)irqMask & 0x00FF);

    hal.WriteCommand(SX1262_RADIO_CLR_IRQSTATUS, buf, sizeof(buf), radioNumber);
}

void ICACHE_RAM_ATTR SX1262Driver::TXnbISR()
{
    currOpmode = SX1262_MODE_STDBY_RC; // radio goes to FS after TX
    // The power level must be changed when in STDBY mode, so this lags power
    // changes by at most 1 packet, but does not interrupt any pending RX/TX
    CommitOutputPower();
    TXdoneCallback();
}

void ICACHE_RAM_ATTR SX1262Driver::TXnb(uint8_t * data, uint8_t size, SX12XX_Radio_Number_t radioNumber)
{

    hal.WriteBuffer(0x00, data, size, radioNumber); //todo fix offset to equal fifo addr
	SetMode(SX1262_MODE_STDBY_XOSC);
    instance->SetMode(SX1262_MODE_TX);

#ifdef DEBUG_SX1262_OTA_TIMING
    beginTX = micros();
#endif
}

bool ICACHE_RAM_ATTR SX1262Driver::RXnbISR(uint16_t irqStatus, SX12XX_Radio_Number_t radioNumber)
{

   uint8_t const FIFOaddr = GetRxBufferAddr(radioNumber);
   hal.ReadBuffer(FIFOaddr, RXdataBuffer, PayloadLength, radioNumber);
   
   return RXdoneCallback(SX12XX_RX_OK);
}

void ICACHE_RAM_ATTR SX1262Driver::RXnb(SX1262_RadioOperatingModes_t rxMode)
{
	SetMode(SX1262_MODE_STDBY_XOSC);
	SetFIFOaddr(0x00, 0x00);
    SetMode(rxMode, SX12XX_Radio_1);
}

uint8_t ICACHE_RAM_ATTR SX1262Driver::GetRxBufferAddr(SX12XX_Radio_Number_t radioNumber)
{
    uint8_t status[2] = {0};
    hal.ReadCommand(SX1262_RADIO_GET_RXBUFFERSTATUS, status, 2, radioNumber);
    return status[1];
}

uint8_t ICACHE_RAM_ATTR SX1262Driver::GetStatus(SX12XX_Radio_Number_t radioNumber)
{
    uint8_t status = 0;
    hal.ReadCommand(SX1262_RADIO_GET_STATUS, (uint8_t *)&status, 1, radioNumber);
    DBGLN("Status: %x, %x, %x", (0b11100000 & status) >> 5, (0b00011100 & status) >> 2, 0b00000001 & status);
	return status;
}

bool ICACHE_RAM_ATTR SX1262Driver::GetFrequencyErrorbool()
{
   return 0;
}

int8_t ICACHE_RAM_ATTR SX1262Driver::GetRssiInst(SX12XX_Radio_Number_t radioNumber)
{
    uint8_t status = 0;

    hal.ReadCommand(SX1262_RADIO_GET_RSSIINST, (uint8_t *)&status, 1, radioNumber);
    return -(int8_t)(status / 2);
}

void ICACHE_RAM_ATTR SX1262Driver::GetLastPacketStats()
{
    uint8_t status[3];

    hal.ReadCommand(SX1262_RADIO_GET_PACKETSTATUS, status, sizeof(status));

    LastPacketRSSI = -status[0] >> 1;
    LastPacketSNRRaw = (int8_t)status[1];
}

void ICACHE_RAM_ATTR SX1262Driver::IsrCallback_1()
{
    instance->IsrCallback(SX12XX_Radio_1);
}

void ICACHE_RAM_ATTR SX1262Driver::IsrCallback_2()
{
    instance->IsrCallback(SX12XX_Radio_2);
}

void ICACHE_RAM_ATTR SX1262Driver::IsrCallback(SX12XX_Radio_Number_t radioNumber)
{
    instance->processingPacketRadio = radioNumber;
    SX12XX_Radio_Number_t irqClearRadio = radioNumber;

    uint16_t irqStatus = instance->GetIrqStatus(radioNumber);
	instance->ClearIrqStatus(SX1262_IRQ_RADIO_ALL, irqClearRadio);
    if (irqStatus & SX1262_IRQ_TX_DONE)
    {
        instance->TXnbISR();
        irqClearRadio = SX12XX_Radio_All;
    }
    else if (irqStatus & SX1262_IRQ_RX_DONE)
    {
        if (instance->RXnbISR(irqStatus, radioNumber))
        {
            instance->lastSuccessfulPacketRadio = radioNumber;
            irqClearRadio = SX12XX_Radio_All; // Packet received so clear all radios and dont spend extra time retrieving data.
        }
    }
    else if (irqStatus == SX1262_IRQ_RADIO_NONE)
    {
        return;
    }
}
