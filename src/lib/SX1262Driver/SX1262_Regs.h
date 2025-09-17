#pragma once

#include "stdint.h"

#define SX1261                                      1
#define SX1262                                      2

#define SX127X_SYNC_WORD                              0x12
#define RADIO_SNR_SCALE 4
#define SX127X_SYNC_WORD                              	   0x12
#define SX1262_RADIO_WAKEUP_TIME                           3 // [ms]
#define AUTO_RX_TX_OFFSET                           	   2
#define CRC_IBM_SEED                                	   0xFFFF
#define CRC_CCITT_SEED                              	   0x1D0F
#define CRC_POLYNOMIAL_IBM                          	   0x8005
#define CRC_POLYNOMIAL_CCITT                        	   0x1021
#define SX1262_REG_LR_CRCSEEDBASEADDR                      0x06BC
#define SX1262_REG_LR_CRCPOLYBASEADDR                      0x06BE
#define SX1262_REG_LR_WHITSEEDBASEADDR_MSB                 0x06B8
#define SX1262_REG_LR_WHITSEEDBASEADDR_LSB                 0x06B9
#define SX1262_REG_LR_PACKETPARAMS                         0x0704
#define SX1262_REG_LR_PAYLOADLENGTH                        0x0702
#define SX1262_REG_LR_SYNCWORDBASEADDRESS                  0x06C0
#define SX1262_REG_LR_SYNCWORD                             0x0740
#define SX1262_LORA_MAC_PRIVATE_SYNCWORD                   0x1424
#define SX1262_LORA_MAC_PUBLIC_SYNCWORD                    0x3444
#define RANDOM_NUMBER_GENERATORBASEADDR             	   0x0819
#define SX1262_REG_RX_GAIN                                 0x08AC
#define SX1262_REG_XTA_TRIM                                0x0911
#define SX1262_REG_OCP                                     0x08E7
#define SX1262_REG_TX_MODULATION                           0x0889
#define SX1262_REG_IQ_POLARITY                             0x0736
#define SX1262_REG_RETENTION_LIST_BASE_ADDRESS             0x029F
#define MAX_NB_SX1262_REG_IN_RETENTION                     4
#define SX1262_REG_RTC_CTRL                                0x0902
#define SX1262_REG_EVT_CLR                                 0x0944

// undocumented registers
#define SX1262_REG_SENSITIVITY_CONFIG                 0x0889 // SX1268 datasheet v1.1, section 15.1
#define SX1262_REG_TX_CLAMP_CONFIG                    0x08D8 // SX1268 datasheet v1.1, section 15.2
#define SX1262_REG_RTC_STOP                           0x0920 // SX1268 datasheet v1.1, section 15.3
#define SX1262_REG_RTC_EVENT                          0x0944 // SX1268 datasheet v1.1, section 15.3
#define SX1262_REG_IQ_CONFIG                          0x0736 // SX1268 datasheet v1.1, section 15.4
#define SX1262_REG_RX_GAIN_RETENTION_0                0x029F // SX1268 datasheet v1.1, section 9.6
#define SX1262_REG_RX_GAIN_RETENTION_1                0x02A0 // SX1268 datasheet v1.1, section 9.6
#define SX1262_REG_RX_GAIN_RETENTION_2                0x02A1 // SX1268 datasheet v1.1, section 9.6

/*!
 * \brief Structure describing the radio status
 */
typedef union RadioStatus_u
{
    uint8_t Value;
    struct
    {   //bit order is lsb -> msb
        uint8_t Reserved  : 1;  //!< Reserved
        uint8_t CmdStatus : 3;  //!< Command status
        uint8_t ChipMode  : 3;  //!< Chip mode
        uint8_t CpuBusy   : 1;  //!< Flag for CPU radio busy
    }Fields;
}SX1262_RadioStatus_t;

/*!
 * \brief Structure describing the error codes for callback functions
 */
typedef enum
{
    SX1262_IRQ_HEADER_ERROR_CODE                   = 0x01,
    SX1262_IRQ_SYNCWORD_ERROR_CODE                 = 0x02,
    SX1262_IRQ_CRC_ERROR_CODE                      = 0x04,
}SX1262_IrqErrorCode_t;

enum IrqPblSyncHeaderCode_t
{
    SX1262_IRQ_PBL_DETECT_CODE                     = 0x01,
    SX1262_IRQ_SYNCWORD_VALID_CODE                 = 0x02,
    SX1262_IRQ_HEADER_VALID_CODE                   = 0x04,
};

/*!
 * \brief Represents the operating mode the radio is actually running
 */
typedef enum
{
    SX1262_MODE_SLEEP                              = 0x00,         //! The radio is in sleep mode
    SX1262_MODE_STDBY_RC,                                          //! The radio is in standby mode with RC oscillator
    SX1262_MODE_STDBY_XOSC,                                        //! The radio is in standby mode with XOSC oscillator
    SX1262_MODE_FS,                                                //! The radio is in frequency synthesis mode
    SX1262_MODE_TX,                                                //! The radio is in transmit mode
    SX1262_MODE_RX,                                                //! The radio is in receive mode
    SX1262_MODE_RX_DC,                                             //! The radio is in receive duty cycle mode
    SX1262_MODE_CAD                                                //! The radio is in channel activity detection mode
}SX1262_RadioOperatingModes_t;

/*!
 * \brief Declares the oscillator in use while in standby mode
 *
 * Using the STDBY_RC standby mode allow to reduce the energy consumption
 * STDBY_XOSC should be used for time critical applications
 */
typedef enum
{
    SX1262_STDBY_RC                                = 0x00,
    SX1262_STDBY_XOSC                              = 0x01,
}SX1262_RadioStandbyModes_t;

/*!
 * \brief Declares the power regulation used to power the device
 *
 * This command allows the user to specify if DC-DC or LDO is used for power regulation.
 * Using only LDO implies that the Rx or Tx current is doubled
 */
typedef enum
{
    SX1262_USE_LDO                                 = 0x00, // default
    SX1262_USE_DCDC                                = 0x01,
}SX1262_RadioRegulatorSX1262_MODE_t;

/*!
 * \brief Represents the possible packet type (i.e. modem) used
 */
typedef enum
{
    SX1262_PACKET_TYPE_GFSK                        = 0x00,
    SX1262_PACKET_TYPE_LORA                        = 0x01,
    SX1262_PACKET_TYPE_NONE                        = 0x0F,
}SX1262_RadioPacketTypes_t;

/*!
 * \brief Represents the ramping time for power amplifier
 */
typedef enum
{
    SX1262_RADIO_RAMP_10_US                        = 0x00,
    SX1262_RADIO_RAMP_20_US                        = 0x01,
    SX1262_RADIO_RAMP_40_US                        = 0x02,
    SX1262_RADIO_RAMP_80_US                        = 0x03,
    SX1262_RADIO_RAMP_200_US                       = 0x04,
    SX1262_RADIO_RAMP_800_US                       = 0x05,
    SX1262_RADIO_RAMP_1700_US                      = 0x06,
    SX1262_RADIO_RAMP_3400_US                      = 0x07,
}SX1262_RadioRampTimes_t;

/*!
 * \brief Represents the number of symbols to be used for channel activity detection operation
 */
typedef enum
{
    SX1262_LORA_CAD_01_SYMBOL                      = 0x00,
    SX1262_LORA_CAD_02_SYMBOL                      = 0x01,
    SX1262_LORA_CAD_04_SYMBOL                      = 0x02,
    SX1262_LORA_CAD_08_SYMBOL                      = 0x03,
    SX1262_LORA_CAD_16_SYMBOL                      = 0x04,
}SX1262_RadioLoRaCadSymbols_t;

/*!
 * \brief Represents the Channel Activity Detection actions after the CAD operation is finished
 */
typedef enum
{
    SX1262_LORA_CAD_ONLY                           = 0x00,
    SX1262_LORA_CAD_RX                             = 0x01,
    SX1262_LORA_CAD_LBT                            = 0x10,
}SX1262_RadioCadExitModes_t;

/*!
 * \brief Represents the modulation shaping parameter
 */
typedef enum
{
    SX1262_MOD_SHAPING_OFF                         = 0x00,
    SX1262_MOD_SHAPING_G_BT_03                     = 0x08,
    SX1262_MOD_SHAPING_G_BT_05                     = 0x09,
    SX1262_MOD_SHAPING_G_BT_07                     = 0x0A,
    SX1262_MOD_SHAPING_G_BT_1                      = 0x0B,
}SX1262_RadioModShapings_t;

/*!
 * \brief Represents the modulation shaping parameter
 */
typedef enum
{
    RX_BW_4800                              = 0x1F,
    RX_BW_5800                              = 0x17,
    RX_BW_7300                              = 0x0F,
    RX_BW_9700                              = 0x1E,
    RX_BW_11700                             = 0x16,
    RX_BW_14600                             = 0x0E,
    RX_BW_19500                             = 0x1D,
    RX_BW_23400                             = 0x15,
    RX_BW_29300                             = 0x0D,
    RX_BW_39000                             = 0x1C,
    RX_BW_46900                             = 0x14,
    RX_BW_58600                             = 0x0C,
    RX_BW_78200                             = 0x1B,
    RX_BW_93800                             = 0x13,
    RX_BW_117300                            = 0x0B,
    RX_BW_156200                            = 0x1A,
    RX_BW_187200                            = 0x12,
    RX_BW_234300                            = 0x0A,
    RX_BW_312000                            = 0x19,
    RX_BW_373600                            = 0x11,
    RX_BW_467000                            = 0x09,
}SX1262_RadioRxBandwidth_t;

/*!
 * \brief Represents the possible spreading factor values in LoRa packet types
 */
typedef enum
{
    SX1262_LORA_SF5                                = 0x05,
    SX1262_LORA_SF6                                = 0x06,
    SX1262_LORA_SF7                                = 0x07,
    SX1262_LORA_SF8                                = 0x08,
    SX1262_LORA_SF9                                = 0x09,
    SX1262_LORA_SF10                               = 0x0A,
    SX1262_LORA_SF11                               = 0x0B,
    SX1262_LORA_SF12                               = 0x0C,
}SX1262_RadioLoRaSpreadingFactors_t;

/*!
 * \brief Represents the bandwidth values for LoRa packet type
 */
typedef enum
{
    SX1262_LORA_BW_500                             = 6,
    SX1262_LORA_BW_250                             = 5,
    SX1262_LORA_BW_125                             = 4,
    SX1262_LORA_BW_062                             = 3,
    SX1262_LORA_BW_041                             = 10,
    SX1262_LORA_BW_031                             = 2,
    SX1262_LORA_BW_020                             = 9,
    SX1262_LORA_BW_015                             = 1,
    SX1262_LORA_BW_010                             = 8,
    SX1262_LORA_BW_007                             = 0,
}SX1262_RadioLoRaBandwidths_t;

/*!
 * \brief Represents the coding rate values for LoRa packet type
 */
typedef enum
{
    SX1262_LORA_CR_4_5                             = 0x01,
    SX1262_LORA_CR_4_6                             = 0x02,
    SX1262_LORA_CR_4_7                             = 0x03,
    SX1262_LORA_CR_4_8                             = 0x04,
}SX1262_RadioLoRaCodingRates_t;

/*!
 * \brief Represents the preamble length used to detect the packet on Rx side
 */
typedef enum
{
    SX1262_RADIO_PREAMBLE_DETECTOR_OFF             = 0x00,         //!< Preamble detection length off
    SX1262_RADIO_PREAMBLE_DETECTOR_08_BITS         = 0x04,         //!< Preamble detection length 8 bits
    SX1262_RADIO_PREAMBLE_DETECTOR_16_BITS         = 0x05,         //!< Preamble detection length 16 bits
    SX1262_RADIO_PREAMBLE_DETECTOR_24_BITS         = 0x06,         //!< Preamble detection length 24 bits
    SX1262_RADIO_PREAMBLE_DETECTOR_32_BITS         = 0x07,         //!< Preamble detection length 32 bit
}SX1262_RadioPreambleDetection_t;

/*!
 * \brief Represents the possible combinations of SyncWord correlators activated
 */
typedef enum
{
    SX1262_RADIO_ADDRESSCOMP_FILT_OFF              = 0x00,         //!< No correlator turned on, i.e. do not search for SyncWord
    SX1262_RADIO_ADDRESSCOMP_FILT_NODE             = 0x01,
    SX1262_RADIO_ADDRESSCOMP_FILT_NODE_BROAD       = 0x02,
}RadioAddressComp_t;

/*!
 *  \brief Radio GFSK packet length mode
 */
typedef enum
{
    SX1262_RADIO_PACKET_FIXED_LENGTH               = 0x00,         //!< The packet is known on both sides, no header included in the packet
    SX1262_RADIO_PACKET_VARIABLE_LENGTH            = 0x01,         //!< The packet is on variable size, header included
}SX1262_RadioPacketLengthModes_t;

/*!
 * \brief Represents the CRC length
 */
typedef enum
{
    SX1262_RADIO_CRC_OFF                           = 0x01,         //!< No CRC in use
    SX1262_RADIO_CRC_1_BYTES                       = 0x00,
    SX1262_RADIO_CRC_2_BYTES                       = 0x02,
    SX1262_RADIO_CRC_1_BYTES_INV                   = 0x04,
    SX1262_RADIO_CRC_2_BYTES_INV                   = 0x06,
    SX1262_RADIO_CRC_2_BYTES_IBM                   = 0xF1,
    SX1262_RADIO_CRC_2_BYTES_CCIT                  = 0xF2,
}SX1262_RadioCrcTypes_t;

/*!
 * \brief Radio whitening mode activated or deactivated
 */
typedef enum
{
    SX1262_RADIO_DC_FREE_OFF                       = 0x00,
    SX1262_RADIO_DC_FREEWHITENING                  = 0x01,
}SX1262_RadioDcFree_t;

/*!
 * \brief Holds the Radio lengths mode for the LoRa packet type
 */
typedef enum
{
    SX1262_LORA_PACKET_VARIABLE_LENGTH             = 0x00,         //!< The packet is on variable size, header included
    SX1262_LORA_PACKET_FIXED_LENGTH                = 0x01,         //!< The packet is known on both sides, no header included in the packet
    SX1262_LORA_PACKET_EXPLICIT                    = SX1262_LORA_PACKET_VARIABLE_LENGTH,
    SX1262_LORA_PACKET_IMPLICIT                    = SX1262_LORA_PACKET_FIXED_LENGTH,
}SX1262_RadioLoRaPacketLengthsModes_t;

/*!
 * \brief Represents the CRC mode for LoRa packet type
 */
typedef enum
{
    SX1262_LORA_CRC_ON                             = 0x01,         //!< CRC activated
    SX1262_LORA_CRC_OFF                            = 0x00,         //!< CRC not used
}SX1262_RadioLoRaCrcModes_t;

/*!
 * \brief Represents the IQ mode for LoRa packet type
 */
typedef enum
{
    SX1262_LORA_IQ_NORMAL                          = 0x00,
    SX1262_LORA_IQ_INVERTED                        = 0x01,
}SX1262_RadioLoRaIQModes_t;

/*!
 * \brief Represents the voltage used to control the TCXO on/off from DIO3
 */
typedef enum
{
    SX1262_TCXO_CTRL_1_6V                          = 0x00,
    SX1262_TCXO_CTRL_1_7V                          = 0x01,
    SX1262_TCXO_CTRL_1_8V                          = 0x02,
    SX1262_TCXO_CTRL_2_2V                          = 0x03,
    SX1262_TCXO_CTRL_2_4V                          = 0x04,
    SX1262_TCXO_CTRL_2_7V                          = 0x05,
    SX1262_TCXO_CTRL_3_0V                          = 0x06,
    SX1262_TCXO_CTRL_3_3V                          = 0x07,
}SX1262_RadioTcxoCtrlVoltage_t;

/*!
 * \brief Represents the interruption masks available for the radio
 *
 * \remark Note that not all these interruptions are available for all packet types
 */
typedef enum
{
    SX1262_IRQ_RADIO_NONE                          = 0x0000,
    SX1262_IRQ_TX_DONE                             = 0x0001,
    SX1262_IRQ_RX_DONE                             = 0x0002,
    SX1262_IRQ_PREAMBLE_DETECTED                   = 0x0004,
    SX1262_IRQ_SYNCWORD_VALID                      = 0x0008,
    SX1262_IRQ_HEADER_VALID                        = 0x0010,
    SX1262_IRQ_HEADER_ERROR                        = 0x0020,
    SX1262_IRQ_CRC_ERROR                           = 0x0040,
    SX1262_IRQ_CAD_DONE                            = 0x0080,
    SX1262_IRQ_CAD_ACTIVITY_DETECTED               = 0x0100,
    SX1262_IRQ_RX_TX_TIMEOUT                       = 0x0200,
    SX1262_IRQ_RADIO_ALL                           = 0xFFFF,
}RadioIrqMasks_t;

/*!
 * \brief Represents all possible opcode understood by the radio
 */
typedef enum RadioCommands_e
{
    SX1262_RADIO_GET_STATUS                        = 0xC0,
    SX1262_RADIO_WRITE_REGISTER                    = 0x0D,
    SX1262_RADIO_READ_REGISTER                     = 0x1D,
    SX1262_RADIO_WRITE_BUFFER                      = 0x0E,
    SX1262_RADIO_READ_BUFFER                       = 0x1E,
    SX1262_RADIO_SET_SLEEP                         = 0x84,
    SX1262_RADIO_SET_STANDBY                       = 0x80,
    SX1262_RADIO_SET_FS                            = 0xC1,
    SX1262_RADIO_SET_TX                            = 0x83,
    SX1262_RADIO_SET_RX                            = 0x82,
    SX1262_RADIO_SET_RXDUTYCYCLE                   = 0x94,
    SX1262_RADIO_SET_CAD                           = 0xC5,
    SX1262_RADIO_SET_TXCONTINUOUSWAVE              = 0xD1,
    SX1262_RADIO_SET_TXCONTINUOUSPREAMBLE          = 0xD2,
    SX1262_RADIO_SET_PACKETTYPE                    = 0x8A,
    SX1262_RADIO_GET_PACKETTYPE                    = 0x11,
    SX1262_RADIO_SET_RFFREQUENCY                   = 0x86,
    SX1262_RADIO_SET_TXPARAMS                      = 0x8E,
    SX1262_RADIO_SET_PACONFIG                      = 0x95,
    SX1262_RADIO_SET_CADPARAMS                     = 0x88,
    SX1262_RADIO_SET_BUFFERBASEADDRESS             = 0x8F,
    SX1262_RADIO_SET_MODULATIONPARAMS              = 0x8B,
    SX1262_RADIO_SET_PACKETPARAMS                  = 0x8C,
    SX1262_RADIO_GET_RXBUFFERSTATUS                = 0x13,
    SX1262_RADIO_GET_PACKETSTATUS                  = 0x14,
    SX1262_RADIO_GET_RSSIINST                      = 0x15,
    SX1262_RADIO_GET_STATS                         = 0x10,
    SX1262_RADIO_RESET_STATS                       = 0x00,
    SX1262_RADIO_SET_DIOIRQPARAMS                  = 0x08,
    SX1262_RADIO_GET_IRQSTATUS                     = 0x12,
    SX1262_RADIO_CLR_IRQSTATUS                     = 0x02,
    SX1262_RADIO_CALIBRATE                         = 0x89,
    SX1262_RADIO_CALIBRATEIMAGE                    = 0x98,
    SX1262_RADIO_SET_REGULATORMODE                 = 0x96,
    SX1262_RADIO_GET_ERROR                         = 0x17,
    SX1262_RADIO_CLR_ERROR                         = 0x07,
    SX1262_RADIO_SET_TCXOMODE                      = 0x97,
    SX1262_RADIO_SET_TXFALLBACKMODE                = 0x93,
    SX1262_RADIO_SET_RFSWITCHMODE                  = 0x9D,
    SX1262_RADIO_SET_STOPRXTIMERONPREAMBLE         = 0x9F,
    SX1262_RADIO_SET_LORASYMBTIMEOUT               = 0xA0,
}SX1262_RadioCommands_t;

/*!
 * \brief Represents a calibration configuration
 */
typedef union
{
    struct
    {
        uint8_t RC64KEnable    : 1;                             //!< Calibrate RC64K clock
        uint8_t RC13MEnable    : 1;                             //!< Calibrate RC13M clock
        uint8_t PLLEnable      : 1;                             //!< Calibrate PLL
        uint8_t ADCPulseEnable : 1;                             //!< Calibrate ADC Pulse
        uint8_t ADCBulkNEnable : 1;                             //!< Calibrate ADC bulkN
        uint8_t ADCBulkPEnable : 1;                             //!< Calibrate ADC bulkP
        uint8_t ImgEnable      : 1;
        uint8_t                : 1;
    }Fields;
    uint8_t Value;
}CalibrationParams_t;

/*!
 * \brief Represents a sleep mode configuration
 */
typedef union
{
    struct
    {
        uint8_t WakeUpRTC               : 1;                    //!< Get out of sleep mode if wakeup signal received from RTC
        uint8_t Reset                   : 1;
        uint8_t WarmStart               : 1;
        uint8_t Reserved                : 5;
    }Fields;
    uint8_t Value;
}SleepParams_t;

/*!
 * \brief Represents the possible radio system error states
 */
 
//SX1262_CMD_GET_DEVICE_ERRORS
#define SX1262_PA_RAMP_ERR                           0b100000000  //  8     8     device errors: PA ramping failed
#define SX1262_PLL_LOCK_ERR                          0b001000000  //  6     6                    PLL failed to lock
#define SX1262_XOSC_START_ERR                        0b000100000  //  5     5                    crystal oscillator failed to start
#define SX1262_IMG_CALIB_ERR                         0b000010000  //  4     4                    image calibration failed
#define SX1262_ADC_CALIB_ERR                         0b000001000  //  3     3                    ADC calibration failed
#define SX1262_PLL_CALIB_ERR                         0b000000100  //  2     2                    PLL calibration failed
#define SX1262_RC13M_CALIB_ERR                       0b000000010  //  1     1                    RC13M calibration failed
#define SX1262_RC64K_CALIB_ERR                       0b000000001  //  0     0                    RC64K calibration failed
 
typedef union
{
    struct
    {
        uint8_t Rc64kCalib              : 1;                    //!< RC 64kHz oscillator calibration failed
        uint8_t Rc13mCalib              : 1;                    //!< RC 13MHz oscillator calibration failed
        uint8_t PllCalib                : 1;                    //!< PLL calibration failed
        uint8_t AdcCalib                : 1;                    //!< ADC calibration failed
        uint8_t ImgCalib                : 1;                    //!< Image calibration failed
        uint8_t XoscStart               : 1;                    //!< XOSC oscillator failed to start
        uint8_t PllLock                 : 1;                    //!< PLL lock failed
        uint8_t BuckStart               : 1;                    //!< Buck converter failed to start
        uint8_t PaRamp                  : 1;                    //!< PA ramp failed
        uint8_t                         : 7;                    //!< Reserved
    }Fields;
    uint16_t Value;
}RadioError_t;

/*!
 * Hardware IO IRQ callback function definition
 */
typedef void ( DioIrqHandler )( void );

/*!
 * \brief Provides the frequency of the chip running on the radio and the frequency step
 *
 * \remark These defines are used for computing the frequency divider to set the RF frequency
 */
#define XTAL_FREQ                                   ( double )32000000
#define FREQ_DIV                                    ( double )pow( 2.0, 25.0 )
#define FREQ_STEP                                   ( double )( XTAL_FREQ / FREQ_DIV )

#define RX_BUFFER_SIZE                              256