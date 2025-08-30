/*! ----------------------------------------------------------------------------
 * @file    deca_device_api.h
 * @brief   DW3000 API Functions
 *
 * @attention
 *
 * Copyright 2013-2020(c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */

#ifndef _DECA_DEVICE_API_H_
#define _DECA_DEVICE_API_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "deca_types.h"

#ifndef DWT_NUM_DW_DEV
#define DWT_NUM_DW_DEV (1)
#endif

#define DWT_BIT_MASK(bit_num)   (((uint32_t)1)<<(bit_num))

/* MACRO */
#define dwt_write32bitreg(addr,value)  dwt_write32bitoffsetreg(addr,0,value)
#define dwt_read32bitreg(addr)     dwt_read32bitoffsetreg(addr,0)
#define dwt_writefastCMD(cmd)     dwt_writetodevice(cmd,0,0,0)

#define dwt_or8bitoffsetreg(addr, offset, or_val) dwt_modify8bitoffsetreg(addr, offset, -1, or_val)
#define dwt_and8bitoffsetreg(addr, offset, and_val) dwt_modify8bitoffsetreg(addr, offset, and_val, 0)
#define dwt_and_or8bitoffsetreg(addr,offset, and_val, or_val) dwt_modify8bitoffsetreg(addr,offset,and_val,or_val)
#define dwt_set_bit_num_8bit_reg(addr,bit_num) dwt_modify8bitoffsetreg(addr,0,-1,DWT_BIT_MASK(bit_num))
#define dwt_clr_bit_num_8bit_reg(addr,bit_num) dwt_modify8bitoffsetreg(addr,0,~DWT_BIT_MASK(bit_num),0)

#define dwt_or16bitoffsetreg(addr, offset, or_val) dwt_modify16bitoffsetreg(addr, offset, -1, or_val)
#define dwt_and16bitoffsetreg(addr, offset, and_val) dwt_modify16bitoffsetreg(addr, offset, and_val, 0)
#define dwt_and_or16bitoffsetreg(addr,offset, and_val, or_val) dwt_modify16bitoffsetreg(addr,offset,and_val,or_val)
#define dwt_set_bit_num_16bit_reg(addr,bit_num) dwt_modify16bitoffsetreg(addr,0,-1,DWT_BIT_MASK(bit_num))
#define dwt_clr_bit_num_16bit_reg(addr,bit_num) dwt_modify16bitoffsetreg(addr,0,~DWT_BIT_MASK(bit_num),0)

#define dwt_or32bitoffsetreg(addr, offset, or_val) dwt_modify32bitoffsetreg(addr, offset, -1, or_val)
#define dwt_and32bitoffsetreg(addr, offset, and_val) dwt_modify32bitoffsetreg(addr, offset, and_val, 0)
#define dwt_and_or32bitoffsetreg(addr,offset, and_val, or_val) dwt_modify32bitoffsetreg(addr,offset,and_val,or_val)
#define dwt_set_bit_num_32bit_reg(addr,bit_num) dwt_modify32bitoffsetreg(addr,0,-1,DWT_BIT_MASK(bit_num))
#define dwt_clr_bit_num_32bit_reg(addr,bit_num) dwt_modify32bitoffsetreg(addr,0,~DWT_BIT_MASK(bit_num),0)

/* */
#define DWT_SUCCESS (0)
#define DWT_ERROR   (-1)

#define DWT_TIME_UNITS      (1.0/499.2e6/128.0) //!< = 15.65e-12 s

#define DWT_A0_DEV_ID       (0xDECA0300)        //!< DW3000 MPW A0 (non PDOA) silicon device ID
#define DWT_A0_PDOA_DEV_ID  (0xDECA0310)        //!< DW3000 MPW A0 (with PDOA) silicon device ID
#define DWT_B0_DEV_ID       (0xDECA0301)        //!< DW3000 MPW B0 (non PDOA) silicon device ID
#define DWT_B0_PDOA_DEV_ID  (0xDECA0311)        //!< DW3000 MPW B0 (with PDOA) silicon device ID
#define DWT_C0_DEV_ID       (0xDECA0302)        //!< DW3000 MPW C0 (non PDOA) silicon device ID
#define DWT_C0_PDOA_DEV_ID  (0xDECA0312)        //!< DW3000 MPW C0 (with PDOA) silicon device ID

//! fast commands
#define CMD_DB_TOGGLE     0x13   //!< Toggle double buffer pointer
#define CMD_CLR_IRQS      0x12   //!< Clear all events/clear interrupt
#define CMD_CCA_TX_W4R    0x11   //!< Check if channel clear prior to TX, enable RX when TX done
#define CMD_DTX_REF_W4R   0x10   //!< Start delayed TX (as DTX_REF below), enable RX when TX done
#define CMD_DTX_RS_W4R    0xF    //!< Start delayed TX (as DTX_RS below), enable RX when TX done
#define CMD_DTX_TS_W4R    0xE    //!< Start delayed TX (as DTX_TS below), enable RX when TX done
#define CMD_DTX_W4R       0xD    //!< Start delayed TX (as DTX below), enable RX when TX done
#define CMD_TX_W4R        0xC    //!< Start TX (as below), enable RX when TX done
#define CMD_CCA_TX        0xB    //!< Check if channel clear prior to TX
#define CMD_DRX_REF       0xA    //!< Enable RX @ time = DREF_TIME + DX_TIME
#define CMD_DTX_REF       0x9    //!< Start delayed TX (RMARKER will be @ time = DREF_TIME + DX_TIME)
#define CMD_DRX_RS        0x8    //!< Enable RX @ time = RX_TIME + DX_TIME
#define CMD_DTX_RS        0x7    //!< Start delayed TX (RMARKER will be @ time = RX_TIME + DX_TIME)
#define CMD_DRX_TS        0x6    //!< Enable RX @ time = TX_TIME + DX_TIME
#define CMD_DTX_TS        0x5    //!< Start delayed TX (RMARKER will be @ time = TX_TIME + DX_TIME)
#define CMD_DRX           0x4    //!< Enable RX @ time specified in DX_TIME register
#define CMD_DTX           0x3    //!< Start delayed TX (RMARKER will be @ time set in DX_TIME register)
#define CMD_RX            0x2    //!< Enable RX
#define CMD_TX            0x1    //!< Start TX
#define CMD_TXRXOFF       0x0    //!< Turn off TX or RX, clear any TX/RX events and put DW3000 into IDLE

//! constants for selecting the bit rate for data TX (and RX)
//! These are defined for write (with just a shift) the TX_FCTRL register
#define DWT_BR_850K     0   //!< UWB bit rate 850 kbits/s
#define DWT_BR_6M8      1   //!< UWB bit rate 6.8 Mbits/s
#define DWT_BR_NODATA   2   //!< No data (SP3 packet format)

//! constants for specifying the (Nominal) mean Pulse Repetition Frequency
//! These are defined for direct write (with a shift if necessary) to CHAN_CTRL and TX_FCTRL regs
#define DWT_PRF_16M     1   //!< UWB PRF 16 MHz
#define DWT_PRF_64M     2   //!< UWB PRF 64 MHz
#define DWT_PRF_SCP     3   //!< SCP UWB PRF ~100 MHz

// Defined constants for "mode" bitmask parameter passed into dwt_starttx() function.
#define DWT_START_TX_IMMEDIATE      0x00    //! Send the frame immediately
#define DWT_START_TX_DELAYED        0x01    //! Send the frame at specified time (time must be less that half period away)
#define DWT_RESPONSE_EXPECTED       0x02    //! Will enable the receiver after TX has completed

// Defined constants for "mode" bitmask parameter passed into dwt_rxenable() function.
#define DWT_START_RX_IMMEDIATE      0x00    //! Enable the receiver immediately
#define DWT_START_RX_DELAYED        0x01    //! Set up delayed RX, if "late" error triggers, then the RX will be enabled immediately
#define DWT_IDLE_ON_DLY_ERR         0x02    //! If delayed RX failed due to "late" error then if this

//DW3000 interrupt events
#define DWT_INT_TFRS            0x00000080          // frame sent
#define DWT_INT_LDED            0x00000400          // micro-code has finished execution
#define DWT_INT_RFCG            0x00004000          // frame received with good CRC
#define DWT_INT_RPHE            0x00001000          // receiver PHY header error
#define DWT_INT_RFCE            0x00008000          // receiver CRC error
#define DWT_INT_RFSL            0x00010000          // receiver sync loss error
#define DWT_INT_RFTO            0x00020000          // frame wait timeout
#define DWT_INT_RXPTO           0x00200000          // preamble detect timeout
#define DWT_INT_SFDT            0x04000000          // SFD timeout
#define DWT_INT_ARFE            0x20000000          // frame rejected (due to frame filtering configuration)

//! constants for specifying TX Preamble length in symbols
#define DWT_PLEN_4096   0x03    //! Standard preamble length 4096 symbols
#define DWT_PLEN_2048   0x0A    //! Non-standard preamble length 2048 symbols
#define DWT_PLEN_1536   0x06    //! Non-standard preamble length 1536 symbols
#define DWT_PLEN_1024   0x02    //! Standard preamble length 1024 symbols
#define DWT_PLEN_512    0x0d    //! Non-standard preamble length 512 symbols
#define DWT_PLEN_256    0x09    //! Non-standard preamble length 256 symbols
#define DWT_PLEN_128    0x05    //! Non-standard preamble length 128 symbols
#define DWT_PLEN_64     0x01    //! Standard preamble length 64 symbols

// Define DW3000 STS modes
#define DWT_STS_MODE_OFF         0x0     // STS is off
#define DWT_STS_MODE_1           0x1     // STS mode 1
#define DWT_STS_MODE_2           0x2     // STS mode 2

// Define DW3000 PDOA modes
#define DWT_PDOA_M0           0x0     // DW PDOA mode is off
#define DWT_PDOA_M1           0x1     // DW PDOA mode  mode 1
#define DWT_PDOA_M3           0x3     // DW PDOA mode  mode 3

//! constants for specifying SFD Types
#define DWT_SFD_IEEE_4A 0   //!< IEEE 8-bit ternary
#define DWT_SFD_DW_8    1   //!< DW 8-bit
#define DWT_SFD_DW_16   2   //!< DW 16-bit
#define DWT_SFD_IEEE_4Z 3   //!< IEEE 8-bit binary (4z)

// Constants for PAC size
#define DWT_PAC8        0   //!< PAC  8
#define DWT_PAC16       1   //!< PAC 16
#define DWT_PAC32       2   //!< PAC 32
#define DWT_PAC4        3   //!< PAC  4

// PHR mode constants
#define DWT_PHRMODE_STD         0x0     // standard PHR mode
#define DWT_PHRMODE_EXT         0x1     // DW proprietary extended frames PHR mode

// PHR rate constants
#define DWT_PHRRATE_STD         0x0     // standard PHR rate
#define DWT_PHRRATE_DTA         0x1     // PHR at data rate (6M81)

/*! ------------------------------------------------------------------------------------------------------------------
 * Structure typedef: dwt_config_t
 *
 * Structure for setting device configuration via dwt_configure() function
 *
 */
typedef struct
{
    uint8_t chan ;           //!< Channel number (5 or 9)
    uint8_t txPreambLength ; //!< DWT_PLEN_64..DWT_PLEN_4096
    uint8_t rxPAC ;          //!< Acquisition Chunk Size (Relates to RX preamble length)
    uint8_t txCode ;         //!< TX preamble code (the code configures the PRF, e.g. 9 -> PRF of 64 MHz)
    uint8_t rxCode ;         //!< RX preamble code (the code configures the PRF, e.g. 9 -> PRF of 64 MHz)
    uint8_t sfdType;         //!< SFD type (0 for short IEEE 8-bit standard, 1 for DW 8-bit, 2 for DW 16-bit, 3 for 4z BPRF)
    uint8_t dataRate ;       //!< Data rate {DWT_BR_850K or DWT_BR_6M8}
    uint8_t phrMode ;        //!< PHR mode {0x0 - standard DWT_PHRMODE_STD, 0x3 - extended frames DWT_PHRMODE_EXT}
    uint8_t phrRate;         //!< PHR rate {0x0 - standard DWT_PHRRATE_STD, 0x1 - at datarate DWT_PHRRATE_DTA}
    uint16_t sfdTO ;         //!< SFD timeout value (in symbols)
    uint8_t stsMode;         //!< STS mode (no STS, STS before PHR or STS after data)
    uint8_t stsLength;       //!< STS length
    uint8_t pdoaMode;        //!< PDOA mode
} dwt_config_t ;

typedef struct
{
    uint8_t   PGdly;
    uint32_t  power;
    uint16_t  PGcount;
} dwt_txconfig_t ;

// Call-back type for all interrupt events
typedef void (*dwt_cb_t)(void);

// TX/RX call-back data
typedef struct
{
    uint32_t status;      //initial value of register as ISR is entered
    uint16_t datalength;  //length of frame
    uint8_t  rx_flags;    //RX frame flags
} dwt_cb_data_t;

// Essential API Functions
int dwt_initialise(int mode);
int dwt_configure(dwt_config_t *config);
void dwt_configuretxrf(dwt_txconfig_t *config);

// Frame transmission/reception
int dwt_writetxdata(uint16_t txDataLength, uint8_t *txDataBytes, uint16_t txBufferOffset);
void dwt_writetxfctrl(uint16_t txFrameLength, uint16_t txBufferOffset, uint8_t ranging);
int dwt_starttx(uint8_t mode);
int dwt_rxenable(int mode);
void dwt_readrxdata(uint8_t *buffer, uint16_t length, uint16_t rxBufferOffset);

// Timestamp functions
void dwt_readtxtimestamp(uint8_t * timestamp);
void dwt_readrxtimestamp(uint8_t * timestamp);
uint32_t dwt_readtxtimestamphi32(void);
uint32_t dwt_readrxtimestamphi32(void);

// System control
void dwt_forcetrxoff(void);
void dwt_setdelayedtrxtime(uint32_t starttime);
uint32_t dwt_read32bitreg(uint32_t regFileID);
void dwt_write32bitreg(uint32_t regFileID, uint32_t regval);

// Interrupt and status
uint32_t dwt_read32bitoffsetreg(int regFileID, int regOffset);
void dwt_write32bitoffsetreg(int regFileID, int regOffset, uint32_t regval);
void dwt_writetodevice(uint32_t recordNumber, uint16_t index, uint16_t length, uint8_t *buffer);
void dwt_readfromdevice(uint32_t recordNumber, uint16_t index, uint16_t length, uint8_t *buffer);

// Configuration functions
void dwt_setrxantennadelay(uint16_t antennaDly);
void dwt_settxantennadelay(uint16_t antennaDly);
uint32_t dwt_readdevid(void);
void dwt_setinterrupt(uint32_t bitmask_lo, uint32_t bitmask_hi, uint8_t enable);

// Distance measurement
int16_t dwt_readclockoffset(void);

// Platform dependent functions (to be implemented)
extern int writetospi(uint16_t headerLength, const uint8_t *headerBuffer, uint16_t bodylength, const uint8_t *bodyBuffer);
extern int readfromspi(uint16_t headerLength, uint8_t *headerBuffer, uint16_t readlength, uint8_t *readBuffer);
extern void deca_sleep(unsigned int time_ms);
extern void deca_usleep(unsigned long time_us);

#ifdef __cplusplus
}
#endif

#endif /* _DECA_DEVICE_API_H_ */
