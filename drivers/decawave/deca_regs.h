/*! ---------------------------------------------------------------------------
 * @file    deca_regs.h
 * @brief   DW3000 Register Definitions (Essential subset)
 *
 * @author  Decawave Software
 * @attention
 * Copyright 2019 - 2020 (c) Decawave Ltd, Dublin, Ireland.
 * All rights reserved.
 */

#ifndef __DECA_REGS_H
#define __DECA_REGS_H                         1

#ifdef __cplusplus
extern "C" {
#endif

// Essential Register IDs
#define DEV_ID_ID                            0x0
#define EUI_64_LO_ID                         0x4
#define EUI_64_HI_ID                         0x8
#define PANADR_ID                            0xc
#define SYS_CFG_ID                           0x10
#define SYS_STATUS_ID                        0x44
#define SYS_ENABLE_LO_ID                     0x48
#define SYS_ENABLE_HI_ID                     0x4c
#define TX_FCTRL_ID                          0x68
#define TX_BUFFER_ID                         0x7C
#define DX_TIME_ID                           0x88
#define RX_BUFFER0_ID                        0xA0
#define RX_BUFFER1_ID                        0xA4
#define RX_FINFO_ID                          0xA8
#define RX_TIME_0_ID                         0xB0
#define TX_TIME_LO_ID                        0xB4
#define TX_TIME_HI_ID                        0xB8
#define TX_ANTD_ID                           0xBC
#define RX_ANTD_ID                           0xC0
#define CHAN_CTRL_ID                         0xC4

// SYS_STATUS bits
#define SYS_STATUS_CPLOCK_BIT_MASK           0x02
#define SYS_STATUS_TXFRS_BIT_MASK            0x80
#define SYS_STATUS_RXFCG_BIT_MASK            0x4000
#define SYS_STATUS_RXFCE_BIT_MASK            0x8000
#define SYS_STATUS_RXRFSL_BIT_MASK           0x10000
#define SYS_STATUS_RXRFTO_BIT_MASK           0x20000
#define SYS_STATUS_RXPHE_BIT_MASK            0x1000
#define SYS_STATUS_RXPTO_BIT_MASK            0x200000
#define SYS_STATUS_LDED_BIT_MASK             0x400

// TX_FCTRL bits
#define TX_FCTRL_TXBLEN_BIT_OFFSET           (0U)
#define TX_FCTRL_TXBLEN_BIT_LEN              (10U)
#define TX_FCTRL_TXBLEN_BIT_MASK             0x3ffU
#define TX_FCTRL_TXBOFFS_BIT_OFFSET          (10U)
#define TX_FCTRL_TXBOFFS_BIT_LEN             (10U)
#define TX_FCTRL_TXBOFFS_BIT_MASK            0xffc00U
#define TX_FCTRL_RANGING_BIT_OFFSET          (20U)
#define TX_FCTRL_RANGING_BIT_LEN             (1U)
#define TX_FCTRL_RANGING_BIT_MASK            0x100000U

// RX_FINFO bits
#define RX_FINFO_RXFLEN_BIT_OFFSET           (0U)
#define RX_FINFO_RXFLEN_BIT_LEN              (10U)
#define RX_FINFO_RXFLEN_BIT_MASK             0x3ffU
#define RXFLEN_MASK                          0x3ffU

// CHAN_CTRL bits
#define CHAN_CTRL_TX_CHAN_BIT_OFFSET         (0U)
#define CHAN_CTRL_TX_CHAN_BIT_LEN            (4U)
#define CHAN_CTRL_TX_CHAN_BIT_MASK           0xfU
#define CHAN_CTRL_RX_CHAN_BIT_OFFSET         (4U)
#define CHAN_CTRL_RX_CHAN_BIT_LEN            (4U)
#define CHAN_CTRL_RX_CHAN_BIT_MASK           0xf0U
#define CHAN_CTRL_TXPCODE_BIT_OFFSET         (8U)
#define CHAN_CTRL_TXPCODE_BIT_LEN            (5U)
#define CHAN_CTRL_TXPCODE_BIT_MASK           0x1f00U
#define CHAN_CTRL_RXPCODE_BIT_OFFSET         (13U)
#define CHAN_CTRL_RXPCODE_BIT_LEN            (5U)
#define CHAN_CTRL_RXPCODE_BIT_MASK           0x3e000U

// Additional essential registers
#define SYS_STATE_LO_ID                      0x50
#define SYS_STATE_HI_ID                      0x54
#define ACK_RESP_T_ID                        0x58
#define RX_SNIFF_ID                          0x5C
#define TX_POWER_ID                          0x60
#define TX_ANTD_ID                           0xBC
#define RX_ANTD_ID                           0xC0

#ifdef __cplusplus
}
#endif

#endif /* __DECA_REGS_H */
