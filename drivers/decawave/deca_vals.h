///*! ------------------------------------------------------------------------------------------------------------------
// * @file    deca_vals.h
// * @brief   DW3000 Register Definitions (Essential values)
// *
// * @attention
// *
// * Copyright 2013-2020 (c) Decawave Ltd, Dublin, Ireland.
// *
// * All rights reserved.
// *
// */

#ifndef _DECA_VALS_H_
#define _DECA_VALS_H_

#ifdef __cplusplus
extern "C" {
#endif

#define FCS_LEN                 (2)
#define RX_BUFFER_0_ID          0x120000            /* Default Receive Data Buffer */
#define RX_BUFFER_1_ID          0x130000            /* 2nd Receive Data Buffer */

#define RF_TXCTRL_CH5           0x1C071134UL
#define RF_TXCTRL_CH9           0x1C010034UL
#define RF_RXCTRL_CH9           0x08B5A833UL
#define RF_PLL_CFG_CH5          0x1F3C
#define RF_PLL_CFG_CH9          0x0F3C
#define RF_PLL_CFG_LD           0x81

#define XTAL_TRIM_BIT_MASK      0x3F           /* max allowed value for XTAL trim */

#define DWT_DGC_CFG             0x32
#define DWT_DGC_CFG0            0x10000240
#define DWT_DGC_CFG1            0x1b6da489

#define DWT_AUTO_CLKS          (0x200 | 0x200000 | 0x100000)  //default CLK_CTRL register value

#define ACC_BUFFER_MAX_LEN      (12288 + 1)         /* +1 is needed for "dummy" read */

#ifdef __cplusplus
}
#endif

#endif /* _DECA_VALS_H_ */
