/**
 * @file hds_twr.c
 * @brief High-Density Symmetric Two-Way Ranging Implementation
 *
 * This file implements the HDS-TWR (High-Density Symmetric Two-Way Ranging) protocol
 * for managing multiple device communications in a dense UWB network environment.
 * The protocol handles collision avoidance, timing synchronization, and coordinated
 * ranging operations between anchors and tags.
 *
 * @author UWB Positioning System Team
 * @date 2024
 */

#include "hds_twr.h"
#include "filter.h"
#include "twr.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Missing constant definitions */
#ifndef UWB_COMMU_DATA_MAXLEN
#define UWB_COMMU_DATA_MAXLEN 16
#endif

#ifndef FRAME_LEN_MAX
#define FRAME_LEN_MAX 0x3FF
#endif

/* Global Variables */
uint8_t SYS_ANC_RESP_FLAG = 0;                    // Anchor response transmission flag
uint8_t SYS_ANC_FINAL_FLAG = 0;                   // Anchor final reception flag

uint16_t Dis_cal = 0;                             // Calculated distance value
uint8_t frame_now;                                 // Current communication frame number
uint16_t Anc_recvFinal_timeflag = 0;              // Final reception timeout counter

/* Timing Configuration */
#define Anchor_delay_base  0x1E700                // Anchor delay base time: 0x3CE00 = 1ms, 0x1E700 = 500us

/* Communication Buffers */
u8 TX_ANC_INFORM_BUFF[TX_ANC_INFORM_LEN];         // Anchor inform transmission buffer
u8 TX_TAG_POLL_BUFF[TX_TAG_POLL_LEN];             // Tag poll transmission buffer
u8 TX_ANC_RESP_BUFF[TX_ANC_RESP_LEN];             // Anchor response transmission buffer
u8 TX_TAG_FINAL_BUFF[TX_TAG_FINAL_LEN];           // Tag final transmission buffer
u8 TX_ANC_REQ_BUFF[TX_ANC_REQ_LEN];               // Anchor request transmission buffer (primary anchor)
u8 TX_ANC_REPLY_BUFF[TX_ANC_REPLY_LEN];           // Anchor reply transmission buffer (secondary anchor)
u8 HDS_rx_buffer[RX_MAX_LEN];                     // Reception buffer

/**
 * @brief Send HDS-TWR Response Frame
 *
 * Implements the anchor response transmission in the HDS-TWR protocol.
 * The function handles delayed transmission timing based on anchor order
 * to prevent collisions in multi-anchor networks.
 *
 * @param A_ID Transmitting anchor ID (0xFF for broadcast)
 * @param B_ID Target tag ID
 * @param En Enable data payload transmission (1=enable, 0=disable)
 *
 * @return uint8_t Operation status:
 *         0 = Processing (not complete)
 *         1 = Success (transmission complete)
 *         2 = Failure (transmission error)
 */
uint8_t HDS_TWR_Send_Resp(uint8_t A_ID, uint8_t B_ID, uint8_t En)
{
	uint32_t delay_time;
	if(SYS_ANC_RESP_FLAG == 0)
	{
		int ret;
		uint8_t anc_order = 0;
		uint8_t send_len = TX_ANC_RESP_FIX_LEN;

		memset(TX_ANC_RESP_BUFF,0,sizeof(TX_ANC_RESP_BUFF));
		TX_ANC_RESP_BUFF[0] = A_ID;
		TX_ANC_RESP_BUFF[1] = B_ID;
		TX_ANC_RESP_BUFF[2] = frame_seq_nb;
		TX_ANC_RESP_BUFF[3] = 0xBC;
		TX_ANC_RESP_BUFF[4] = En;

		if(A_ID == 0xFF)
			anc_order = 1;
		else
			anc_order = 17 - (0xFF - A_ID);                  // Calculate order based on anchor ID for timing

		if(En == 1)
		{
			// Add communication data payload if available
			TX_ANC_RESP_BUFF[5] = 0; // Set to 0 if no communication helper available
			send_len += 0; // No additional data for now
		}

		delay_time = dwt_readsystimestamphi32();           // Get current system timestamp
		delay_time += Anchor_delay_base * anc_order;	   // Add order-based delay
		dwt_setdelayedtrxtime(delay_time);
		dwt_writetxdata(send_len, TX_ANC_RESP_BUFF, 0);     // Write data to DW1000 for delayed transmission
		dwt_writetxfctrl(send_len, 0, 1);                   // Configure transmission frame control
		dwt_setrxaftertxdelay(0);
		ret = dwt_starttx(DWT_START_TX_DELAYED);             // Start delayed transmission
		if(ret == DWT_SUCCESS)
			SYS_ANC_RESP_FLAG=1;
		else  // Transmission start failed
			return 2;
	}

	if(SYS_ANC_RESP_FLAG == 1)
	{
		if((dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
		{
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);   // Clear transmission flag
			SYS_ANC_RESP_FLAG = 0;
			return 1;
		}
	}

	return 0;
}

/**
 * @brief Receive HDS-TWR Final Frame and Calculate Distance
 *
 * Implements the anchor final frame reception and distance calculation
 * in the HDS-TWR protocol. Handles timeout management and timestamp
 * extraction for distance computation.
 *
 * @param A_ID Source anchor ID
 * @param B_ID Target tag ID
 * @param recv_timeout Reception timeout in 0.1ms units
 *
 * @return uint8_t Operation status:
 *         0 = Processing (not complete)
 *         1 = Success (distance calculated)
 *         2 = Failure (timeout or error)
 */
uint8_t HDS_TWR_Recv_FinalAndCal(uint8_t A_ID ,uint8_t B_ID, uint16_t recv_timeout)
{

	if(SYS_ANC_FINAL_FLAG == 0)
	{
		Anc_recvFinal_timeflag = 0;
		dwt_forcetrxoff();
		dwt_setrxtimeout(0);    // Set reception timeout, 0 means no timeout
		dwt_rxenable(0);
		SYS_ANC_FINAL_FLAG = 1;
	}

	if(Anc_recvFinal_timeflag > recv_timeout)
	{
		Anc_recvFinal_timeflag = 0;
		SYS_ANC_FINAL_FLAG = 0;
		dwt_forcetrxoff();
		return 2;
	}

	if(SYS_ANC_FINAL_FLAG == 1)
	{
		if((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)) // Check chip status for successful reception or error
		{
			SYS_ANC_FINAL_FLAG = 2;
		}
		else return 0;
	}

	if(SYS_ANC_FINAL_FLAG == 2)  // Successfully received or timeout
	{
		if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) // Successful reception
		{
			SYS_ANC_FINAL_FLAG = 3;
		}
		else
		{
			/* Clear RX error events in the DW1000 status register. */
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
			SYS_ANC_FINAL_FLAG=0;
		}
	}

	if(SYS_ANC_FINAL_FLAG == 3)
	{
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);            // Clear flag
		frame_len = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX;    		// Get received data length
		dwt_readrxdata(HDS_rx_buffer, frame_len, 0);                           // Read received data
		if(HDS_rx_buffer[0] == B_ID && HDS_rx_buffer[3] == 0xCD)
		{
			SYS_ANC_FINAL_FLAG = 4;
		}
		else if(HDS_rx_buffer[3] == 0xDE && HDS_rx_buffer[0] == 0xFF)
		{
			SYS_ANC_FINAL_FLAG = 0;
			Anc_recvFinal_timeflag = 0;
			return 2;
		}
		else
		{
			SYS_ANC_FINAL_FLAG = 0;
		}
	}

	if(SYS_ANC_FINAL_FLAG == 4)
	{
		uint8_t Anc_order = 0;
		// Processing
		if(A_ID != 0xFF)
			Anc_order = ANCHOR_LIST_COUNT - (0xFF - A_ID);

		Time_ts[2] = (uint32)get_tx_timestamp_u64();              // Get T2: Anchor sends Resp timestamp
		final_msg_get_ts(&HDS_rx_buffer[4 + Anc_order*4],&Time_ts[3]);  // Get T3: Tag receives Resp timestamp
		final_msg_get_ts(&HDS_rx_buffer[68],&Time_ts[0]);               // Get T0: Tag sends Poll timestamp
		final_msg_get_ts(&HDS_rx_buffer[72],&Time_ts[4]);               // Get T4: Tag sends Final timestamp (as current time)
		Time_ts[5] = (uint32)get_rx_timestamp_u64();              // Get T5: Anchor receives Final timestamp
		dwt_readdiagnostics(&rx_diag);                        // Read signal strength diagnostic information
		SYS_ANC_FINAL_FLAG = 5;
	}

	if(SYS_ANC_FINAL_FLAG == 5)                                 // Calculate distance
	{
		if(Twr_CalDist(B_ID,&Dis_cal) == -1)
		{
			// Processing failed
			SYS_ANC_FINAL_FLAG = 0;
			return 2;
		}
		else
		{
			SYS_ANC_FINAL_FLAG = 0;
			return 1;
		}

	}
	return 0;
}
uint8_t HDS_TWR_Recv_FinalAndCal(uint8_t A_ID ,uint8_t B_ID, uint16_t recv_timeout)
{

	if(SYS_ANC_FINAL_FLAG == 0)
	{
		Anc_recvFinal_timeflag = 0;
		dwt_forcetrxoff();
		dwt_setrxtimeout(0);
		dwt_rxenable(0);
		SYS_ANC_FINAL_FLAG = 1;
	}

	if(Anc_recvFinal_timeflag > recv_timeout)
	{
		Anc_recvFinal_timeflag = 0;
		SYS_ANC_FINAL_FLAG = 0;
//		dwt_rxreset();
		dwt_forcetrxoff();
		return 2;
	}

	if(SYS_ANC_FINAL_FLAG == 1)
	{
		if((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR))
		{
			SYS_ANC_FINAL_FLAG = 2;
		}
		else return 0;
	}

	if(SYS_ANC_FINAL_FLAG == 2)
	{
		if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
		{
			SYS_ANC_FINAL_FLAG = 3;
		}
		else
		{
			/* Clear RX error events in the DW1000 status register. */
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
			SYS_ANC_FINAL_FLAG=0;
//				if(status_reg & SYS_STATUS_ALL_RX_TO)
//				  return 2;
//			  else
//          return 0;
		}
	}

	if(SYS_ANC_FINAL_FLAG == 3)
	{
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
		frame_len = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX;
		dwt_readrxdata(HDS_rx_buffer, frame_len, 0);
		if(HDS_rx_buffer[0] == B_ID && HDS_rx_buffer[3] == 0xCD)
		{
			SYS_ANC_FINAL_FLAG = 4;
		}
		else if(HDS_rx_buffer[3] == 0xDE && HDS_rx_buffer[0] == 0xFF)
		{
			SYS_ANC_FINAL_FLAG = 0;
			Anc_recvFinal_timeflag = 0;
			return 2;
		}
		else
		{
			SYS_ANC_FINAL_FLAG = 0;
		}
	}

	if(SYS_ANC_FINAL_FLAG == 4)
	{
		uint8_t Anc_order = 0;
		uint8_t i;
		// Processing
		if(A_ID != 0xFF)
			Anc_order = ANCHOR_LIST_COUNT - (0xFF - A_ID);

		Time_ts[2] = (uint32)get_tx_timestamp_u64();
		final_msg_get_ts(&HDS_rx_buffer[4 + Anc_order*4],&Time_ts[3]);
		final_msg_get_ts(&HDS_rx_buffer[68],&Time_ts[0]);
		final_msg_get_ts(&HDS_rx_buffer[72],&Time_ts[4]);
		Time_ts[5] = (uint32)get_rx_timestamp_u64();
		dwt_readdiagnostics(&rx_diag);
		SYS_ANC_FINAL_FLAG = 5;
	}

	if(SYS_ANC_FINAL_FLAG == 5)
	{
		if(Twr_CalDist(B_ID,&Dis_cal) == -1)
		{
			// Processing
			SYS_ANC_FINAL_FLAG = 0;
			return 2;
		}
		else
		{
			SYS_ANC_FINAL_FLAG = 0;
			return 1;
		}

	}
	return 0;
}

