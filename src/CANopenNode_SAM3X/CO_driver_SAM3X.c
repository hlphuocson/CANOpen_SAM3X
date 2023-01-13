/*
* CAN module object for SAM3X (FD)CAN peripheral IP.
*
* This file is a template for other microcontrollers.
*
* @file        CO_driver.c
* @ingroup     CO_driver
* @author      Hamed Jafarzadeh 	2022
* 				Tilen Marjerle		2021
* 				Janez Paternoster	2020
* @copyright   2004 - 2020 Janez Paternoster
*
* This file is part of CANopenNode, an opensource CANopen Stack.
* Project home page is <https://github.com/CANopenNode/CANopenNode>.
* For more information on CANopen see <http://www.can-cia.org/>.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
* Implementation Author:               Tilen Majerle <tilen@majerle.eu>
*/
#include "CANopenNode_SAM3X\CO_driver_target.h"
#include "CO_app_SAM3X.h"
// #include "asf.h"
#include "CANopenNode\301\CO_SDOserver.h"
//#include "htc.h"



/* Local CAN module object */
static CO_CANmodule_t* CANModule_local = NULL; /* Local instance of global CAN module */

/* CAN masks for identifiers */
#define CANID_MASK 0x07FF /*!< CAN standard ID mask */
#define FLAG_RTR   0x8000 /*!< RTR flag, part of identifier */
#define CO_UNUSED(v)  (void)(v)
#define CANMB_TX      (CANMB_NUMBER - 1)
#define sysclk_get_cpu_hz() 84000000
#if defined(__cplusplus)
extern "C" {
#endif

void reset_mailbox_conf(can_mb_conf_t *p_mailbox)
{
	p_mailbox->ul_mb_idx = 0;
	p_mailbox->uc_obj_type = 0;
	p_mailbox->uc_id_ver = 0;
	p_mailbox->uc_length = 0;
	p_mailbox->uc_tx_prio = 0;
	p_mailbox->ul_status = 0;
	p_mailbox->ul_id_msk = 0;
	p_mailbox->ul_id = 0;
	p_mailbox->ul_fid = 0;
	p_mailbox->ul_datal = 0;
	p_mailbox->ul_datah = 0;
}

/******************************************************************************/
void CO_CANsetConfigurationMode(void* CANptr) {
	/* Put CAN module in configuration mode */
	//     if (CANptr != NULL) {
	// #ifdef CO_SAM3X_FDCAN_Driver
	//         HAL_FDCAN_Stop(((CANopenNodeSAM3X*)CANptr)->CANHandle);
	// #else
	//         HAL_CAN_Stop(((CANopenNodeSAM3X*)CANptr)->CANHandle);
	// #endif
	//     }
	CO_UNUSED(CANptr);
}

/******************************************************************************/
void
CO_CANsetNormalMode(CO_CANmodule_t *CANmodule) {
	/* Put CAN module in normal mode */
	//     if (CANmodule->CANptr != NULL) {
	// #ifdef CO_SAM3X_FDCAN_Driver
	//         if (HAL_FDCAN_Start(((CANopenNodeSAM3X*)CANmodule->CANptr)->CANHandle) == HAL_OK)
	// #else
	//         if (HAL_CAN_Start(((CANopenNodeSAM3X*)CANmodule->CANptr)->CANHandle) == HAL_OK)
	// #endif
	//         {
	//             CANmodule->CANnormal = true;
	//         }
	//     }
	CO_UNUSED(CANmodule->CANptr);

	CANmodule->CANnormal = true;
}


/******************************************************************************/
CO_ReturnError_t CO_CANmodule_init(
CO_CANmodule_t         *CANmodule,
void* CANptr,
CO_CANrx_t              rxArray[],
uint16_t                rxSize,
CO_CANtx_t              txArray[],
uint16_t                txSize,
uint16_t                CANbitRate)
{
	uint16_t i;
	uint32_t ul_sysclk;
	/* Keep a local copy of CANModule */
    CANModule_local = CANmodule;

	/* Configure object variables */
	CANmodule->CANptr = ((CANopenNodeSAM3X*)CANptr)->CANHandle;//CANbaseAddress;
	CANmodule->rxArray = rxArray;
	CANmodule->rxSize = rxSize;
	CANmodule->txArray = txArray;
	CANmodule->txSize = txSize;
	CANmodule->CANerrorStatus = 0;
	CANmodule->CANnormal = false;
	CANmodule->useCANrxFilters = false;
	CANmodule->bufferInhibitFlag = false;
	CANmodule->firstCANtxMessage = true;
	CANmodule->CANtxCount = 0U;
	CANmodule->errOld = 0U;
	CANmodule->em = NULL;

	for(i=0U; i<rxSize; i++)
	{
		rxArray[i].ident = 0U;
		rxArray[i].mask = 0xFFFFU;
        rxArray[i].object = NULL;
		rxArray[i].CANrx_callback = NULL;
	}
	for(i=0U; i<txSize; i++)
	{
		txArray[i].bufferFull = false;
	}

	/* Configure CAN module registers */
	if (CANmodule->CANptr == CAN0)
	{
		/* Configure CAN timing */

		/* Enable CAN0 & CAN0 clock. */
		pmc_enable_periph_clk(ID_CAN0);

		ul_sysclk = sysclk_get_cpu_hz();

		if (can_init(CAN0, ul_sysclk, CANbitRate))
		{
			printf("CAN0 initialization is completed\n\r");

			/* Disable all CAN0 interrupts */
			can_disable_interrupt(CAN0, CAN_DISABLE_ALL_INTERRUPT_MASK);

			
		}

		can_reset_all_mailbox(CANmodule->CANptr);
		
		/* Init CAN0 mailbox 0-6 as reception mailboxes */
		for (uint8_t i = 0; i <= (CANMB_NUMBER-2); i++)
		{
			reset_mailbox_conf(&CANmodule->rxMbConf[i]);
			CANmodule->rxMbConf[i].ul_mb_idx = i;
			CANmodule->rxMbConf[i].uc_obj_type = CAN_MB_RX_MODE;

			if (i == (CANMB_NUMBER-2)) CANmodule->rxMbConf[i].uc_obj_type = CAN_MB_RX_OVER_WR_MODE;
			CANmodule->rxMbConf[i].ul_datah=0;
			CANmodule->rxMbConf[i].ul_datal=0;
			CANmodule->rxMbConf[i].uc_length=0;
			can_mailbox_init(CANmodule->CANptr, &CANmodule->rxMbConf[i]);
			/* Enable CAN0 mailbox number i interrupt. */
			can_enable_interrupt(CANmodule->CANptr, (0x01u << ((uint8_t)i)));
		}
		/* Configure and enable interrupt of CAN0 */
		NVIC_EnableIRQ(CAN0_IRQn);
		
		/* Init last CAN0 mailbox, number 7, as transmit mailbox */
		reset_mailbox_conf(&CANmodule->txMbConf);
		CANmodule->txMbConf.ul_mb_idx = (CANMB_TX);
		CANmodule->txMbConf.uc_obj_type = CAN_MB_TX_MODE;
		CANmodule->txMbConf.uc_tx_prio = 14;
		CANmodule->txMbConf.uc_id_ver = 0;
		CANmodule->txMbConf.ul_id_msk = 0;
		can_mailbox_init(CANmodule->CANptr, &CANmodule->txMbConf);
		/* CAN module filters are not used, all messages with standard 11-bit */
		/* identifier will be received */
		/* Configure mask 0 so that all messages with standard identifier are accepted */
		
	}

	if (CANmodule->CANptr == CAN1)
	{
	/* CAN1 Transceiver */
	//static sn65hvd234_ctrl_t can1_transceiver;
	
	/* Initialize CAN1 Transceiver */
	//sn65hvd234_set_rs(&can1_transceiver, PIN_CAN1_TR_RS_IDX);
	//sn65hvd234_set_en(&can1_transceiver, PIN_CAN1_TR_EN_IDX);
	//
	///* Enable CAN1 Transceiver */
	//sn65hvd234_disable_low_power(&can1_transceiver); //Low power mode == listening only mode
	//sn65hvd234_enable(&can1_transceiver);
	
	/* Configure CAN timing */
	
	/* Enable CAN1 & CAN1 clock */
	pmc_enable_periph_clk(ID_CAN1);
	
	ul_sysclk = sysclk_get_cpu_hz();
	
	if (can_init(CAN1, ul_sysclk, CANbitRate))
	{
	printf("CAN1 initialization is completed\n\r");
	
	/* Disable all CAN1 interrupts */
	can_disable_interrupt(CAN1, CAN_DISABLE_ALL_INTERRUPT_MASK);
	
	/* Configure and enable interrupt of CAN1 */
	NVIC_EnableIRQ(CAN1_IRQn);
	}
	
	can_reset_all_mailbox(CANmodule->CANptr);
	
	/* CAN module filters are not used, all messages with standard 11-bit */
	/* identifier will be received */
	/* Configure mask 0 so that all messages with standard identifier are accepted */
	
	/* Init CAN1 mailbox 0-6 as reception mailboxes */
	
	for (i = 0; i <= (CANMB_NUMBER-2); i++)
	{
	reset_mailbox_conf(&CANmodule->rxMbConf[i]);
	CANmodule->rxMbConf[i].ul_mb_idx = i;
	CANmodule->rxMbConf[i].uc_obj_type = CAN_MB_RX_MODE;
	
	if (i == (CANMB_NUMBER-2))
	CANmodule->rxMbConf[i].uc_obj_type = CAN_MB_RX_OVER_WR_MODE;
	
	/* Standard mode only, not extended mode */
	CANmodule->rxMbConf[i].ul_id_msk = CAN_MAM_MIDvA_Msk;
	CANmodule->rxMbConf[i].ul_id = CAN_MID_MIDvA(0);
	can_mailbox_init(CANmodule->CANptr, &CANmodule->rxMbConf[i]);
	
	/* Enable CAN1 mailbox number i interrupt. */
	can_enable_interrupt(CANmodule->CANptr, (0x1u << i));
	}
	/* Configure and enable interrupt of CAN0 */
		NVIC_EnableIRQ(CAN0_IRQn);
	
	/* Init last CAN1 mailbox, number 7, as transmit mailbox */
	reset_mailbox_conf(&CANmodule->txMbConf);
	CANmodule->txMbConf.ul_mb_idx = (CANMB_TX);
	CANmodule->txMbConf.uc_obj_type = CAN_MB_TX_MODE;
	CANmodule->txMbConf.uc_tx_prio = 14;
	CANmodule->txMbConf.uc_id_ver = 0;
	CANmodule->txMbConf.ul_id_msk = 0;
	can_mailbox_init(CANmodule->CANptr, &CANmodule->txMbConf);
	}

	return CO_ERROR_NO;
}

/******************************************************************************/
void
CO_CANmodule_disable(CO_CANmodule_t* CANmodule) {
	//     if (CANmodule != NULL && CANmodule->CANptr != NULL) {
	// #ifdef CO_SAM3X_FDCAN_Driver
	//         HAL_FDCAN_Stop(((CANopenNodeSAM3X*)CANmodule->CANptr)->CANHandle);

	// #else
	//         HAL_CAN_Stop(((CANopenNodeSAM3X*)CANmodule->CANptr)->CANHandle);
	// #endif
	//     }
	can_disable(CANmodule->CANptr);
	
}

/******************************************************************************/
CO_ReturnError_t CO_CANrxBufferInit(
CO_CANmodule_t         *CANmodule,
uint16_t                index,
uint16_t                ident,
uint16_t                mask,
bool_t               rtr,
void                   *object,
void                  (*CANrx_callback)(void* object, void* message))
{
	CO_ReturnError_t ret = CO_ERROR_NO;

	if (CANmodule != NULL && object != NULL && CANrx_callback != NULL && index < CANmodule->rxSize) {

		/* Buffer, which will be configured */
		CO_CANrx_t *buffer = &CANmodule->rxArray[index];

		/* Configure object variables */
		buffer->object = object;
		buffer->CANrx_callback = CANrx_callback;

		/* CAN identifier and CAN mask, bit aligned with CAN module. Different on different microcontrollers. */
		// buffer->ident = ident & 0x07FFU;
		// if(rtr){
		// 	buffer->ident |= 0x0800U;
		// }
		// buffer->mask = (mask & 0x07FFU) | 0x0800U;
		/*
         * Configure global identifier, including RTR bit
         *
         * This is later used for RX operation match case
         */
        buffer->ident = (ident & CANID_MASK) | (rtr ? FLAG_RTR : 0x00);
        buffer->mask = (mask & CANID_MASK) | FLAG_RTR;
		/* Set CAN hardware module filter and mask. */
		if(CANmodule->useCANrxFilters)
		{
		}
	}
	else
	{
		ret = CO_ERROR_ILLEGAL_ARGUMENT;
	}

	return ret;
}

/******************************************************************************/
//CO_CANtx_t*
// CO_CANtxBufferInit(CO_CANmodule_t* CANmodule, uint16_t index, uint16_t ident, bool_t rtr, uint8_t noOfBytes,
//                    bool_t syncFlag) {
//     CO_CANtx_t* buffer = NULL;

//     if (CANmodule != NULL && index < CANmodule->txSize) {
//         buffer = &CANmodule->txArray[index];

//         /* CAN identifier, DLC and rtr, bit aligned with CAN module transmit buffer */
//         buffer->ident = ((uint32_t)ident & CANID_MASK) | ((uint32_t)(rtr ? FLAG_RTR : 0x00));
//         buffer->DLC = noOfBytes;
//         buffer->bufferFull = false;
//         buffer->syncFlag = syncFlag;
//     }
//     return buffer;
// }

CO_CANtx_t *CO_CANtxBufferInit(
CO_CANmodule_t         *CANmodule,
uint16_t                index,
uint16_t                ident,
bool_t               rtr,
uint8_t                 noOfBytes,
bool_t               syncFlag)
{
	CO_CANtx_t *buffer = NULL;

	if((CANmodule != NULL) && (index < CANmodule->txSize))
	{
		/* Get specific buffer */
		buffer = &CANmodule->txArray[index];

		/* CAN identifier, DLC and rtr, bit aligned with CAN module transmit buffer. */
		buffer->ident = ident;
		buffer->rtr = rtr;

		buffer->bufferFull = false;
		buffer->syncFlag = syncFlag;
		buffer->DLC = noOfBytes;
	}

	return buffer;
}

/**
* \brief           Send CAN message to network
* This function must be called with atomic access.
*
* \param[in]       CANmodule: CAN module instance
* \param[in]       buffer: Pointer to buffer to transmit
*/
static uint8_t
prv_send_can_message(CO_CANmodule_t* CANmodule, CO_CANtx_t* buffer) {

	uint8_t success = 0;

	/* Check if TX FIFO is ready to accept more messages */
	#ifdef CO_SAM3X_FDCAN_Driver
	//static FDCAN_TxHeaderTypeDef tx_hdr;
	//if (HAL_FDCAN_GetTxFifoFreeLevel(((CANopenNodeSAM3X*)CANmodule->CANptr)->CANHandle) > 0) {
	///*
	//* RTR flag is part of identifier value
	//* hence it needs to be properly decoded
	//*/
	//tx_hdr.Identifier = buffer->ident & CANID_MASK;
	//tx_hdr.TxFrameType = (buffer->ident & FLAG_RTR) ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
	//tx_hdr.IdType = FDCAN_STANDARD_ID;
	//tx_hdr.FDFormat = FDCAN_CLASSIC_CAN;
	//tx_hdr.BitRateSwitch = FDCAN_BRS_OFF;
	//tx_hdr.MessageMarker = 0;
	//tx_hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	//tx_hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	//
	//switch (buffer->DLC) {
	//case 0:
	//tx_hdr.DataLength = FDCAN_DLC_BYTES_0;
	//break;
	//case 1:
	//tx_hdr.DataLength = FDCAN_DLC_BYTES_1;
	//break;
	//case 2:
	//tx_hdr.DataLength = FDCAN_DLC_BYTES_2;
	//break;
	//case 3:
	//tx_hdr.DataLength = FDCAN_DLC_BYTES_3;
	//break;
	//case 4:
	//tx_hdr.DataLength = FDCAN_DLC_BYTES_4;
	//break;
	//case 5:
	//tx_hdr.DataLength = FDCAN_DLC_BYTES_5;
	//break;
	//case 6:
	//tx_hdr.DataLength = FDCAN_DLC_BYTES_6;
	//break;
	//case 7:
	//tx_hdr.DataLength = FDCAN_DLC_BYTES_7;
	//break;
	//case 8:
	//tx_hdr.DataLength = FDCAN_DLC_BYTES_8;
	//break;
	//default: /* Hard error... */
	//break;
	//}
	//
	///* Now add message to FIFO. Should not fail */
	//success =
	//HAL_FDCAN_AddMessageToTxFifoQ(((CANopenNodeSAM3X*)CANmodule->CANptr)->CANHandle, &tx_hdr, buffer->data)
	//== HAL_OK;
}
#else
//static CAN_TxHeaderTypeDef tx_hdr;
///* Check if TX FIFO is ready to accept more messages */
//if (HAL_CAN_GetTxMailboxesFreeLevel(((CANopenNodeSAM3X*)CANmodule->CANptr)->CANHandle) > 0) {
///*
//* RTR flag is part of identifier value
//* hence it needs to be properly decoded
//*/
//tx_hdr.ExtId = 0u;
//tx_hdr.IDE = CAN_ID_STD;
//tx_hdr.DLC = buffer->DLC;
//tx_hdr.StdId = buffer->ident & CANID_MASK;
//tx_hdr.RTR = (buffer->ident & FLAG_RTR) ? CAN_RTR_REMOTE : CAN_RTR_DATA;
//
//uint32_t TxMailboxNum; // Transmission MailBox number
//
///* Now add message to FIFO. Should not fail */
//success = HAL_CAN_AddTxMessage(((CANopenNodeSAM3X*)CANmodule->CANptr)->CANHandle, &tx_hdr, buffer->data,
//&TxMailboxNum)
//== HAL_OK;
//}
#endif
return success;
}

/******************************************************************************/
// CO_ReturnError_t
// CO_CANsend(CO_CANmodule_t* CANmodule, CO_CANtx_t* buffer) {
//     CO_ReturnError_t err = CO_ERROR_NO;

//     /* Verify overflow */
//     if (buffer->bufferFull) {
//         if (!CANmodule->firstCANtxMessage) {
//             /* don't set error, if bootup message is still on buffers */
//             CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
//         }
//         err = CO_ERROR_TX_OVERFLOW;
//     }

//     /*
//      * Send message to CAN network
//      *
//      * Lock interrupts for atomic operation
//      */
//     CO_LOCK_CAN_SEND(CANmodule);
//     if (prv_send_can_message(CANmodule, buffer)) {
//         CANmodule->bufferInhibitFlag = buffer->syncFlag;
//     } else {
//         buffer->bufferFull = true;
//         CANmodule->CANtxCount++;
//     }
//     CO_UNLOCK_CAN_SEND(CANmodule);

//     return err;
// }

CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer)
{
	CO_ReturnError_t err = CO_ERROR_NO;

	/* Verify overflow */
	if(buffer->bufferFull){
		if(!CANmodule->firstCANtxMessage){
			/* Don't set error, if bootup message is still on buffers */
			CO_errorReport((CO_EM_t*)CANmodule->em, CO_EM_CAN_TX_OVERFLOW, CO_EMC_CAN_OVERRUN, buffer->ident);
		}
		err = CO_ERROR_TX_OVERFLOW;
	}

	CO_DISABLE_INTERRUPTS();

	/* If CAN TX buffer is free, copy message to it */
	if (((can_mailbox_get_status(CANmodule->CANptr, CANMB_TX) & CAN_MSR_MRDY) == CAN_MSR_MRDY) && (CANmodule->CANtxCount == 0))
	{
		CANmodule->bufferInhibitFlag = buffer->syncFlag;

		/* Copy message and txRequest */
		CANmodule->txMbConf.ul_id = CAN_MID_MIDvA(buffer->ident);
		CANmodule->txMbConf.ul_datal = *((uint32_t *) &(buffer->data[0]));
		CANmodule->txMbConf.ul_datah = *((uint32_t *) &(buffer->data[4]));
		CANmodule->txMbConf.uc_length = buffer->DLC;

		if (buffer->rtr)
		can_mailbox_tx_remote_frame(CANmodule->CANptr, &CANmodule->txMbConf);
		else
		can_mailbox_write(CANmodule->CANptr, &CANmodule->txMbConf);

		can_global_send_transfer_cmd(CANmodule->CANptr, 1 << CANMB_TX);
	}
	else /* If no buffer is free, message will be sent by interrupt */
	{
		buffer->bufferFull = true;
		CANmodule->CANtxCount++;
	}
	can_enable_interrupt(CANmodule->CANptr, 0x1u << CANMB_TX);
	CO_ENABLE_INTERRUPTS();

	return err;
}

/******************************************************************************/
// void
// CO_CANclearPendingSyncPDOs(CO_CANmodule_t* CANmodule) {
//     uint32_t tpdoDeleted = 0U;

//     CO_LOCK_CAN_SEND(CANmodule);
//     /* Abort message from CAN module, if there is synchronous TPDO.
//      * Take special care with this functionality. */
//     if (/*messageIsOnCanBuffer && */ CANmodule->bufferInhibitFlag) {
//         /* clear TXREQ */
//         CANmodule->bufferInhibitFlag = false;
//         tpdoDeleted = 1U;
//     }
//     /* delete also pending synchronous TPDOs in TX buffers */
//     if (CANmodule->CANtxCount > 0) {
//         for (uint16_t i = CANmodule->txSize; i > 0U; --i) {
//             if (CANmodule->txArray[i].bufferFull) {
//                 if (CANmodule->txArray[i].syncFlag) {
//                     CANmodule->txArray[i].bufferFull = false;
//                     CANmodule->CANtxCount--;
//                     tpdoDeleted = 2U;
//                 }
//             }
//         }
//     }
//     CO_UNLOCK_CAN_SEND(CANmodule);
//     if (tpdoDeleted) {
//         CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
//     }
// }

void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule)
{
	uint32_t tpdoDeleted = 0U;

	CO_DISABLE_INTERRUPTS();
	/* Abort message from CAN module, if there is synchronous TPDO.
	* Take special care with this functionality. */
	if(/*messageIsOnCanBuffer && */CANmodule->bufferInhibitFlag){
		/* clear TXREQ */
		CANmodule->bufferInhibitFlag = false;
		tpdoDeleted = 1U;
	}
	/* delete also pending synchronous TPDOs in TX buffers */
	// if(CANmodule->CANtxCount != 0U){
	// 	uint16_t i;
	// 	CO_CANtx_t *buffer = &CANmodule->txArray[0];
	// 	for(i = CANmodule->txSize; i > 0U; i--){
	// 		if(buffer->bufferFull){
	// 			if(buffer->syncFlag){
	// 				buffer->bufferFull = false;
	// 				CANmodule->CANtxCount--;
	// 				tpdoDeleted = 2U;
	// 			}
	// 		}
	// 		buffer++;
	// 	}
	// }
	if (CANmodule->CANtxCount > 0) {
        for (uint16_t i = CANmodule->txSize; i > 0U; --i) {
            if (CANmodule->txArray[i].bufferFull) {
                if (CANmodule->txArray[i].syncFlag) {
                    CANmodule->txArray[i].bufferFull = false;
                    CANmodule->CANtxCount--;
                    tpdoDeleted = 2U;
                }
            }
        }
    }
	CO_ENABLE_INTERRUPTS();

	if(tpdoDeleted != 0U){
		CO_errorReport((CO_EM_t*)CANmodule->em, CO_EM_TPDO_OUTSIDE_WINDOW, CO_EMC_COMMUNICATION, tpdoDeleted);
	}
}

/******************************************************************************/
void CO_CANverifyErrors(CO_CANmodule_t *CANmodule){
	uint16_t rxErrors = 0, txErrors = 0, overflow = 0;
	CO_EM_t* em = (CO_EM_t*)CANmodule->em;
	uint32_t err;

	/* get error counters from module. Id possible, function may use different way to
	* determine errors. */

	rxErrors = can_get_rx_error_cnt(CANmodule->CANptr);

	txErrors = can_get_tx_error_cnt(CANmodule->CANptr);

	//overflow = CANmodule->txSize;

	err = ((uint32_t)txErrors << 16) | ((uint32_t)rxErrors << 8) | overflow;

	if(CANmodule->errOld != err)
	{
		uint16_t status = CANmodule->CANerrorStatus;
		CANmodule->errOld = err;

		if(txErrors >= 256U)   /* bus off */
		{
			CO_errorReport(em, CO_EM_CAN_TX_BUS_OFF, CO_EMC_BUS_OFF_RECOVERED, err);
			status |= CO_CAN_ERRTX_BUS_OFF;
			// In this driver, we assume that auto bus recovery is activated ! so this error will eventually handled automatically.
		}
		else  /* not bus off */
		{
			CO_errorReset(em, CO_EM_CAN_TX_BUS_OFF, err);
			/* recalculate CANerrorStatus, first clear some flags */
			status &= 0xFFFF
			^ (CO_CAN_ERRTX_BUS_OFF | CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_WARNING
			| CO_CAN_ERRTX_PASSIVE);
			if((rxErrors >= 96U) || (txErrors >= 96U))
			{     /* bus warning */
				CO_errorReport(em, CO_EM_CAN_BUS_WARNING, CO_EMC_NO_ERROR, err);
				status |= CO_CAN_ERRRX_WARNING | CO_CAN_ERRTX_WARNING;
			}

			if(rxErrors >= 128U) /* RX bus passive */
			{
				CO_errorReport(em, CO_EM_CAN_RX_BUS_PASSIVE, CO_EMC_CAN_PASSIVE, err);
				status |= CO_CAN_ERRRX_PASSIVE;
			}
			else
			{
				CO_errorReset(em, CO_EM_CAN_RX_BUS_PASSIVE, err);
				status &= 0xffff
				^ (CO_CAN_ERRRX_PASSIVE);
			}

			if(txErrors >= 128U) /* TX bus passive */
			{
				if(!CANmodule->firstCANtxMessage)
				{
					CO_errorReport(em, CO_EM_CAN_TX_BUS_PASSIVE, CO_EMC_CAN_PASSIVE, err);
					status |= CO_CAN_ERRTX_PASSIVE;
				}
			}
			else
			{
				bool_t isError = CO_isError(em, CO_EM_CAN_TX_BUS_PASSIVE);
				if(isError)
				{
					CO_errorReset(em, CO_EM_CAN_TX_BUS_PASSIVE, err);
					CO_errorReset(em, CO_EM_CAN_TX_OVERFLOW, err);
					status &= 0xFFFF
					^ (CO_CAN_ERRTX_PASSIVE | CO_CAN_ERRTX_OVERFLOW);
				}
			}

			if((rxErrors < 96U) && (txErrors < 96U)) /* no error */
			{
				CO_errorReset(em, CO_EM_CAN_BUS_WARNING, err);
				status &= 0xFFFF
				^ (CO_CAN_ERRTX_BUS_OFF | CO_CAN_ERRRX_WARNING | CO_CAN_ERRTX_WARNING);
			}
		}

		if(overflow != 0U) /* CAN RX bus overflow */
		{
			CO_errorReport(em, CO_EM_CAN_RXB_OVERFLOW, CO_EMC_CAN_OVERRUN, err);
			status |= CO_CAN_ERRRX_OVERFLOW;
		}
		CANmodule->CANerrorStatus = status;
	}
}

/******************************************************************************/
/* Get error counters from the module. If necessary, function may use
* different way to determine errors. */
static uint16_t rxErrors = 0, txErrors = 0, overflow = 0;

void
CO_CANmodule_process(CO_CANmodule_t* CANmodule) {
	// uint32_t err = 0;

	// CANOpen just care about Bus_off, Warning, Passive and Overflow
	// I didn't find overflow error register in SAM3X, if you find it please let me know

	// #ifdef CO_SAM3X_FDCAN_Driver

	//     err = ((FDCAN_HandleTypeDef*)((CANopenNodeSAM3X*)CANmodule->CANptr)->CANHandle)->Instance->PSR
	//           & (FDCAN_PSR_BO | FDCAN_PSR_EW | FDCAN_PSR_EP);

	//     if (CANmodule->errOld != err) {

	//         uint16_t status = CANmodule->CANerrorStatus;

	//         CANmodule->errOld = err;

	//         if (err & FDCAN_PSR_BO) {
	//             status |= CO_CAN_ERRTX_BUS_OFF;
	//             // In this driver we expect that the controller is automatically handling the protocol exceptions.

	//         } else {
	//             /* recalculate CANerrorStatus, first clear some flags */
	//             status &= 0xFFFF
	//                       ^ (CO_CAN_ERRTX_BUS_OFF | CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_WARNING
	//                          | CO_CAN_ERRTX_PASSIVE);

	//             if (err & FDCAN_PSR_EW) {
	//                 status |= CO_CAN_ERRRX_WARNING | CO_CAN_ERRTX_WARNING;
	//             }

	//             if (err & FDCAN_PSR_EP) {
	//                 status |= CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_PASSIVE;
	//             }
	//         }

	//         CANmodule->CANerrorStatus = status;
	//     }
	// #else

	// err = ((CAN_HandleTypeDef*)((CANopenNodeSAM3X*)CANmodule->CANptr)->CANHandle)->Instance->ESR
	//       & (CAN_ESR_BOFF | CAN_ESR_EPVF | CAN_ESR_EWGF);

	// //    uint32_t esrVal = ((CAN_HandleTypeDef*)((CANopenNodeSAM3X*)CANmodule->CANptr)->CANHandle)->Instance->ESR; Debug purpose
	// if (CANmodule->errOld != err) {

	//     uint16_t status = CANmodule->CANerrorStatus;

	//     CANmodule->errOld = err;

	//     if (err & CAN_ESR_BOFF) {
	//         status |= CO_CAN_ERRTX_BUS_OFF;
	//         // In this driver, we assume that auto bus recovery is activated ! so this error will eventually handled automatically.

	//     } else {
	//         /* recalculate CANerrorStatus, first clear some flags */
	//         status &= 0xFFFF
	//                   ^ (CO_CAN_ERRTX_BUS_OFF | CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_WARNING
	//                      | CO_CAN_ERRTX_PASSIVE);

	//         if (err & CAN_ESR_EWGF) {
	//             status |= CO_CAN_ERRRX_WARNING | CO_CAN_ERRTX_WARNING;
	//         }

	//         if (err & CAN_ESR_EPVF) {
	//             status |= CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_PASSIVE;
	//         }
	//     }
	
	//     CANmodule->CANerrorStatus = status;
	// }
	// CO_CANverifyErrors(CO_CANmodule_t *CANmodule)

	// #endif
	CO_CANverifyErrors(CANmodule);
}
/******************************************************************************/
void CO_CANinterrupt(CO_CANmodule_t *CANmodule)
{
	uint32_t ul_status;
	
	ul_status = can_get_status(CANModule_local->CANptr);

	if (ul_status & GLOBAL_MAILBOX_MASK)
	{
		for (uint8_t i = 0; i < CANMB_NUMBER; i++)
		{
			ul_status = can_mailbox_get_status(CANModule_local->CANptr, i);

			if ((ul_status & CAN_MSR_MRDY) == CAN_MSR_MRDY) //Mailbox Ready Bit
			{
				
				//Handle interrupt
				if (i < CANMB_TX){
					//Receive interrupt
					CO_CANrxMsg_t *rcvMsg;      /* pointer to received message in CAN module */
					CO_CANrxMsg_t rcvMsgBuf;
					uint16_t index;             /* index of received message */
					uint32_t rcvMsgIdent;       /* identifier of the received message */
					CO_CANrx_t* buffer = NULL;  /* receive message buffer from CO_CANmodule_t object. */
					bool_t msgMatched = false;

					CANModule_local->rxMbConf[i].ul_mb_idx = i;
					CANModule_local->rxMbConf[i].ul_status = ul_status;
					can_mailbox_read(CANModule_local->CANptr, &CANModule_local->rxMbConf[i]);


					/* Get message from module here */
					memset(rcvMsgBuf.data, 0, 8);
					memcpy(rcvMsgBuf.data, &CANModule_local->rxMbConf[i].ul_datal, CANModule_local->rxMbConf[i].uc_length);
					rcvMsgBuf.ident = CANModule_local->rxMbConf[i].ul_fid;
					rcvMsgBuf.DLC = CANModule_local->rxMbConf[i].uc_length;

					rcvMsg = &rcvMsgBuf;
					
					rcvMsgIdent = rcvMsg->ident;

					if(CANModule_local->useCANrxFilters)
					{
						/* CAN module filters are used. Message with known 11-bit identifier has */
						/* been received */
						index = 0;  /* get index of the received message here. Or something similar */
						if(index < CANModule_local->rxSize)
						{
							buffer = &CANModule_local->rxArray[index];
							/* verify also RTR */
							if(((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U)
							{
								msgMatched = true;
							}
						}
						
					}
					else
					{
						/* CAN module filters are not used, message with any standard 11-bit identifier */
						/* has been received. Search rxArray from CANmodule for the same CAN-ID. */
						buffer = CANModule_local->rxArray;

						for(index = CANModule_local->rxSize; index > 0U; --index,++buffer)
						{
							if(((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U)
							{
								msgMatched = true;
								break;
							}
						}
					}
					//buffer->CANrx_callback=&CO_SDO_receive;
					/* Call specific function, which will process the message */
					if(msgMatched && (buffer != NULL) && (buffer->CANrx_callback != NULL))
					{
						buffer->CANrx_callback(buffer->object, (void*)rcvMsg);
					}
				}
				else
				{
					/* First CAN message (bootup) was sent successfully */
					CANmodule->firstCANtxMessage = false;
					/* Clear flag from previous message */
					CANmodule->bufferInhibitFlag = false;
					/* Are there any new messages waiting to be send */
					if(CANmodule->CANtxCount > 0U)
					{
						uint16_t j;    /* Index of transmitting message */

						/* First buffer */
						CO_CANtx_t *buffer = &CANmodule->txArray[0];
						/* Search through whole array of pointers to transmit message buffers. */
						for(j = CANmodule->txSize; j > 0U; j--)
						{
							/* If message buffer is full, send it. */
							if(buffer->bufferFull)
							{
								buffer->bufferFull = false;
								CANmodule->CANtxCount--;

								/* Copy message to CAN buffer */
								CANmodule->txMbConf.ul_datal = *((uint32_t *) &(buffer->data[0]));
								CANmodule->txMbConf.ul_datah = *((uint32_t *) &(buffer->data[4]));

								/* Write transmit information into mailbox. */
								CANmodule->txMbConf.ul_id = CAN_MID_MIDvA(buffer->ident);
								CANmodule->txMbConf.uc_length = buffer->DLC;

								if (buffer->rtr)
								can_mailbox_tx_remote_frame(CANmodule->CANptr, &CANmodule->txMbConf);
								else
								can_mailbox_write(CANmodule->CANptr, &CANmodule->txMbConf);

								can_global_send_transfer_cmd(CANmodule->CANptr, 1 << CANMB_TX);

								break; /* Exit for loop */
							}
							buffer++;
							} /* End of for loop */

							/* Clear counter if no more messages */
							if(j == 0U){
								CANmodule->CANtxCount = 0U;
							}
						}
						else
						{
							/* Nothing more to send */
							can_disable_interrupt(CANmodule->CANptr, 0x1u << CANMB_TX);
						}
					}
					break;
				}
			}
		}
		else
		{
			if (ul_status & CAN_SR_ERRA);   //error active
			if (ul_status & CAN_SR_WARN);   //warning limit
			//CO_EM_CAN_BUS_WARNING
			if (ul_status & CAN_SR_ERRP);   //error passive
			//CO_EM_CAN_TX_BUS_PASSIVE
			if (ul_status & CAN_SR_BOFF);   //bus off
			//CO_EM_CAN_TX_BUS_OFF
			if (ul_status & CAN_SR_SLEEP);  //controller in sleep mode
			if (ul_status & CAN_SR_WAKEUP); //controller woke up
			if (ul_status & CAN_SR_TOVF);   //timer overflow
			if (ul_status & CAN_SR_TSTP);   //timestamp - start or end of frame
			if (ul_status & CAN_SR_CERR);   //CRC error in mailbox
			if (ul_status & CAN_SR_SERR);   //stuffing error in mailbox
			if (ul_status & CAN_SR_AERR);   //ack error
			if (ul_status & CAN_SR_FERR);   //form error
			if (ul_status & CAN_SR_BERR);   //bit error
		}
	}

	/**
	* \brief           Read message from RX FIFO
	* \param           hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
	*                      the configuration information for the specified FDCAN.
	* \param[in]       fifo: Fifo number to use for read
	* \param[in]       fifo_isrs: List of interrupts for respected FIFO
	*/
	//#ifdef CO_SAM3X_FDCAN_Driver
	//static void
	//prv_read_can_received_msg(FDCAN_HandleTypeDef* hfdcan, uint32_t fifo, uint32_t fifo_isrs)
	//#else
	//static void
	//prv_read_can_received_msg(CAN_HandleTypeDef* hcan, uint32_t fifo, uint32_t fifo_isrs)
	//#endif
	//{
	//
	//CO_CANrxMsg_t rcvMsg;
	//CO_CANrx_t* buffer = NULL; /* receive message buffer from CO_CANmodule_t object. */
	//uint16_t index;            /* index of received message */
	//uint32_t rcvMsgIdent;      /* identifier of the received message */
	//uint8_t messageFound = 0;
	//
	//#ifdef CO_SAM3X_FDCAN_Driver
	//static FDCAN_RxHeaderTypeDef rx_hdr;
	///* Read received message from FIFO */
	//if (HAL_FDCAN_GetRxMessage(hfdcan, fifo, &rx_hdr, rcvMsg.data) != HAL_OK) {
	//return;
	//}
	///* Setup identifier (with RTR) and length */
	//rcvMsg.ident = rx_hdr.Identifier | (rx_hdr.RxFrameType == FDCAN_REMOTE_FRAME ? FLAG_RTR : 0x00);
	//switch (rx_hdr.DataLength) {
	//case FDCAN_DLC_BYTES_0:
	//rcvMsg.dlc = 0;
	//break;
	//case FDCAN_DLC_BYTES_1:
	//rcvMsg.dlc = 1;
	//break;
	//case FDCAN_DLC_BYTES_2:
	//rcvMsg.dlc = 2;
	//break;
	//case FDCAN_DLC_BYTES_3:
	//rcvMsg.dlc = 3;
	//break;
	//case FDCAN_DLC_BYTES_4:
	//rcvMsg.dlc = 4;
	//break;
	//case FDCAN_DLC_BYTES_5:
	//rcvMsg.dlc = 5;
	//break;
	//case FDCAN_DLC_BYTES_6:
	//rcvMsg.dlc = 6;
	//break;
	//case FDCAN_DLC_BYTES_7:
	//rcvMsg.dlc = 7;
	//break;
	//case FDCAN_DLC_BYTES_8:
	//rcvMsg.dlc = 8;
	//break;
	//default:
	//rcvMsg.dlc = 0;
	//break; /* Invalid length when more than 8 */
	//}
	//rcvMsgIdent = rcvMsg.ident;
	//#else
	//static CAN_RxHeaderTypeDef rx_hdr;
	///* Read received message from FIFO */
	//if (HAL_CAN_GetRxMessage(hcan, fifo, &rx_hdr, rcvMsg.data) != HAL_OK) {
	//return;
	//}
	///* Setup identifier (with RTR) and length */
	//rcvMsg.ident = rx_hdr.StdId | (rx_hdr.RTR == CAN_RTR_REMOTE ? FLAG_RTR : 0x00);
	//rcvMsg.dlc = rx_hdr.DLC;
	//rcvMsgIdent = rcvMsg.ident;
	//#endif
	//
	///*
	//* Hardware filters are not used for the moment
	//* \todo: Implement hardware filters...
	//*/
	//if (CANModule_local->useCANrxFilters) {
	//__BKPT(0);
	//} else {
	///*
	//* We are not using hardware filters, hence it is necessary
	//* to manually match received message ID with all buffers
	//*/
	//buffer = CANModule_local->rxArray;
	//for (index = CANModule_local->rxSize; index > 0U; --index, ++buffer) {
	//if (((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U) {
	//messageFound = 1;
	//break;
	//}
	//}
	//}
	//
	///* Call specific function, which will process the message */
	//if (messageFound && buffer != NULL && buffer->CANrx_callback != NULL) {
	//buffer->CANrx_callback(buffer->object, (void*)&rcvMsg);
	//}
	//}
	//
	//#ifdef CO_SAM3X_FDCAN_Driver
	///**
	//* \brief           Rx FIFO 0 callback.
	//* \param[in]       hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
	//*                      the configuration information for the specified FDCAN.
	//* \param[in]       RxFifo0ITs: indicates which Rx FIFO 0 interrupts are signaled.
	//*/
	//void
	//HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs) {
	//if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
	//prv_read_can_received_msg(hfdcan, FDCAN_RX_FIFO0, RxFifo0ITs);
	//}
	//}
	//
	///**
	//* \brief           Rx FIFO 1 callback.
	//* \param[in]       hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
	//*                      the configuration information for the specified FDCAN.
	//* \param[in]       RxFifo1ITs: indicates which Rx FIFO 0 interrupts are signaled.
	//*/
	//void
	//HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo1ITs) {
	//if (RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) {
	//prv_read_can_received_msg(hfdcan, FDCAN_RX_FIFO1, RxFifo1ITs);
	//}
	//}
	//
	///**
	//* \brief           TX buffer has been well transmitted callback
	//* \param[in]       hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
	//*                      the configuration information for the specified FDCAN.
	//* \param[in]       BufferIndexes: Bits of successfully sent TX buffers
	//*/
	//void
	//HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t BufferIndexes) {
	//CANModule_local->firstCANtxMessage = false;            /* First CAN message (bootup) was sent successfully */
	//CANModule_local->bufferInhibitFlag = false;            /* Clear flag from previous message */
	//if (CANModule_local->CANtxCount > 0U) {                /* Are there any new messages waiting to be send */
	//CO_CANtx_t* buffer = &CANModule_local->txArray[0]; /* Start with first buffer handle */
	//uint16_t i;
	//
	///*
	//* Try to send more buffers, process all empty ones
	//*
	//* This function is always called from interrupt,
	//* however to make sure no preemption can happen, interrupts are anyway locked
	//* (unless you can guarantee no higher priority interrupt will try to access to FDCAN instance and send data,
	//*  then no need to lock interrupts..)
	//*/
	//CO_LOCK_CAN_SEND(CANModule_local);
	//for (i = CANModule_local->txSize; i > 0U; --i, ++buffer) {
	///* Try to send message */
	//if (buffer->bufferFull) {
	//if (prv_send_can_message(CANModule_local, buffer)) {
	//buffer->bufferFull = false;
	//CANModule_local->CANtxCount--;
	//CANModule_local->bufferInhibitFlag = buffer->syncFlag;
	//}
	//}
	//}
	///* Clear counter if no more messages */
	//if (i == 0U) {
	//CANModule_local->CANtxCount = 0U;
	//}
	//CO_UNLOCK_CAN_SEND(CANModule_local);
	//}
	//}
	//#elif defined(CO_SAM3X_CAN_Driver)
	///**
	//* \brief           Rx FIFO 0 callback.
	//* \param[in]       hcan: pointer to an CAN_HandleTypeDef structure that contains
	//*                      the configuration information for the specified CAN.
	//*/
	//void
	//HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
	//prv_read_can_received_msg(hcan, CAN_RX_FIFO0, 0);
	//}
	//
	///**
	//* \brief           Rx FIFO 1 callback.
	//* \param[in]       hcan: pointer to an CAN_HandleTypeDef structure that contains
	//*                      the configuration information for the specified CAN.
	//*/
	//void
	//HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan) {
	//prv_read_can_received_msg(hcan, CAN_RX_FIFO1, 0);
	//}

	/**
	* \brief           TX buffer has been well transmitted callback
	* \param[in]       hcan: pointer to an CAN_HandleTypeDef structure that contains
	*                      the configuration information for the specified CAN.
	* \param[in]       MailboxNumber: the mailbox number that has been transmitted
	*/
	void
	CO_CANinterrupt_TX(CO_CANmodule_t* CANmodule, uint32_t MailboxNumber) {

		// CANmodule->firstCANtxMessage = false;            /* First CAN message (bootup) was sent successfully */
		// CANmodule->bufferInhibitFlag = false;            /* Clear flag from previous message */
		// if (CANmodule->CANtxCount > 0U) {                /* Are there any new messages waiting to be send */
		//     CO_CANtx_t* buffer = &CANmodule->txArray[0]; /* Start with first buffer handle */
		//     uint16_t i;

		//     /*
		//  * Try to send more buffers, process all empty ones
		//  *
		//  * This function is always called from interrupt,
		//  * however to make sure no preemption can happen, interrupts are anyway locked
		//  * (unless you can guarantee no higher priority interrupt will try to access to CAN instance and send data,
		//  *  then no need to lock interrupts..)
		//  */
		//     CO_LOCK_CAN_SEND(CANmodule);
		//     for (i = CANmodule->txSize; i > 0U; --i, ++buffer) {
		//         /* Try to send message */
		//         if (buffer->bufferFull) {
		//             if (prv_send_can_message(CANmodule, buffer)) {
		//                 buffer->bufferFull = false;
		//                 CANmodule->CANtxCount--;
		//                 CANmodule->bufferInhibitFlag = buffer->syncFlag;
		//             }
		//         }
		//     }
		//     /* Clear counter if no more messages */
		//     if (i == 0U) {
		//         CANmodule->CANtxCount = 0U;
		//     }
		//     CO_UNLOCK_CAN_SEND(CANmodule);
		// }
	}

	//void
	//HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef* hcan) {
	//CO_CANinterrupt_TX(CANModule_local, CAN_TX_MAILBOX0);
	//}
	//
	//void
	//HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef* hcan) {
	//CO_CANinterrupt_TX(CANModule_local, CAN_TX_MAILBOX0);
	//}
	//
	//void
	//HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef* hcan) {
	//CO_CANinterrupt_TX(CANModule_local, CAN_TX_MAILBOX0);
	//}
	//#endif

#ifdef __cplusplus
}
#endif