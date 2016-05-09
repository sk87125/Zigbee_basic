/****************************************************************************
 *
 * MODULE:             wuart_e.c
 *
 * COMPONENT:          $RCSfile: wuart_e.c,v $
 *
 * VERSION:            $Name:  $
 *
 * REVISION:           $Revision: 1.1.1.1 $
 *
 * DATED:              $Date: 2007/04/18 03:10:39 $
 *
 * STATUS:             $State: Exp $
 *
 * AUTHOR:             Ian Morris
 *
 * DESCRIPTION
 *
 * CHANGE HISTORY:
 *
 * $Log: wuart_e.c,v $
 * Revision 1.1.1.1  2007/04/18 03:10:39  isaac_tung
 * no message
 *
 * Revision 1.1  2006/08/24 14:58:28  imorr
 * Initial version
 *
 *
 *
 * LAST MODIFIED BY:   $Author: isaac_tung $
 *                     $Modtime: $
 *
 *
 ****************************************************************************
 *
 *  (c) Copyright 2000 JENNIC Ltd
 *
 ****************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include <jendefs.h>
#include <AppHardwareApi.h>
#include <AppQueueApi.h>
#include <mac_sap.h>
#include <mac_pib.h>
#include <string.h>
#include <AppApi.h>

#include "config.h"
#include "serialq.h"
#include "uart.h"
#include "serial.h"

#include <FontalButton.h>
#include <FontalLED.h>
#include "Fttml-lite.h"
#include <HtsDriver.h>
/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
/* State machine states */
typedef enum
{
    E_STATE_OFF,
    E_STATE_SCANNING,
    E_STATE_ASSOCIATING,
    E_STATE_RUNNING,
} teState;

/* All application data with scope within the entire file is kept here,
   including all stored node data */
typedef struct
{
    struct
    {
        teState eState;
        uint8   u8Channel;
        uint16  u16ShortAddr;
    } sSystem;
} tsDeviceData;

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
extern uint8 gu8SenserCnt;
extern bool_t gbSenserFlag;
/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
PRIVATE tsDeviceData sDeviceData;
uint8 u8TxFrameHandle = 0;
uint8 u8RxFrameHandle = 0;

bool_t bToggle;
uint16 u16TestCount = 0;

PRIVATE void *pvMac;
PRIVATE MAC_Pib_s *psPib;

uint8 gu8PowerLevel = 0;
uint8 bFlag;//ziyi
int gdPower[] = {0x00,0x3a,0x34,0x2e,0x28,0x22}; //{16,10,4,-2,-8,-12};

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/
PRIVATE void vWUART_Init(void);
PRIVATE void vStartActiveScan(void);
PRIVATE void vStartAssociate(void);
PRIVATE void vProcessEventQueues(void);
PRIVATE void vProcessIncomingMlme(MAC_MlmeDcfmInd_s *psMlmeInd);
PRIVATE void vProcessIncomingData(MAC_McpsDcfmInd_s *psMcpsInd);
PRIVATE void vProcessIncomingHwEvent(AppQApiHwInd_s *psAHI_Ind);
PRIVATE void vHandleActiveScanResponse(MAC_MlmeDcfmInd_s *psMlmeInd);
PRIVATE void vHandleAssociateResponse(MAC_MlmeDcfmInd_s *psMlmeInd);
PRIVATE void vWUART_TxData(void);
PRIVATE void vTickTimerISR(uint32 u32Device, uint32 u32ItemBitmap);

PUBLIC void vUartPrint(uint8 u8UartPort, char * pString);
PUBLIC void Num2Str(char *pcString, uint16 u16Data);
/****************************************************************************
 *
 * NAME: AppColdStart
 *
 * DESCRIPTION:
 *
 * PARAMETERS:      Name            RW  Usage
 * None.
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * Entry point for a power on reset or wake from sleep mode.
 ****************************************************************************/
PUBLIC void AppColdStart(void)
{
	bFlag = FALSE;//ziyi
    vWUART_Init();

    vStartActiveScan();

    while(1)
    {
        vProcessEventQueues();
        // do something

    }
}

/****************************************************************************
 *
 * NAME: AppWarmStart
 *
 * DESCRIPTION:
 * Entry point for a wake from sleep mode with the memory contents held. We
 * are not using this mode and so should never get here.
 *
 * PARAMETERS:      Name            RW  Usage
 * None.
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
PUBLIC void AppWarmStart(void)
{
    AppColdStart();
}

/****************************************************************************
 *
 * NAME: vWUART_Init
 *
 * DESCRIPTION:
 * Initialises stack and hardware, sets non-default values in the 802.15.4
 * PIB.
 *
 * PARAMETERS:      Name            RW  Usage
 * None.
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
PRIVATE void vWUART_Init(void)
{



    sDeviceData.sSystem.eState = E_STATE_OFF;

    /* Initialise stack and hardware interfaces. We aren't using callbacks
       at all, just monitoring the upward queues in a loop */
    (void)u32AppQApiInit(NULL, NULL, NULL);
    (void)u32AHI_Init();

    led_init();
    btn_init();
    vAHI_DioInterruptEnable(0,BUTTON0|BUTTON1);



    pvMac = pvAppApiGetMacHandle();
	psPib = MAC_psPibGetHandle(pvMac);

    /* Set Pan ID in PIB (also sets match register in hardware) */
    MAC_vPibSetPanId(pvMac, PAN_ID);

    /* Enable receiver to be on when idle */
    MAC_vPibSetRxOnWhenIdle(pvMac, 1, FALSE);

    /* Initialise the serial port and rx/tx queues */
    //vSerial_Init();

    /* Initialise tick timer to give 10ms interrupt */
    vAHI_TickTimerConfigure(E_AHI_TICK_TIMER_DISABLE);
    vAHI_TickTimerWrite(0);
    vAHI_TickTimerInit(vTickTimerISR);
    vAHI_TickTimerInterval(TICKER_PERIOD/4);  // 1/4s
    vAHI_TickTimerConfigure(E_AHI_TICK_TIMER_RESTART);
    vAHI_TickTimerIntEnable(TRUE);
	//********************************************************
	vHTSreset();
	while (1)
	{

	}


}

/****************************************************************************
 *
 * NAME: vProcessEventQueues
 *
 * DESCRIPTION:
 * Check each of the three event queues and process and items found.
 *
 * PARAMETERS:      Name            RW  Usage
 * None.
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
PRIVATE void vProcessEventQueues(void)
{
    MAC_MlmeDcfmInd_s *psMlmeInd;
	MAC_McpsDcfmInd_s *psMcpsInd;
    AppQApiHwInd_s    *psAHI_Ind;

    /* Check for anything on the MCPS upward queue */
    do
    {
        psMcpsInd = psAppQApiReadMcpsInd();
        if (psMcpsInd != NULL)
        {
            vProcessIncomingData(psMcpsInd);
            vAppQApiReturnMcpsIndBuffer(psMcpsInd);
        }
    } while (psMcpsInd != NULL);

    /* Check for anything on the MLME upward queue */
    do
    {
        psMlmeInd = psAppQApiReadMlmeInd();
        if (psMlmeInd != NULL)
        {
            vProcessIncomingMlme(psMlmeInd);
            vAppQApiReturnMlmeIndBuffer(psMlmeInd);
        }
    } while (psMlmeInd != NULL);

    /* Check for anything on the AHI upward queue */
    do
    {
        psAHI_Ind = psAppQApiReadHwInd();
        if (psAHI_Ind != NULL)
        {
            vProcessIncomingHwEvent(psAHI_Ind);
            vAppQApiReturnHwIndBuffer(psAHI_Ind);
        }
    } while (psAHI_Ind != NULL);
}

/****************************************************************************
 *
 * NAME: vProcessIncomingMlme
 *
 * DESCRIPTION:
 * Process any incoming managment events from the stack.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  psMlmeInd
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
PRIVATE void vProcessIncomingMlme(MAC_MlmeDcfmInd_s *psMlmeInd)
{
    /* We respond to several MLME indications and confirmations, depending
       on mode */
    switch (psMlmeInd->u8Type)
    {
    /* Deferred confirmation that the scan is complete */
    case MAC_MLME_DCFM_SCAN:
        /* Only respond to this if scanning */
        if (sDeviceData.sSystem.eState == E_STATE_SCANNING)
        {
            vHandleActiveScanResponse(psMlmeInd);
        }
        break;

    /* Deferred confirmation that the association process is complete */
    case MAC_MLME_DCFM_ASSOCIATE:
        /* Only respond to this if associating */
        if (sDeviceData.sSystem.eState == E_STATE_ASSOCIATING)
        {
            vHandleAssociateResponse(psMlmeInd);
        }
        break;

    default:
        break;
    }
}

/****************************************************************************
 *
 * NAME: vProcessIncomingData
 *
 * DESCRIPTION:
 * Process incoming data events from the stack.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  psMcpsInd
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
PRIVATE void vProcessIncomingData(MAC_McpsDcfmInd_s *psMcpsInd)
{
    MAC_RxFrameData_s *psFrame;
    MAC_Addr_s *psAddr;
    uint16 u16NodeAddr;


    psFrame = &psMcpsInd->uParam.sIndData.sFrame;
    psAddr = &psFrame->sSrcAddr;

    /* Check that this is a data frame */
    if (psMcpsInd->u8Type == MAC_MCPS_IND_DATA)
    {
        /* Check that data is from UART node */
        u16NodeAddr = psAddr->uAddr.u16Short;


//        if (u16NodeAddr == COORD_ADDR)
        {
            //if (psFrame->au8Sdu[0] == u8RxFrameHandle)
            if (FtTML_CheckRecvSeq(u16NodeAddr,psFrame->au8Sdu[0]) == FT_RECV_SEQ_NO_ERROR)
            {

            }
            /* Must have missed a frame */
            else if (psFrame->au8Sdu[0] > u8RxFrameHandle)
            {

            }
            /* Must be the same frame as last time */
            else if (psFrame->au8Sdu[0] < u8RxFrameHandle)
            {
                /* Dont do anything as we already have the data */
            }
        }

    }
}

/****************************************************************************
 *
 * NAME: vProcessIncomingHwEvent
 *
 * DESCRIPTION:
 * Process any hardware events.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  psAHI_Ind
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
PRIVATE void vProcessIncomingHwEvent(AppQApiHwInd_s *psAHI_Ind)
{
    if ((psAHI_Ind->u32DeviceId == E_AHI_DEVICE_SYSCTRL) )
    {

    }

}

/****************************************************************************
 *
 * NAME: vTickTimerISR
 *
 * DESCRIPTION:
 *
 *
 * PARAMETERS:      Name            RW  Usage
 * None.
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
PRIVATE void vTickTimerISR(uint32 u32Device, uint32 u32ItemBitmap)
{
	uint16 u8Temp = 0;
    uint16 u8Humidity = 0;
    char aChar[5];

    if ((sDeviceData.sSystem.eState == E_STATE_RUNNING) )
    {
		//Read data
		while(bFlag<3){
			if (bFlag== 0)
			vHTSstartReadTemp();
			else if (bFlag== 1)	{
			u8Temp =  u16HTSreadTempResult();
			Num2Str( aChar,u8Temp);
			vUartPrint(0,"Temp ");
			vUartPrint(0,aChar);
			vUartPrint(0,"\t\t");
			}
			else if (bFlag == 2)
			vHTSstartReadHumidity();
			else if (bFlag== 3){
			u8Humidity =  u16HTSreadHumidityResult();
			Num2Str( aChar,u8Humidity);
			vUartPrint(0,"Humidity ");
			vUartPrint(0,aChar);
			vUartPrint(0,"\r");
			}
			bFlag++;
		}
		bFlag = 0;

        // send data
        if (btn_pressed(BUTTON1))
            vWUART_TxData();

        // change power level
        if (btn_pressed(BUTTON0))
        {
            if (gu8PowerLevel >=4)
                gu8PowerLevel = 0;
            else
                gu8PowerLevel++;
            eAppApiPlmeSet(PHY_PIB_ATTR_TX_POWER,gdPower[gu8PowerLevel]);
            led_toggle(LED0);
        }

    }
}

/****************************************************************************
 *
 * NAME: vStartActiveScan
 *
 * DESCRIPTION:
 * Start a scan to search for a network to join.
 *
 * PARAMETERS:      Name            RW  Usage
 * None.
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
PRIVATE void vStartActiveScan(void)
{
    MAC_MlmeReqRsp_s  sMlmeReqRsp;
    MAC_MlmeSyncCfm_s sMlmeSyncCfm;

    sDeviceData.sSystem.eState = E_STATE_SCANNING;

    /* Request scan */
    sMlmeReqRsp.u8Type = MAC_MLME_REQ_SCAN;
    sMlmeReqRsp.u8ParamLength = sizeof(MAC_MlmeReqScan_s);
    sMlmeReqRsp.uParam.sReqScan.u8ScanType = MAC_MLME_SCAN_TYPE_ACTIVE;
    sMlmeReqRsp.uParam.sReqScan.u32ScanChannels = SCAN_CHANNELS;
    sMlmeReqRsp.uParam.sReqScan.u8ScanDuration = ACTIVE_SCAN_DURATION;

    vAppApiMlmeRequest(&sMlmeReqRsp, &sMlmeSyncCfm);
}

/****************************************************************************
 *
 * NAME: vHandleActiveScanResponse
 *
 * DESCRIPTION:
 * Handle the reponse generated by the stack as a result of the network scan.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  psMlmeInd
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
PRIVATE void vHandleActiveScanResponse(MAC_MlmeDcfmInd_s *psMlmeInd)
{
    MAC_PanDescr_s *psPanDesc;
    int i;

    /* Make sure it is what we're after */
    if ((psMlmeInd->uParam.sDcfmScan.u8Status == MAC_ENUM_SUCCESS)
        && (psMlmeInd->uParam.sDcfmScan.u8ScanType == MAC_MLME_SCAN_TYPE_ACTIVE))
    {
        /* Determine which, if any, network contains demo coordinator.
           Algorithm for determining which network to connect to is
           beyond the scope of 802.15.4, and we use a simple approach
           of matching the required PAN ID and short address, both of
           which we already know */

        i = 0;
        while (i < psMlmeInd->uParam.sDcfmScan.u8ResultListSize)
        {
            psPanDesc = &psMlmeInd->uParam.sDcfmScan.uList.asPanDescr[i];

            if ((psPanDesc->sCoord.u16PanId == PAN_ID)
             && (psPanDesc->sCoord.u8AddrMode == 2)
             && (psPanDesc->sCoord.uAddr.u16Short == COORD_ADDR))
            {
                /* Matched so start to synchronise and associate */
                sDeviceData.sSystem.u8Channel = psPanDesc->u8LogicalChan;
                vStartAssociate();
                return;
            }
            i++;
        }
    }
    /* Failed to find coordinator: keep trying */
    vStartActiveScan();
}

/****************************************************************************
 *
 * NAME: vStartAssociate
 *
 * DESCRIPTION:
 * Start the association process with the network coordinator.
 *
 * PARAMETERS:      Name            RW  Usage
 * None.
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * Assumes that a network has been found during the network scan.
 ****************************************************************************/
PRIVATE void vStartAssociate(void)
{
    MAC_MlmeReqRsp_s  sMlmeReqRsp;
    MAC_MlmeSyncCfm_s sMlmeSyncCfm;

    sDeviceData.sSystem.eState = E_STATE_ASSOCIATING;

    /* Create associate request. We know short address and PAN ID of
       coordinator as this is preset and we have checked that received
       beacon matched this */
    sMlmeReqRsp.u8Type = MAC_MLME_REQ_ASSOCIATE;
    sMlmeReqRsp.u8ParamLength = sizeof(MAC_MlmeReqAssociate_s);
    sMlmeReqRsp.uParam.sReqAssociate.u8LogicalChan = sDeviceData.sSystem.u8Channel;
    sMlmeReqRsp.uParam.sReqAssociate.u8Capability = 0x80; /* We want short address, other features off */
    sMlmeReqRsp.uParam.sReqAssociate.u8SecurityEnable = FALSE;
    sMlmeReqRsp.uParam.sReqAssociate.sCoord.u8AddrMode = 2;
    sMlmeReqRsp.uParam.sReqAssociate.sCoord.u16PanId = PAN_ID;
    sMlmeReqRsp.uParam.sReqAssociate.sCoord.uAddr.u16Short = COORD_ADDR;

    vAppApiMlmeRequest(&sMlmeReqRsp, &sMlmeSyncCfm);
}

/****************************************************************************
 *
 * NAME: vHandleAssociateResponse
 *
 * DESCRIPTION:
 * Handle the response generated by the stack as a result of the associate
 * start request.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  psMlmeInd
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
PRIVATE void vHandleAssociateResponse(MAC_MlmeDcfmInd_s *psMlmeInd)
{
    /* If successfully associated with network coordinator */
    if (psMlmeInd->uParam.sDcfmAssociate.u8Status == MAC_ENUM_SUCCESS)
    {
        /* Store short address that we have been assigned */
        sDeviceData.sSystem.u16ShortAddr = psMlmeInd->uParam.sDcfmAssociate.u16AssocShortAddr;
        /* We are now in the running state */
        sDeviceData.sSystem.eState = E_STATE_RUNNING;
        led_off(LED0);
        led_on(LED1);
    }
    else
    {
        /* Try, try again */
    	vStartActiveScan();
    }
}

/****************************************************************************
 *
 * NAME: vTxUARTData
 *
 * DESCRIPTION:
 *
 * PARAMETERS:      Name            RW  Usage
 * None.
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
PRIVATE void vWUART_TxData(void)
{
    MAC_McpsReqRsp_s  sMcpsReqRsp;
    MAC_McpsSyncCfm_s sMcpsSyncCfm;
    uint8 *pu8Payload, i = 0;
    int16 i16RxChar;
    uint16 u16DstAddr = COORD_ADDR;

    uint8 aPLData[] = "-abcdefghijklmnopq-";


    if (i16RxChar >= 0)
    {

        eAppApiPlmeSet(PHY_PIB_ATTR_TX_POWER, gdPower[gu8PowerLevel]&0x3f);

        /* Create frame transmission request */
        sMcpsReqRsp.u8Type = MAC_MCPS_REQ_DATA;
        sMcpsReqRsp.u8ParamLength = sizeof(MAC_McpsReqData_s);
        /* Set handle so we can match confirmation to request */
        sMcpsReqRsp.uParam.sReqData.u8Handle = 1;
        /* Use short address for source */
        sMcpsReqRsp.uParam.sReqData.sFrame.sSrcAddr.u8AddrMode = 2;
        sMcpsReqRsp.uParam.sReqData.sFrame.sSrcAddr.u16PanId = PAN_ID;
        sMcpsReqRsp.uParam.sReqData.sFrame.sSrcAddr.uAddr.u16Short = sDeviceData.sSystem.u16ShortAddr;
        /* Use short address for destination */
        sMcpsReqRsp.uParam.sReqData.sFrame.sDstAddr.u8AddrMode = 2;
        sMcpsReqRsp.uParam.sReqData.sFrame.sDstAddr.u16PanId = PAN_ID;
        sMcpsReqRsp.uParam.sReqData.sFrame.sDstAddr.uAddr.u16Short = u16DstAddr;
        /* Frame requires ack but not security, indirect transmit or GTS */
        sMcpsReqRsp.uParam.sReqData.sFrame.u8TxOptions = MAC_TX_OPTION_ACK;

        pu8Payload = sMcpsReqRsp.uParam.sReqData.sFrame.au8Sdu;
        //pu8Payload[i++] = u8TxFrameHandle++;
        pu8Payload[i++] = FtTML_GetNextSendSeq(sMcpsReqRsp.uParam.sReqData.sFrame.sDstAddr.uAddr.u16Short);
        pu8Payload[i++] = 20;
        pu8Payload[i++] = 5-gu8PowerLevel;

        memcpy(pu8Payload+i, aPLData,19);
        i+=20;

        /* Set frame length */
        sMcpsReqRsp.uParam.sReqData.sFrame.u8SduLength = i;
        led_toggle(LED1);
        /* Request transmit */
        vAppApiMcpsRequest(&sMcpsReqRsp, &sMcpsSyncCfm);
    }
}
PUBLIC void Num2Str(char *pcString, uint16 u16Data)
{
    	//char numList[] = "0123456789";
	int8 u8Ind = 4;

//	for (u8I = 4; u8I >=0; u8I--)
	do{
		//if (u16Data%10 == 0)
		//	*(pcString+i) = 0x30;
		//else
		pcString[u8Ind]= 0x30 +(u16Data%10);
		u16Data /=10;
		u8Ind--;
	}
	while (u8Ind>=0);
	pcString[5] = 0;
//	uartio_print(pcString);
}
// print string to UART function.........
PUBLIC void vUartPrint(uint8 u8UartPort, char * pString)
{
    while (*pString)
     {
         while ((u8AHI_UartReadLineStatus(u8UartPort) & 0x20) == 0);
         vAHI_UartWriteData(0, *pString);
         pString++;
     }
}
/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
