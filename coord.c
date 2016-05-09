/****************************************************************************
 *
 * MODULE:             wuart_c.c
 *
 * COMPONENT:          $RCSfile: wuart_c.c,v $
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
 * $Log: wuart_c.c,v $
 * Revision 1.1.1.1  2007/04/18 03:10:39  isaac_tung
 * no message
 *
 * Revision 1.1  2006/08/24 14:58:35  imorr
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
/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/* System states with respect to screen display being shown */
typedef enum
{
    E_STATE_INIT,
    E_STATE_START_ENERGY_SCAN,
    E_STATE_ENERGY_SCANNING,
    E_STATE_START_COORDINATOR,
    E_STATE_RUNNING_UART_APP

} teState;

/* Used to track an association between extended address and short address */
typedef struct
{
    uint32 u32ExtAddrLo;
    uint32 u32ExtAddrHi;
    uint16 u16ShortAddr;

    uint16 u16RecvCnt;
    uint8  u8Power;
    uint8  u8LastLQI;
	uint16 u8Temp;
    uint16 u8Humidity;
    //char aChar[5];
} tsAssocNodes;


/* All application data with scope within the entire file is kept here,
   including all stored node data, GUI settings and current state */
typedef struct
{
    struct
	{
	    tsAssocNodes asAssocNodes[MAX_UART_NODES];
	    uint8        u8AssociatedNodes;
	} sNode;

    struct
    {
        teState eState;
        uint8   u8Channel;
    } sSystem;
} tsCoordData;

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
PRIVATE tsCoordData sCoordData;
PRIVATE uint8 u8RxFrameHandle = 0;
PRIVATE void *pvMac;
PRIVATE MAC_Pib_s *psPib;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/
PRIVATE void vWUART_Init(void);
PRIVATE void vWUART_TxData(void);
PRIVATE void vProcessEventQueues(void);
PRIVATE void vStartCoordinator(void);
PRIVATE void vHandleNodeAssociation(MAC_MlmeDcfmInd_s *psMlmeInd);
PRIVATE void vProcessIncomingMlme(MAC_MlmeDcfmInd_s *psMlmeInd);
PRIVATE void vProcessIncomingData(MAC_McpsDcfmInd_s *psMcpsInd);
PRIVATE void vProcessIncomingHwEvent(AppQApiHwInd_s *psAHI_Ind);
PRIVATE void vTickTimerISR(uint32 u32Device, uint32 u32ItemBitmap);
PRIVATE void vStartEnergyScan(void);
PRIVATE void vHandleEnergyScanResponse(MAC_MlmeDcfmInd_s *psMlmeInd);
PRIVATE void vDrawUI(void);
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
    vWUART_Init();

    while(1)
    {
        vProcessEventQueues();

        switch (sCoordData.sSystem.eState)
        {
        case E_STATE_INIT:
            sCoordData.sSystem.u8Channel = CHANNEL_MIN;
            sCoordData.sSystem.eState = E_STATE_START_ENERGY_SCAN;
            break;

        case E_STATE_START_ENERGY_SCAN:
            vStartEnergyScan();
            sCoordData.sSystem.eState = E_STATE_ENERGY_SCANNING;
            break;

        case E_STATE_ENERGY_SCANNING:
            break;

        case E_STATE_START_COORDINATOR:
            vStartCoordinator();
            sCoordData.sSystem.eState = E_STATE_RUNNING_UART_APP;
            break;

        case E_STATE_RUNNING_UART_APP:
// do something
            break;
        }

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

    sCoordData.sNode.u8AssociatedNodes = 0;

    /* Initialise stack and hardware interfaces. We aren't using callbacks
       at all, just monitoring the upward queues in a loop */
    (void)u32AppQApiInit(NULL, NULL, NULL);
    (void)u32AHI_Init();

    led_init();
    btn_init();
    vAHI_DioInterruptEnable(0,BUTTON0|BUTTON1);

    pvMac = pvAppApiGetMacHandle();
    psPib = MAC_psPibGetHandle(pvMac);

    /* Set Pan ID and short address in PIB (also sets match registers in hardware) */
    MAC_vPibSetPanId(pvMac, PAN_ID);
    MAC_vPibSetShortAddr(pvMac, COORD_ADDR);

    /* Allow nodes to associate */
    psPib->bAssociationPermit = 1;

    /* Enable receiver to be on when idle */
    MAC_vPibSetRxOnWhenIdle(pvMac, 1, FALSE);

    /* Initialise the serial port and rx/tx queues */
    vSerial_Init();

    /* Initialise tick timer to give 10ms interrupt */
    vAHI_TickTimerConfigure(E_AHI_TICK_TIMER_DISABLE);
    vAHI_TickTimerWrite(0);
    vAHI_TickTimerInit(vTickTimerISR);
    vAHI_TickTimerInterval(TICKER_PERIOD*5);
    vAHI_TickTimerConfigure(E_AHI_TICK_TIMER_RESTART);
    vAHI_TickTimerIntEnable(TRUE);
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
    switch(psMlmeInd->u8Type)
    {
    case MAC_MLME_IND_ASSOCIATE:

        /* Only allow nodes to associate if network has been started */
        if (sCoordData.sSystem.eState == E_STATE_RUNNING_UART_APP)
        {
            vHandleNodeAssociation(psMlmeInd);
        }
        break;

    case MAC_MLME_DCFM_SCAN:

        if (psMlmeInd->uParam.sDcfmScan.u8Status == MAC_ENUM_SUCCESS)
        {
            if (psMlmeInd->uParam.sDcfmScan.u8ScanType == MAC_MLME_SCAN_TYPE_ENERGY_DETECT)
            {
                if (sCoordData.sSystem.eState == E_STATE_ENERGY_SCANNING)
                {
                    vHandleEnergyScanResponse(psMlmeInd);
                    sCoordData.sSystem.eState = E_STATE_START_COORDINATOR;
                }
            }
        }
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
    uint8 i;
    char aSArrd[6];

    psFrame = &psMcpsInd->uParam.sIndData.sFrame;
    psAddr = &psFrame->sSrcAddr;

    /* Check that this is a data frame */
    if (psMcpsInd->u8Type == MAC_MCPS_IND_DATA)
    {
        /* Check that data is from UART node */
        u16NodeAddr = psAddr->uAddr.u16Short;
        memset(aSArrd,'\0',6);

        if (FtTML_CheckRecvSeq(u16NodeAddr,psFrame->au8Sdu[0]) == FT_RECV_SEQ_NO_ERROR)
        {
            for (i = 0; i< sCoordData.sNode.u8AssociatedNodes; i++)
            {
                if (u16NodeAddr == sCoordData.sNode.asAssocNodes[i].u16ShortAddr)
                {
                    sCoordData.sNode.asAssocNodes[i].u16RecvCnt++;
                    led_toggle(LED1);
                    sCoordData.sNode.asAssocNodes[i].u8Power= psFrame->au8Sdu[2];
                    sCoordData.sNode.asAssocNodes[i].u8LastLQI = psFrame->u8LinkQuality;
					sCoordData.sNode.asAssocNodes[i].u8Temp = psFrame->u8LinkQuality;//ziyi
					sCoordData.sNode.asAssocNodes[i].u8Humidity = psFrame->u8LinkQuality;//ziyi
                }
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
            if (btn_pressed(BUTTON0))
                	led_toggle(LED0);
            else if (btn_pressed(BUTTON1))
                	led_toggle(LED1);
        }
    #if UART == E_AHI_UART_0
        /* If this is an event from UART0 */
        if (psAHI_Ind->u32DeviceId == E_AHI_DEVICE_UART0)
        {
            /* If data has been received */
            if ((psAHI_Ind->u32ItemBitmap & 0x000000FF) == E_AHI_UART_INT_RXDATA)
            {
                // redraw Page
                if (((psAHI_Ind->u32ItemBitmap & 0x0000FF00) >> 8) == 0x20)
                    vDrawUI();

            }
            else if (psAHI_Ind->u32ItemBitmap == E_AHI_UART_INT_TX)
            {

            }
        }
    #else
        /* If this is an event from UART1 */
        if (psAHI_Ind->u32DeviceId == E_AHI_DEVICE_UART1)
        {
            /* If data has been received */
            if ((psAHI_Ind->u32ItemBitmap & 0x000000FF) == E_AHI_UART_INT_RXDATA)
            {
                if ((tLocal.u8State ==STATE_WIRELESS_UART) && (((psAHI_Ind->u32ItemBitmap & 0x0000FF00) >> 8) != 0x1b) && (sCoordData.sSystem.eState == E_STATE_RUNNING_UART_APP))
                {
                    /* Process UART0 RX interrupt */
                    vUART_RxCharISR((psAHI_Ind->u32ItemBitmap & 0x0000FF00) >> 8);
                    while ((u8AHI_UartReadLineStatus(UART) & 0x20) == 0);
                    vAHI_UartWriteData(UART,(psAHI_Ind->u32ItemBitmap & 0x0000FF00) >> 8);
                    if ((((psAHI_Ind->u32ItemBitmap & 0x0000FF00) >> 8) =='\n') || (((psAHI_Ind->u32ItemBitmap & 0x0000FF00) >> 8) =='\r'))
                    {
                          vSerial_TxChar('\n');
                          vSerial_TxChar(':');
                          vSerialQ_AddItem(RX_QUEUE, '\n');
                        vWUART_TxData();
                    }
                }
                else if ((tLocal.u8State ==STATE_UART_EVENT) && (((psAHI_Ind->u32ItemBitmap & 0x0000FF00) >> 8) != 0x1b))
                {
                    while ((u8AHI_UartReadLineStatus(UART) & 0x20) == 0);
                    vAHI_UartWriteData(UART,(psAHI_Ind->u32ItemBitmap & 0x0000FF00) >> 8);
                }
                else vProcInput((psAHI_Ind->u32ItemBitmap & 0x0000FF00) >> 8);
            }
            else if (psAHI_Ind->u32ItemBitmap == E_AHI_UART_INT_TX)
            {
                if (tLocal.u8State ==STATE_WIRELESS_UART)
                {
                    /* Process UART0 TX interrupt */
                    vUART_TxCharISR();
                }
            }
        }
    #endif
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
    vDrawUI();
}

/****************************************************************************
 *
 * NAME: vHandleNodeAssociation
 *
 * DESCRIPTION:
 * Handle request by node to join the network. If the nodes address matches
 * the address of a light switch then it is assumed to be a light switch and
 * is allowed to join the network.
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
PRIVATE void vHandleNodeAssociation(MAC_MlmeDcfmInd_s *psMlmeInd)
{
    MAC_MlmeReqRsp_s   sMlmeReqRsp;
    MAC_MlmeSyncCfm_s  sMlmeSyncCfm;

    uint16 u16ShortAddress;
    uint8 u8NodeInd = 0;

    /* Default to PAN access denied */
    uint8 u8AssocStatus = 2;

    /* Default short address */
    u16ShortAddress = 0xffff;

    /* Check that the device only wants to use a short address */
    if (psMlmeInd->uParam.sIndAssociate.u8Capability & 0x80)
    {
        for (u8NodeInd = 0; u8NodeInd < sCoordData.sNode.u8AssociatedNodes ; u8NodeInd++)
        {
            if ((memcmp(&sCoordData.sNode.asAssocNodes[u8NodeInd].u32ExtAddrLo,&psMlmeInd->uParam.sIndAssociate.sDeviceAddr.u32L,4) == 0)
                && (memcmp(&sCoordData.sNode.asAssocNodes[u8NodeInd].u32ExtAddrHi,&psMlmeInd->uParam.sIndAssociate.sDeviceAddr.u32H,4) == 0))
            {
                u16ShortAddress =    sCoordData.sNode.asAssocNodes[u8NodeInd].u16ShortAddr;
                u8AssocStatus = 0;
                sCoordData.sNode.asAssocNodes[u8NodeInd].u16RecvCnt = 0;
                sCoordData.sNode.asAssocNodes[u8NodeInd].u8Power = 0;
                sCoordData.sNode.asAssocNodes[u8NodeInd].u8LastLQI = 0;			
				sCoordData.sNode.asAssocNodes[sCoordData.sNode.u8AssociatedNodes].u8Temp = 0;			
				sCoordData.sNode.asAssocNodes[sCoordData.sNode.u8AssociatedNodes].u8Humidity = 0;
                FtTML_ResetSeq(u16ShortAddress);
                led_toggle(LED0);
                break;
            }
        }

        if (u16ShortAddress != 0xffff) {}
        else if (sCoordData.sNode.u8AssociatedNodes < MAX_UART_NODES)
        {
            /* Allocate short address as next in list */
            u16ShortAddress = UART_NODE_ADDR_BASE + sCoordData.sNode.u8AssociatedNodes;

            /* Store details for future use */
            sCoordData.sNode.asAssocNodes[sCoordData.sNode.u8AssociatedNodes].u16ShortAddr = u16ShortAddress;
            sCoordData.sNode.asAssocNodes[sCoordData.sNode.u8AssociatedNodes].u32ExtAddrLo = psMlmeInd->uParam.sIndAssociate.sDeviceAddr.u32L;
            sCoordData.sNode.asAssocNodes[sCoordData.sNode.u8AssociatedNodes].u32ExtAddrHi = psMlmeInd->uParam.sIndAssociate.sDeviceAddr.u32H;
            sCoordData.sNode.asAssocNodes[sCoordData.sNode.u8AssociatedNodes].u16RecvCnt = 0;
            sCoordData.sNode.asAssocNodes[sCoordData.sNode.u8AssociatedNodes].u8Power = 0;
            sCoordData.sNode.asAssocNodes[sCoordData.sNode.u8AssociatedNodes].u8LastLQI = 0;			
            sCoordData.sNode.asAssocNodes[sCoordData.sNode.u8AssociatedNodes].u8Temp = 0;			
            sCoordData.sNode.asAssocNodes[sCoordData.sNode.u8AssociatedNodes].u8Humidity = 0;
            sCoordData.sNode.u8AssociatedNodes++;


            /* Assume association succeeded */
            u8AssocStatus = 0;

            /* Turn on LED to show node has assocaited */
			led_toggle(LED0);

        }
    }

    /* Create association response */
    sMlmeReqRsp.u8Type = MAC_MLME_RSP_ASSOCIATE;
    sMlmeReqRsp.u8ParamLength = sizeof(MAC_MlmeRspAssociate_s);
    sMlmeReqRsp.uParam.sRspAssociate.sDeviceAddr.u32H = psMlmeInd->uParam.sIndAssociate.sDeviceAddr.u32H;
    sMlmeReqRsp.uParam.sRspAssociate.sDeviceAddr.u32L = psMlmeInd->uParam.sIndAssociate.sDeviceAddr.u32L;
    sMlmeReqRsp.uParam.sRspAssociate.u16AssocShortAddr = u16ShortAddress;
    sMlmeReqRsp.uParam.sRspAssociate.u8Status = u8AssocStatus;
    sMlmeReqRsp.uParam.sRspAssociate.u8SecurityEnable = FALSE;

    /* Send association response. There is no confirmation for an association
       response, hence no need to check */
    vAppApiMlmeRequest(&sMlmeReqRsp, &sMlmeSyncCfm);
}

/****************************************************************************
 *
 * NAME: bStartCoordinator
 *
 * DESCRIPTION:
 * Starts the network by configuring the controller board to act as the PAN
 * coordinator.
 *
 * PARAMETERS:      Name            RW  Usage
 * None.
 *
 * RETURNS:
 * TRUE if network was started successfully otherwise FALSE
 *
 * NOTES:
 * None.
 ****************************************************************************/
PRIVATE void vStartCoordinator(void)
{
    /* Structures used to hold data for MLME request and response */
    MAC_MlmeReqRsp_s   sMlmeReqRsp;
    MAC_MlmeSyncCfm_s  sMlmeSyncCfm;

    /* Start Pan */
    sMlmeReqRsp.u8Type = MAC_MLME_REQ_START;
    sMlmeReqRsp.u8ParamLength = sizeof(MAC_MlmeReqStart_s);
    sMlmeReqRsp.uParam.sReqStart.u16PanId = PAN_ID;
    sMlmeReqRsp.uParam.sReqStart.u8Channel = sCoordData.sSystem.u8Channel;
    sMlmeReqRsp.uParam.sReqStart.u8BeaconOrder = 0x0f; /* No beacons */
    sMlmeReqRsp.uParam.sReqStart.u8SuperframeOrder = 0x0f;
    sMlmeReqRsp.uParam.sReqStart.u8PanCoordinator = TRUE;
    sMlmeReqRsp.uParam.sReqStart.u8BatteryLifeExt = FALSE;
    sMlmeReqRsp.uParam.sReqStart.u8Realignment = FALSE;
    sMlmeReqRsp.uParam.sReqStart.u8SecurityEnable = FALSE;

    vAppApiMlmeRequest(&sMlmeReqRsp, &sMlmeSyncCfm);
    led_off(LED_ALL);
    led_on(LED0);
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

    i16RxChar = i16Serial_RxChar();

    if (i16RxChar >= 0)
    {
        // set max power level
        eAppApiPlmeSet(PHY_PIB_ATTR_TX_POWER, 0xbf);
        /* Create frame transmission request */
        sMcpsReqRsp.u8Type = MAC_MCPS_REQ_DATA;
        sMcpsReqRsp.u8ParamLength = sizeof(MAC_McpsReqData_s);
        /* Set handle so we can match confirmation to request */
        sMcpsReqRsp.uParam.sReqData.u8Handle = 1;
        /* Use short address for source */
        sMcpsReqRsp.uParam.sReqData.sFrame.sSrcAddr.u8AddrMode = 2;
        sMcpsReqRsp.uParam.sReqData.sFrame.sSrcAddr.u16PanId = PAN_ID;
        sMcpsReqRsp.uParam.sReqData.sFrame.sSrcAddr.uAddr.u16Short = COORD_ADDR;
        /* Use short address for destination */
        sMcpsReqRsp.uParam.sReqData.sFrame.sDstAddr.u8AddrMode = 2;
        sMcpsReqRsp.uParam.sReqData.sFrame.sDstAddr.u16PanId = PAN_ID;
        sMcpsReqRsp.uParam.sReqData.sFrame.sDstAddr.uAddr.u16Short = 0xFFFF; //sCoordData.sNode.asAssocNodes[0].u16ShortAddr;
        /* Frame requires ack but not security, indirect transmit or GTS */
        sMcpsReqRsp.uParam.sReqData.sFrame.u8TxOptions = MAC_TX_OPTION_ACK;

        pu8Payload = sMcpsReqRsp.uParam.sReqData.sFrame.au8Sdu;

        //pu8Payload[i++] = u8TxFrameHandle++;
        pu8Payload[i++] = FtTML_GetNextSendSeq(sMcpsReqRsp.uParam.sReqData.sFrame.sDstAddr.uAddr.u16Short);
        pu8Payload[i++] = (uint8)i16RxChar;

        while ((i16RxChar >= 0) && (i < MAX_DATA_PER_FRAME))
        {
            i16RxChar = i16Serial_RxChar();

            if (i16RxChar >= 0)
            {
            	/* Set payload data */
	            pu8Payload[i++] = (uint8)i16RxChar;
	        }
        }

        /* Set frame length */
        sMcpsReqRsp.uParam.sReqData.sFrame.u8SduLength = i;
led_toggle(LED0);
        /* Request transmit */
        vAppApiMcpsRequest(&sMcpsReqRsp, &sMcpsSyncCfm);
    }
}

/****************************************************************************
 *
 * NAME: vStartEnergyScan
 *
 * DESCRIPTION:
 * Starts an enery sacn on the channels specified.
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
PRIVATE void vStartEnergyScan(void)
{
    /* Structures used to hold data for MLME request and response */
    MAC_MlmeReqRsp_s   sMlmeReqRsp;
    MAC_MlmeSyncCfm_s  sMlmeSyncCfm;

    /* Start energy detect scan */
    sMlmeReqRsp.u8Type = MAC_MLME_REQ_SCAN;
    sMlmeReqRsp.u8ParamLength = sizeof(MAC_MlmeReqStart_s);
    sMlmeReqRsp.uParam.sReqScan.u8ScanType = MAC_MLME_SCAN_TYPE_ENERGY_DETECT;
    sMlmeReqRsp.uParam.sReqScan.u32ScanChannels = SCAN_CHANNELS;
    sMlmeReqRsp.uParam.sReqScan.u8ScanDuration = ENERGY_SCAN_DURATION;

    vAppApiMlmeRequest(&sMlmeReqRsp, &sMlmeSyncCfm);
}

/****************************************************************************
 *
 * NAME: vHandleEnergyScanResponse
 *
 * DESCRIPTION:
 * Selects a channel with low enery content for use by the wireless UART.
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
PRIVATE void vHandleEnergyScanResponse(MAC_MlmeDcfmInd_s *psMlmeInd)
{
    uint8 i = 0;
    uint8 u8MinEnergy;

	u8MinEnergy = (psMlmeInd->uParam.sDcfmScan.uList.au8EnergyDetect[0]) ;

	/* Search list to find quietest channel */
    while (i < psMlmeInd->uParam.sDcfmScan.u8ResultListSize)
    {
        if ((psMlmeInd->uParam.sDcfmScan.uList.au8EnergyDetect[i]) < u8MinEnergy)
        {
			u8MinEnergy = (psMlmeInd->uParam.sDcfmScan.uList.au8EnergyDetect[i]);
			sCoordData.sSystem.u8Channel = i + CHANNEL_MIN;
		}
		i++;
    }
}

PRIVATE void vDrawUI(void)
{
    uint8 aTmpData[20];
    uint8 ind;

    vSerial_TxString("\f");
    vSerial_TxString("FontalTech EVK (Ft-6250) Base Board Lab7 Demo Program 0v1\r\n");
    vSerial_TxString("=========================================================\r\n");

    Num2Str(aTmpData,sCoordData.sSystem.u8Channel);
    vSerial_TxString("My Cannel: \t");
    vSerial_TxString(aTmpData);
    Num2Str(aTmpData,sCoordData.sNode.u8AssociatedNodes);
    vSerial_TxString("\tNum of Nodes: \t");
    vSerial_TxString(aTmpData);
    vSerial_TxString("\r\n");
    vSerial_TxString("\t(Press SPACE BAR to refresh)\r\n");
    vSerial_TxString("--------------------------------------------------------------------------------\r\n");
    vSerial_TxString("Node Addr\tRecv. Count\tPower\tLQI\tTemp\tHumidity\tDistance\r\n");
    vSerial_TxString("--------------------------------------------------------------------------------\r\n");

    vUART_TxCharISR();
    for (ind= 0; ind < sCoordData.sNode.u8AssociatedNodes; ind++)
    {
        Num2Str(aTmpData,sCoordData.sNode.asAssocNodes[ind].u16ShortAddr);
        vSerial_TxString(aTmpData);
        vSerial_TxString(":\t\t");

        Num2Str(aTmpData,sCoordData.sNode.asAssocNodes[ind].u16RecvCnt);
        vSerial_TxString(aTmpData);
        vSerial_TxString("\t\t");

        Num2Str(aTmpData,0x0000 | sCoordData.sNode.asAssocNodes[ind].u8Power);
        vSerial_TxString(aTmpData);
        vSerial_TxString("\t");

        Num2Str(aTmpData,0x0000 | sCoordData.sNode.asAssocNodes[ind].u8LastLQI);
        vSerial_TxString(aTmpData);
		vSerial_TxString("\t");
		
		//Temp
		Num2Str(aTmpData,0x0000 | sCoordData.sNode.asAssocNodes[ind].u8Temp);
        vSerial_TxString(aTmpData);
		vSerial_TxString("\t");
		
		//Humidity
		Num2Str(aTmpData,0x0000 | sCoordData.sNode.asAssocNodes[ind].u8Humidity);
        vSerial_TxString(aTmpData);
		vSerial_TxString("\t");
		
		//Distance
		/*Num2Str(aTmpData,0x0000 | sCoordData.sNode.asAssocNodes[ind].aChar);
        vSerial_TxString(aTmpData);*/
        vSerial_TxString("\r\n");
        vUART_TxCharISR();
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
/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
