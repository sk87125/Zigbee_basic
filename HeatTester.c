#include <jendefs.h>
#include "AppHardwareApi.h"
#include <AppApi.h>
#include <Utilities.h>
#include <FontalLED.h>
#include <FontalButton.h>
//#include <FontalWtd.h>
#include <HtsDriver.h>

/*********************************************************************************************
This project use to test:
1) DIO Interrupt use callback function
2) check sysctrlregistercallback with other(like UART) callback have any problem or not
3) chekc ticktimer callback event control can start/stop manuelly



*********************************************************************************************/
// function pre define
PUBLIC void AppColdStart(void);
PUBLIC void AppWarmStart(void);
PRIVATE void InitSystem(void);
PRIVATE void vUartISR(uint32 u32DeviceId, uint32 u32ItemBitmap);
PRIVATE void vSystemISR(uint32 u32DeviceId, uint32 u32ItemBitmap);
PUBLIC void vUartPrint(uint8 u8UartPort, char * pString);
PRIVATE void vTickTimerISR(uint32 u32DeviceId, uint32 u32ItemBitmap);
PUBLIC void vCast_NumToString(uint32 u32Data, char *pcString);
PUBLIC void Num2Str(char *pcString, uint16 u16Data);


uint8 bFlag;
uint32 m_u32TickCnt = 0;
// start functions

PUBLIC void AppColdStart(void)
{
    bFlag = FALSE;
	InitSystem();
}

PUBLIC void AppWarmStart(void)
{
	InitSystem();
}

PRIVATE void InitSystem(void)
{
	// set the URAT values
	uint8 u8UartPort = E_AHI_UART_0;
	uint8 u8BaudRate = E_AHI_UART_RATE_38400;

	// Jennic Hardware init
	u32AHI_Init();

	// init App API to use callback function
	u32AppApiInit(NULL,NULL,NULL,NULL,NULL,NULL);

	// ==============start to do UART's Job ===================
	vAHI_UartEnable(u8UartPort);

    	vAHI_UartReset(u8UartPort, TRUE, TRUE);
    	vAHI_UartReset(u8UartPort, FALSE, FALSE);

    	vAHI_UartSetClockDivisor(u8UartPort, u8BaudRate);
    	vAHI_UartSetControl(u8UartPort, FALSE, FALSE, E_AHI_UART_WORD_LEN_8, TRUE, FALSE);


	// ==============end of UART's Job =====================

	// ============== init button and led's DIO =================
	led_init();
    	btn_init();
        //wtd_init();
        // ===================================================

	// =======start  == register callback functions =================

	vAHI_TickTimerInit(vTickTimerISR);  // for tick timer only
	vAHI_SysCtrlRegisterCallback(vSystemISR);	// for other events(use to test DIO event)

	if(u8UartPort == E_AHI_UART_0){
		vAHI_Uart0RegisterCallback((void*)vUartISR);
	} else {
		vAHI_Uart1RegisterCallback((void*)vUartISR);
	}
	// =======end   == register callback functions ==================


	/* Enable TX Fifo empty and Rx data interrupts */
	vAHI_UartSetInterrupt(u8UartPort, FALSE, FALSE, FALSE, TRUE, E_AHI_UART_FIFO_LEVEL_1);

	// ********* start the ticktimer when get the UART event  *****************

	vAHI_TickTimerConfigure(E_AHI_TICK_TIMER_RESTART);
	vAHI_TickTimerWrite(0);  // reset the "tick" before startup the timer
	vAHI_TickTimerInterval((16*1000*1000));  // 10m  (16MHz)
	vAHI_TickTimerIntEnable(TRUE);
	// ***************************************************************
	vHTSreset();
	while (1)
	{

	}
}

// uart callback================
PRIVATE void vUartISR(uint32 u32DeviceId, uint32 u32ItemBitmap)
{
	uint8 u8InterruptStatus=0;
	uint8 u8UartPort = 0;
    	uint8 u8RcvChar = 0x00;

	// only do something when get the RX event from UART0
	if ((u32DeviceId == E_AHI_DEVICE_UART0) && (u32ItemBitmap = E_AHI_UART_INT_RXDATA))
	{
		u8UartPort = E_AHI_UART_0;
		u8InterruptStatus = u8AHI_UartReadInterruptStatus(u32DeviceId);
		if (((u8InterruptStatus & 1) ==0) )
		{
			u8RcvChar = u8AHI_UartReadData(u8UartPort);
			vUartPrint(0,"\f");
			vUartPrint(0,"FontalTech EVK (FT-6251) Senser Board Demo Program\r\n");
        		vUartPrint(0,"======================================================\r\n");
			vUartPrint(0,"Temperature(C) \tHumidity(%)\r\n");
			vUartPrint(0,"======================================================\r\n");

		}

	}/*
	else if ((u32DeviceId == E_AHI_DEVICE_UART1) && (u32ItemBitmap = E_AHI_UART_INT_RXDATA))
	{
		u8UartPort = E_AHI_UART_1;
		u8InterruptStatus = u8AHI_UartReadInterruptStatus(u32DeviceId);
		if (((u8InterruptStatus & 1) ==0) )
		{
			u8RcvChar = u8AHI_UartReadData(u8UartPort);
		}
	}*/
}



// test dio event mix with other event
PRIVATE void vSystemISR(uint32 u32DeviceId, uint32 u32ItemBitmap)
{
	switch (u32DeviceId)
	{
	case E_AHI_DEVICE_SYSCTRL:
		break;
	case E_AHI_DEVICE_PHYCTRL:
		vUartPrint(E_AHI_UART_0,"E_AHI_DEVICE_PHYCTRL\r\n");
		break;
	case E_AHI_DEVICE_INTPER:
		vUartPrint(E_AHI_UART_0,"E_AHI_DEVICE_INTPER\r\n");
		break;
	case E_AHI_DEVICE_UART0:
		vUartPrint(E_AHI_UART_0,"E_AHI_DEVICE_INTPER\r\n");
		break;
	case E_AHI_DEVICE_TIMER1:
		vUartPrint(E_AHI_UART_0,"E_AHI_DEVICE_TIMER1\r\n");
		break;
	case E_AHI_DEVICE_TICK_TIMER:
		vUartPrint(E_AHI_UART_0,"E_AHI_DEVICE_TICK_TIMER\r\n");
		break;
	default:
		vUartPrint(E_AHI_UART_0,"UNKNOW_DEVICE\r\n");
		break;
	}

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

// ticktimer callback
PRIVATE void vTickTimerISR(uint32 u32DeviceId, uint32 u32ItemBitmap)
{
    uint16 u8Temp = 0;
    uint16 u8Humidity = 0;
    char aChar[5];

    switch (u32DeviceId)
    {
	case E_AHI_DEVICE_TIMER0:
	    vUartPrint(E_AHI_UART_0,"-E_AHI_DEVICE_TIMER0\r\n");
	    break;
	case E_AHI_DEVICE_TIMER1:
	    vUartPrint(E_AHI_UART_0,"-E_AHI_DEVICE_TIMER1\r\n");
	    break;
	case E_AHI_DEVICE_TICK_TIMER:
	    // send the watchdog reset to keep the system work
	    //wtd_ResetState();

	    if (bFlag== 0)
		vHTSstartReadTemp();
	    else if (bFlag== 1)
	    {
		u8Temp =  u16HTSreadTempResult();
		Num2Str( aChar,u8Temp);
		vUartPrint(0,"Temp ");
		vUartPrint(0,aChar);
		vUartPrint(0,"\t\t");
	    }
	    else if (bFlag == 2)
		vHTSstartReadHumidity();
	    else if (bFlag== 3)
	    {

		u8Humidity =  u16HTSreadHumidityResult();
		Num2Str( aChar,u8Humidity);
		vUartPrint(0,"Humidity ");
		vUartPrint(0,aChar);
		vUartPrint(0,"\r");
	    }
	    bFlag++;

	    if (bFlag >3)
		bFlag = 0;

	    break;
	default:
	    vUartPrint(E_AHI_UART_0,"-UNKNOW_DEVICE\r\n");
	    break;
    }
}

PUBLIC void vCast_NumToString(uint32 u32Data, char *pcString)
{
    int    i;
    uint8  u8Nybble;

    for (i = 28; i >= 0; i -= 4)
    {
        u8Nybble = (uint8)((u32Data >> i) & 0x0f);
        u8Nybble += 0x30;
        if (u8Nybble > 0x39)
            u8Nybble += 7;

        *pcString = u8Nybble;
        pcString++;
    }
    *pcString = 0;
}

PUBLIC void Num2Str(char *pcString, uint16 u16Data)
{
	int8 u8Ind = 4;
	do{
		pcString[u8Ind]= 0x30 +(u16Data%10);
		u16Data /=10;
		u8Ind--;
	}
	while (u8Ind>=0);
	pcString[5] = 0;
}

