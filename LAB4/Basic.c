/**************************************************************************//**
 * @file     main.c
 * @version  V3.0
 * $Revision: 2 $
 * $Date: 2016/02/29 LoRa Node v1.2
 *		Add server.
 *		for experiment, CMD SF change, add BW change
 * @brief
 *           LoRA Channel activity detection(CAD) test.
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC100Series.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "timers.h"
#include "semphr.h"

#define PLL_CLOCK           50000000

static void vTaskPrintParkingSpaces(void* pvParameters);
static void CountParkingSpaces(void* pvParameters);

SemaphoreHandle_t parkingSemaphore=NULL;
int parking_spaces=2;

void vStartThreadTasks( void );

/* Function prototype declaration */
void SYS_Init(void);
void UART0_Init(void);


/*****************************************
*
*        DIO Mapping interrupt
*
*****************************************/

void GPIO_Init()
{
		//INIT LEDSs
		GPIO_SetMode(PC, BIT12, GPIO_PMD_OUTPUT);
		GPIO_SetMode(PC, BIT13, GPIO_PMD_OUTPUT);
		GPIO_SetMode(PC, BIT14, GPIO_PMD_OUTPUT);
		GPIO_SetMode(PC, BIT15, GPIO_PMD_OUTPUT);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184 MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);
	
		/* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));
    
    /* Enable external 12 MHz XTAL */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock rate as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);
		
    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);		

    /* Select HXT as the clock source of UART0 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

    /* Enable SPI1 peripheral clock */
    CLK_EnableModuleClock(SPI0_MODULE);          //change SPI0_MODULE

    /* Select HCLK as the clock source of SPI1 */
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL1_SPI0_S_HCLK, MODULE_NoMsk);

		/* Enable Timer 0 module clock */
		CLK_EnableModuleClock(TMR0_MODULE);
		
		/* Select Timer 0 module clock source */
		CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_HXT, NULL);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
    //SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP = SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;

//		SYS->GPC_MFP = SYS_GPC_MFP_PC8_SPI1_SS0 | SYS_GPC_MFP_PC9_SPI1_CLK | SYS_GPC_MFP_PC10_SPI1_MISO0 | SYS_GPC_MFP_PC11_SPI1_MOSI0;
//		SYS->ALT_MFP = SYS_ALT_MFP_PC8_SPI1_SS0;

		
//		SYS->GPA_MFP = SYS_GPA_MFP_PA1_GPIO | SYS_GPA_MFP_PA2_GPIO | SYS_GPA_MFP_PA3_GPIO;
		
    /* Setup SPI0 multi-function pins */
    SYS->GPC_MFP = SYS_GPC_MFP_PC0_SPI0_SS0 | SYS_GPC_MFP_PC1_SPI0_CLK | SYS_GPC_MFP_PC2_SPI0_MISO0 | SYS_GPC_MFP_PC3_SPI0_MOSI0;
    SYS->ALT_MFP = SYS_ALT_MFP_PC0_SPI0_SS0 | SYS_ALT_MFP_PC1_SPI0_CLK | SYS_ALT_MFP_PC2_SPI0_MISO0 | SYS_ALT_MFP_PC3_SPI0_MOSI0;
}

void UART0_Init(void)
{
		SYS_ResetModule(UART0_RST);
	
    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);
}

/**********************************************/
/*                Main function               */
/*                LoRa SPI sample             */
/*        Bit length of a transaction: 8      */
/* SPI1, SS0(PC.8), CLK(PC.9), MISO0(PC.10)   */
/*                MOSI0(PC.11)                */
/**********************************************/



void Debounce(){
		/*  Configure PE.5 as Quasi-bidirection mode and enable interrupt by falling edge trigger */
		GPIO_SetMode(PA, BIT0, GPIO_PMD_QUASI);
		GPIO_SetMode(PA, BIT1, GPIO_PMD_QUASI);
		GPIO_SetMode(PA, BIT2, GPIO_PMD_QUASI);
		GPIO_SetMode(PA, BIT3, GPIO_PMD_QUASI);
		GPIO_SetMode(PA, BIT4, GPIO_PMD_QUASI);
		GPIO_SetMode(PA, BIT5, GPIO_PMD_QUASI);
		
		GPIO_EnableInt(PA, 0, GPIO_INT_FALLING);
		GPIO_EnableInt(PA, 1, GPIO_INT_FALLING);
		GPIO_EnableInt(PA, 2, GPIO_INT_FALLING);
		//NVIC_EnableIRQ(GPAB_IRQn);
		
		PA3=0; PA4=1; PA5=1;
		/* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 128 clocks of LIRC clock */
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCLKSRC_LIRC, GPIO_DBCLKSEL_128);
    GPIO_ENABLE_DEBOUNCE(PA, BIT0 | BIT1 | BIT2);
}

int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init system, IP clock and multi-function I/O. */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();
		GPIO_Init();  // LED initial
		UART0_Init();

		Debounce();	
	
		printf("----------------LAB4-Basic----------------\r\n");
	
		// Create a semaphore for parking spaces with an initial count of 5
    parkingSemaphore = xSemaphoreCreateCounting(5, 2); // 5 spaces, 2 spaces leave
	
		vStartThreadTasks();
		vTaskStartScheduler();

		while(1);

}


void vStartThreadTasks( void )
{
		xTaskCreate(vTaskPrintParkingSpaces, "vTaskMsgPro1",128,NULL,1,( xTaskHandle * ) NULL );  //pxTaskCode , pcName,usStackDepth,pvParameters,uxPriority,pxCreatedTask
		xTaskCreate(CountParkingSpaces, "CountParkingSpaces",128,NULL,1,( xTaskHandle * ) NULL );
	
}

// Function to simulate "get in" button press
void getInButtonPressed() {
		printf("get in\n");
    if(xSemaphoreTake(parkingSemaphore, 0) == pdTRUE) {
        printf("Car in!\n");
				parking_spaces--;
    } else {
        printf("No Parking Space!\n");
				PC12=0;
    }
}

// Function to simulate "get out" button press
void getOutButtonPressed() {
		printf("get out\n");
		if(parking_spaces==5){
			printf("No Car inside!\n");
		}else{
			printf("Car out!\n");
			xSemaphoreGive(parkingSemaphore);
			parking_spaces++;
		}
}

static void CountParkingSpaces(void* pvParameters)
{
			while(1)
			{
						//Task1 count per second (use vTaskDelay)
						printf("%d parking spaces~\n",parking_spaces);
						vTaskDelay(5000);
			}
}

static void vTaskPrintParkingSpaces(void* pvParameters)
{
		while(1)
		{
				if(GPIO_GET_INT_FLAG(PA, BIT2))	//key1
				{	
							//printf("i am key1~~~\n");
							GPIO_CLR_INT_FLAG(PA, BIT2);
							getInButtonPressed();
				}
				if(GPIO_GET_INT_FLAG(PA, BIT1)) 	//key2
				{
						GPIO_CLR_INT_FLAG(PA, BIT1);
						getOutButtonPressed();
				}
				if(parking_spaces==0){
					PC12=0;
				}else{
					PC12=1;
				}
		}
}


/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
