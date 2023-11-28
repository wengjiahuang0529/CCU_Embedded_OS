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


#define PLL_CLOCK           50000000
#define Timer1_period 			pdMS_TO_TICKS( 5000 )
#define Timer2_period 			pdMS_TO_TICKS( 10000 )

volatile uint32_t g_au32TMRINTCount[2] = {0};
volatile int key;

static void vTaskMsgPro1(void* pvParameters);
static void vTaskMsgPro2(void* pvParameters);
static void vTaskCountTime(void* pvParameters);

static TimerHandle_t xTimer1 = NULL; 
static TimerHandle_t xTimer2 = NULL; 
int pressedkey1=0;
int pressed=0;

int cnt=0;


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
void Task2_vTimerCallback(){
		printf("Stop timer 2\n");
		//printf("%d second. I'm Timer. OhYa!\n",cnt);
		xTimerStop(xTimer2,portMAX_DELAY);
		pressed=0;
}

void Task1_vTimerCallback(){
		printf("Stop timer 1\n");
		xTimerStop(xTimer1,portMAX_DELAY);
		pressedkey1=0;
}

void GPAB_IRQHandler(void){
    /* To check if PA interrupt occurred */
		if(GPIO_GET_INT_FLAG(PA, BIT0)) 				//key3: clear to zero
		{
        GPIO_CLR_INT_FLAG(PA, BIT0);
				printf("i am key3\n");
    }
		else if(GPIO_GET_INT_FLAG(PA, BIT1)) 	//key2
		{
				GPIO_CLR_INT_FLAG(PA, BIT1);
				printf("i am key2\n");
		}
		else if(GPIO_GET_INT_FLAG(PA, BIT2))	//key1
		{
				GPIO_CLR_INT_FLAG(PA, BIT2);
				printf("i am key1\n");
		}
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
		/* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCLKSRC_LIRC, GPIO_DBCLKSEL_128);
    GPIO_ENABLE_DEBOUNCE(PA, BIT0 | BIT1 | BIT2);
		
		printf("----------------LAB3----------------\r\n");
		vStartThreadTasks();
		vTaskStartScheduler();

		while(1);

}


void vStartThreadTasks( void )
{
		xTaskCreate(vTaskMsgPro1, "vTaskMsgPro1",128,NULL,1,( xTaskHandle * ) NULL );  //pxTaskCode , pcName,usStackDepth,pvParameters,uxPriority,pxCreatedTask
		xTaskCreate(vTaskMsgPro2, "vTaskMsgPro2",128,NULL,1,( xTaskHandle * ) NULL );
		xTaskCreate(vTaskCountTime, "vTaskCountTime",128,NULL,1,( xTaskHandle * ) NULL );
}

static void vTaskCountTime(void* pvParameters){
			while(1)
			{
						//Task1 count per second (use vTaskDelay)
						printf("%d second. I'm Delay\n",cnt);
						cnt++;
						vTaskDelay(1000);
			}
}

static void vTaskMsgPro1(void* pvParameters)
{
		while(1){
					if(GPIO_GET_INT_FLAG(PA, BIT2))	//key1
					{					
						//printf("i am key1~~~\n");
						GPIO_CLR_INT_FLAG(PA, BIT2);

						if(xTimer1==NULL){
								xTimer1 = xTimerCreate("Timer1", pdMS_TO_TICKS(5000), pdTRUE, (void *)0, Task1_vTimerCallback);
						}
						
						if(pressedkey1==0){
								pressedkey1=1;	
								//xTimerChangePeriod(xTimer1, pdMS_TO_TICKS(5000),0);
								
							 if(xTimer1!=NULL){
									if(xTimerStart(xTimer1,0)==pdPASS){
											printf("Timer1 start\n");
									}else{
											printf("Failed to start timer1\n");
									}
							 }
						}else{
								if(pressedkey1){
									//stop timer;
									printf("You pressed to stop Timer1 !!!\n");
									xTimerStop(xTimer1,portMAX_DELAY);
									pressedkey1=0;
								}
							}					
					}
			}
}

static void vTaskMsgPro2(void* pvParameters)
{
		while(1)
		{
				if(GPIO_GET_INT_FLAG(PA, BIT1)) 	//key2
				{
						//printf("i am key2~~~\n");
						GPIO_CLR_INT_FLAG(PA, BIT1);
						
						if(xTimer2==NULL){
								xTimer2 = xTimerCreate("Timer2", pdMS_TO_TICKS(10000), pdTRUE, (void *)0, Task2_vTimerCallback);
						}
					
						if(pressed==0){
							pressed=1;
							//xTimerChangePeriod(xTimer2, pdMS_TO_TICKS(10000),0);
							 if(xTimer2!=NULL){
									if(xTimerStart(xTimer2,0)==pdPASS){
											printf("Timer2 start\n");
									}else{
											printf("Failed to start timer2\n");
									}
							 }
						}else{
								if(pressed){
								//stop timer;
								printf("You pressed to stop Timer2 !!!\n");
								xTimerStop(xTimer2,portMAX_DELAY);
								pressed=0;
							}
						}
				}
		}
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
