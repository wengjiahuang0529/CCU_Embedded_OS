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
#include "string.h"
#include "NUC100Series.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"

#define PLL_CLOCK           50000000

/*------------------LAB2------------------*/
#define COMMAND_QUEUE_LENGTH 50
#define RXBUFSIZE 1024 
uint8_t g_u8RecData[RXBUFSIZE]  = {0};
volatile uint32_t g_u32comRbytes = 0;
volatile uint32_t g_u32comRhead  = 0;
volatile uint32_t g_u32comRtail  = 0;
volatile int32_t g_bWait         = TRUE;
/*------------------LAB2------------------*/

static void TaskA(void* pvParameters);
static void TaskB(void* pvParameters);


void vStartThreadTasks( void );


/* Function prototype declaration */
void SYS_Init(void);
void UART0_Init(void);

/*------------------LAB2------------------*/
// Define command structure
typedef struct {
    char command[50]; // Store the command string

} Command_t;


QueueHandle_t xCommandQueueA; // Task A puts commands here
QueueHandle_t xCommandQueueB; // Task B reads and executes commands
/*------------------LAB2------------------*/

/*****************************************
*
*        DIO Mapping interrupt
*
*****************************************/

void GPIO_Init()
{
		//INIT LEDS
		GPIO_SetMode(PC, BIT12, GPIO_PMD_OUTPUT);
		GPIO_SetMode(PC, BIT13, GPIO_PMD_OUTPUT);
		GPIO_SetMode(PC, BIT14, GPIO_PMD_OUTPUT);
		GPIO_SetMode(PC, BIT15, GPIO_PMD_OUTPUT);
	
		//INIT RGB LED
		GPIO_SetMode(PA,BIT12,GPIO_PMD_OUTPUT);
		GPIO_SetMode(PA,BIT13,GPIO_PMD_OUTPUT);
		GPIO_SetMode(PA,BIT14,GPIO_PMD_OUTPUT);
	
		//Buzzer
		GPIO_SetMode(PB,BIT11,GPIO_PMD_OUTPUT);
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
/**********************************************/
void UART_FunctionTest(void);

int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init system, IP clock and multi-function I/O. */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();
	
		GPIO_Init();  // LEDs and RGB LEDs initial
		UART0_Init();

		xCommandQueueA = xQueueCreate(COMMAND_QUEUE_LENGTH, sizeof(Command_t));  //maximum number of items , each item size in bytes
		xCommandQueueB = xQueueCreate(COMMAND_QUEUE_LENGTH, sizeof(Command_t));
	
		printf("\n\n---LAB2 Basic---\r\n");
	
		//UART_FunctionTest(); 
	
		vStartThreadTasks();
		vTaskStartScheduler();

		while(1);

}

void UART02_IRQHandler(void) 
{
    uint8_t u8InChar = 0xFF;
    uint32_t u32IntSts = UART0->ISR; 
		uint8_t *txt=g_u8RecData; 
	
		char RGB_LED[6][20];
		strcpy(RGB_LED[0],"blue on");
		strcpy(RGB_LED[1],"blue off");
		strcpy(RGB_LED[2],"green on");
		strcpy(RGB_LED[3],"green off");
		strcpy(RGB_LED[4],"red on");
		strcpy(RGB_LED[5],"red off");
	
    if(u32IntSts & UART_ISR_RDA_INT_Msk) //
    {
				//if(!g_u8RecData[0]) printf("Input:");
				u8InChar = UART_READ(UART0);
			  if(u8InChar == '0') //input 0 to end.
          {
                g_bWait = FALSE;
          }
				if(u8InChar==0X0D){
						g_u8RecData[g_u32comRtail]='\0'; //end of string
						printf("Input:%s\n",(char*)txt);
				
						if(strcmp(RGB_LED[0],(char*)txt)==0)
						{
								PA12=0;
						}
						if(strcmp(RGB_LED[1],(char*)txt)==0)
						{
								PA12=1;
						}
						if(strcmp(RGB_LED[2],(char*)txt)==0)
						{
								PA13=0;
						}
						if(strcmp(RGB_LED[3],(char*)txt)==0)
						{
								PA13=1;
						}
						if(strcmp(RGB_LED[4],(char*)txt)==0)
						{
								PA14=0;
						}
						if(strcmp(RGB_LED[5],(char*)txt)==0)
						{
								PA14=1;
						}
						//reset
						g_u8RecData[0]=0;
						g_u32comRtail=0;
						g_u32comRbytes=0;
				}					
			else if(g_u32comRbytes < RXBUFSIZE)
        {
            g_u8RecData[g_u32comRtail] = u8InChar; //put the char in g_u8RecData
            g_u32comRtail = (g_u32comRtail == (RXBUFSIZE - 1)) ? 0 : (g_u32comRtail + 1); //check oversize
            g_u32comRbytes++;
        }
		}
}


void UART_FunctionTest()
{
		printf("LAB2-UART\n");
    /* Enable Interrupt and install the call back function */
    UART_EnableInt(UART0, ( UART_IER_RDA_IEN_Msk | UART_IER_THRE_IEN_Msk | UART_IER_TOUT_IEN_Msk));
    while(g_bWait);
	
    /* Disable Interrupt */
    UART_DisableInt(UART0, (UART_IER_RDA_IEN_Msk | UART_IER_THRE_IEN_Msk | UART_IER_TOUT_IEN_Msk));
    g_bWait = TRUE;
    printf("\nUART Sample Demo End.\n");
}





void vStartThreadTasks( void )
{
		xTaskCreate(TaskA, "TaskA",128,NULL,1,( xTaskHandle * ) NULL );  //pxTaskCode , pcName,usStackDepth,pvParameters,uxPriority,pxCreatedTask
		xTaskCreate(TaskB, "TaskB",128,NULL,1,( xTaskHandle * ) NULL );
}

static void TaskA(void* pvParameters)
{
		Command_t cmd;
	
		//size_t size = sizeof(Command_t);
		//printf("Size of Command_t structure: %zu bytes\n", size);
	
		while(1)
		{
				char input[50];
				printf("\n\nInput Command:");
				scanf("%s",input);
				printf("%s\n",input);
			
				strcpy(cmd.command, input);
			
			  // Send the command to Task B
        xQueueSend(xCommandQueueA, &cmd, portMAX_DELAY);  	//queue handle //a pointer to the item //waiting time

				vTaskDelay(5);
		}
}

static void TaskB(void* pvParameters)
{
		Command_t cmd;
		while(1)
		{
				if (xQueueReceive(xCommandQueueA, &cmd, portMAX_DELAY) == pdPASS)
        {
					  // Process the command
            if (strcmp(cmd.command, "LED0") == 0)
            {
							  // Turn on LED0
                printf("LEDs turned ON\n");
								PC12=0;
								PC13=0;
								PC14=0;
								PC15=0;
						}
						else if(strcmp(cmd.command, "LED1") == 0)
						{
							  printf("LED turned OFF\n");
								PC12=1;
								PC13=1;
								PC14=1;
								PC15=1;
						}
						else if(strcmp(cmd.command,"BUZ0")==0)
						{
								printf("buzzer turned ON\n");
								PB11=0;
						}
						else if(strcmp(cmd.command,"BUZ1")==0)
						{
								printf("buzzer turned ON\n");
								PB11=1;
						}
						
						else if(strcmp(cmd.command,"BLU0")==0)
						{
								printf("Change the color of RGB LED to Blue\n");
								PA12=0;
						}
						else if(strcmp(cmd.command,"BLU1")==0)
						{
								printf("Turn off RGB LED of Blue\n");
								PA12=1;
						}
						
						else if(strcmp(cmd.command,"GRE0")==0)
						{
								printf("Change the color of RGB LED to Green\n");
								PA13=0;
						}
						else if(strcmp(cmd.command,"GRE1")==0)
						{
								printf("Turn off RGB LED of Green\n");
								PA13=1;
						}
						
						else if(strcmp(cmd.command,"RED0")==0)
						{
								printf("Change the color of RGB LED to Red\n");
								PA14=0;
						}
						else if(strcmp(cmd.command,"RED1")==0)
						{
								printf("Turn off RGB LED of Red\n");
								PA14=1;
						}
						else{
								printf("Wrong command!");
						}
				}
		}
}


/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
