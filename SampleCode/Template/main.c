/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "misc_config.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/

struct flag_32bit flag_PROJ_CTL;
#define FLAG_PROJ_TIMER_PERIOD_1000MS                 	(flag_PROJ_CTL.bit0)
#define FLAG_PROJ_SPI_Slave_RX                   		(flag_PROJ_CTL.bit1)
#define FLAG_PROJ_TIMER_PERIOD_10MS                 	(flag_PROJ_CTL.bit2)
#define FLAG_PROJ_REVERSE3                              (flag_PROJ_CTL.bit3)
#define FLAG_PROJ_REVERSE4                              (flag_PROJ_CTL.bit4)
#define FLAG_PROJ_REVERSE5                              (flag_PROJ_CTL.bit5)
#define FLAG_PROJ_REVERSE6                              (flag_PROJ_CTL.bit6)
#define FLAG_PROJ_REVERSE7                              (flag_PROJ_CTL.bit7)


/*_____ D E F I N I T I O N S ______________________________________________*/

volatile unsigned int counter_systick = 0;
volatile uint32_t counter_tick = 0;

#define SPI_DATA_NUM					                (1) 
#define SPI_TARGET_FREQ							        (2000000ul) //(8300000ul)

#define EMULATE_MASTER_SPI_PORT                         (USPI0)

#define TARGET_SPI_SLAVE_PORT                           (SPI0)

uint16_t g_SlaveRxBuffer[SPI_DATA_NUM]={0};

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

unsigned int get_systick(void)
{
	return (counter_systick);
}

void set_systick(unsigned int t)
{
	counter_systick = t;
}

void systick_counter(void)
{
	counter_systick++;
}

void SysTick_Handler(void)
{

    systick_counter();

    if (get_systick() >= 0xFFFFFFFF)
    {
        set_systick(0);      
    }

    // if ((get_systick() % 1000) == 0)
    // {
       
    // }

    #if defined (ENABLE_TICK_EVENT)
    TickCheckTickEvent();
    #endif    
}

void SysTick_delay(unsigned int delay)
{  
    
    unsigned int tickstart = get_systick(); 
    unsigned int wait = delay; 

    while((get_systick() - tickstart) < wait) 
    { 
    } 

}

void SysTick_enable(unsigned int ticks_per_second)
{
    set_systick(0);
    if (SysTick_Config(SystemCoreClock / ticks_per_second))
    {
        /* Setup SysTick Timer for 1 second interrupts  */
        dbg_printf("Set system tick error!!\n");
        while (1);
    }

    #if defined (ENABLE_TICK_EVENT)
    TickInitTickEvent();
    #endif
}

uint32_t get_tick(void)
{
	return (counter_tick);
}

void set_tick(uint32_t t)
{
	counter_tick = t;
}

void tick_counter(void)
{
	counter_tick++;
    if (get_tick() >= 60000)
    {
        set_tick(0);
    }
}

// void delay_ms(uint16_t ms)
// {
// 	TIMER_Delay(TIMER0, 1000*ms);
// }


void SPI0_IRQHandler(void)
{
    // /* Check RX EMPTY flag */
    // while(SPI_GET_RX_FIFO_EMPTY_FLAG(TARGET_SPI_SLAVE_PORT) == 0)
    // {
    //     /* Read RX FIFO */
    //     g_au32DestinationData[g_u32RxDataCount++] = SPI_READ_RX(TARGET_SPI_SLAVE_PORT);
    // }

    /* Check the RX FIFO time-out interrupt flag */
    // if(SPI_GetIntFlag(TARGET_SPI_SLAVE_PORT, SPI_FIFO_RXTO_INT_MASK))
    // {
    //     SPI_ClearIntFlag(TARGET_SPI_SLAVE_PORT, SPI_FIFO_RXTO_INT_MASK);

    //     /* If RX FIFO is not empty, read RX FIFO. */
    //     while(SPI_GET_RX_FIFO_EMPTY_FLAG(TARGET_SPI_SLAVE_PORT) == 0)
    //         g_au32DestinationData[g_u32RxDataCount++] = SPI_READ_RX(TARGET_SPI_SLAVE_PORT);

    // }

    if(SPI_GetIntFlag(TARGET_SPI_SLAVE_PORT, SPI_UNIT_INT_MASK))
    {
        SPI_ClearIntFlag(TARGET_SPI_SLAVE_PORT, SPI_UNIT_INT_MASK);

        // printf("SPI_UNIT_INT_MASK = 0x%2X\r\n" , SPI_READ_RX(TARGET_SPI_SLAVE_PORT) );
        g_SlaveRxBuffer[0] = SPI_READ_RX(TARGET_SPI_SLAVE_PORT);
        FLAG_PROJ_SPI_Slave_RX = 1;
    }    

}

void SPI_Slave_Init(void)
{
	SYS_ResetModule(SPI0_RST);

    SPI_Open(TARGET_SPI_SLAVE_PORT, SPI_SLAVE, SPI_MODE_0, 16, NULL);

	SPI_ClearRxFIFO(TARGET_SPI_SLAVE_PORT);

	SPI_SetFIFO(TARGET_SPI_SLAVE_PORT , 2 , 2);

	// SPI_EnableInt(TARGET_SPI_SLAVE_PORT, SPI_UNIT_INT_MASK | SPI_FIFO_RXTO_INT_MASK );
    SPI_EnableInt(TARGET_SPI_SLAVE_PORT, SPI_UNIT_INT_MASK );
    
    NVIC_EnableIRQ(SPI0_IRQn);
}

void USPI_Master_Tx(uint8_t* u8bffer , uint16_t len)
{
	uint16_t i = 0;

	USPI_SET_SS_LOW(EMULATE_MASTER_SPI_PORT);

    while (i < len)
    {
        USPI_WRITE_TX(EMULATE_MASTER_SPI_PORT, u8bffer[i++]);
		while(USPI_IS_BUSY(EMULATE_MASTER_SPI_PORT));
    }

    // USPI_SET_SS_HIGH(EMULATE_MASTER_SPI_PORT);	
}

void USPI_Master_Output(void)
{
    static uint16_t count = 0;
    uint8_t buffer[2] = {0};
    uint16_t data_output = 0;

    buffer[0] = 0x5A;
    buffer[1] = 0x00 + count;
    data_output = (buffer[0] << 8) | buffer[1] ;

	USPI_SET_SS_LOW(EMULATE_MASTER_SPI_PORT);

    USPI_WRITE_TX(EMULATE_MASTER_SPI_PORT, data_output);
    while(USPI_IS_BUSY(EMULATE_MASTER_SPI_PORT));

    count++;

    // USPI_SET_SS_HIGH(EMULATE_MASTER_SPI_PORT);    
}

// emulate MASTER send data , 16 clock , rising , clock freq:8.3Mhz
void USPI_Master_Init(void)
{
    USPI_Open(EMULATE_MASTER_SPI_PORT, USPI_MASTER, USPI_MODE_0, 16, SPI_TARGET_FREQ);

    USPI_DisableAutoSS(EMULATE_MASTER_SPI_PORT);

}

//
// check_reset_source
//
uint8_t check_reset_source(void)
{
    uint32_t src = SYS_GetResetSrc();

    SYS->RSTSTS |= 0x1FF;
    printf("Reset Source <0x%08X>\r\n", src);

    #if 1   //DEBUG , list reset source
    if (src & BIT0)
    {
        printf("0)POR Reset Flag\r\n");       
    }
    if (src & BIT1)
    {
        printf("1)NRESET Pin Reset Flag\r\n");       
    }
    if (src & BIT2)
    {
        printf("2)WDT Reset Flag\r\n");       
    }
    if (src & BIT3)
    {
        printf("3)LVR Reset Flag\r\n");       
    }
    if (src & BIT4)
    {
        printf("4)BOD Reset Flag\r\n");       
    }
    if (src & BIT5)
    {
        printf("5)System Reset Flag \r\n");       
    }
    if (src & BIT6)
    {
        printf("6)Reserved.\r\n");       
    }
    if (src & BIT7)
    {
        printf("7)CPU Reset Flag\r\n");       
    }
    if (src & BIT8)
    {
        printf("8)CPU Lockup Reset Flag\r\n");       
    }
    #endif
    
    if (src & SYS_RSTSTS_PORF_Msk) {
        SYS_ClearResetSrc(SYS_RSTSTS_PORF_Msk);
        
        printf("power on from POR\r\n");
        return FALSE;
    } 
    else if (src & SYS_RSTSTS_CPURF_Msk)
    {
        SYS_ClearResetSrc(SYS_RSTSTS_CPURF_Msk);

        printf("power on from CPU reset\r\n");
        return FALSE;         
    }    
    else if (src & SYS_RSTSTS_PINRF_Msk)
    {
        SYS_ClearResetSrc(SYS_RSTSTS_PINRF_Msk);
        
        printf("power on from nRESET pin\r\n");
        return FALSE;
    }
    
    printf("power on from unhandle reset source\r\n");
    return FALSE;
}

void TMR1_IRQHandler(void)
{
	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
		tick_counter();

		if ((get_tick() % 1000) == 0)
		{
            FLAG_PROJ_TIMER_PERIOD_1000MS = 1;//set_flag(flag_timer_period_1000ms ,ENABLE);
		}

		if ((get_tick() % 50) == 0)
		{

		}	

		if ((get_tick() % 10) == 0)
		{
            FLAG_PROJ_TIMER_PERIOD_10MS = 1;
		}	
    }
}

void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void loop(void)
{
	// static uint32_t LOG1 = 0;
	// static uint32_t LOG2 = 0;

    if ((get_systick() % 1000) == 0)
    {
        // dbg_printf("%s(systick) : %4d\r\n",__FUNCTION__,LOG2++);    
    }

    if (FLAG_PROJ_TIMER_PERIOD_1000MS)//(is_flag_set(flag_timer_period_1000ms))
    {
        FLAG_PROJ_TIMER_PERIOD_1000MS = 0;//set_flag(flag_timer_period_1000ms ,DISABLE);

        // dbg_printf("%s(timer) : %4d\r\n",__FUNCTION__,LOG1++);
        PB14 ^= 1;        
    }

    if (FLAG_PROJ_TIMER_PERIOD_10MS)
    {
        FLAG_PROJ_TIMER_PERIOD_10MS = 0;
        USPI_Master_Output();
    }

	if (FLAG_PROJ_SPI_Slave_RX)
	{
		FLAG_PROJ_SPI_Slave_RX = 0;

		#if 1	// debug
		dbg_printf("SPI RX [S] : 0x%4X\r\n" , g_SlaveRxBuffer[0]);
		#endif
	}
}

void UARTx_Process(void)
{
	uint8_t res = 0;
	res = UART_READ(UART0);

	if (res > 0x7F)
	{
		dbg_printf("invalid command\r\n");
	}
	else
	{
		dbg_printf("press : %c\r\n" , res);
		switch(res)
		{
			case '1':
                USPI_Master_Output();
				break;

			case 'X':
			case 'x':
			case 'Z':
			case 'z':
                SYS_UnlockReg();
				// NVIC_SystemReset();	// Reset I/O and peripherals , only check BS(FMC_ISPCTL[1])
                // SYS_ResetCPU();     // Not reset I/O and peripherals
                SYS_ResetChip();    // Reset I/O and peripherals ,  BS(FMC_ISPCTL[1]) reload from CONFIG setting (CBS)	
				break;
		}
	}
}

void UART02_IRQHandler(void)
{
    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
			UARTx_Process();
        }
    }

    if(UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART0, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }	
}

void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
    UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
    NVIC_EnableIRQ(UART02_IRQn);
	
	#if (_debug_log_UART_ == 1)	//debug
	dbg_printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	dbg_printf("CLK_GetHCLKFreq : %8d\r\n",CLK_GetHCLKFreq());
	dbg_printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	dbg_printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	dbg_printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	dbg_printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	
	#endif	

    #if 0
    dbg_printf("FLAG_PROJ_TIMER_PERIOD_1000MS : 0x%2X\r\n",FLAG_PROJ_TIMER_PERIOD_1000MS);
    dbg_printf("FLAG_PROJ_REVERSE1 : 0x%2X\r\n",FLAG_PROJ_REVERSE1);
    dbg_printf("FLAG_PROJ_REVERSE2 : 0x%2X\r\n",FLAG_PROJ_REVERSE2);
    dbg_printf("FLAG_PROJ_REVERSE3 : 0x%2X\r\n",FLAG_PROJ_REVERSE3);
    dbg_printf("FLAG_PROJ_REVERSE4 : 0x%2X\r\n",FLAG_PROJ_REVERSE4);
    dbg_printf("FLAG_PROJ_REVERSE5 : 0x%2X\r\n",FLAG_PROJ_REVERSE5);
    dbg_printf("FLAG_PROJ_REVERSE6 : 0x%2X\r\n",FLAG_PROJ_REVERSE6);
    dbg_printf("FLAG_PROJ_REVERSE7 : 0x%2X\r\n",FLAG_PROJ_REVERSE7);
    #endif

}

void GPIO_Init (void)
{
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB14MFP_Msk)) | (SYS_GPB_MFPH_PB14MFP_GPIO);
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB15MFP_Msk)) | (SYS_GPB_MFPH_PB15MFP_GPIO);

    GPIO_SetMode(PB, BIT14, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PB, BIT15, GPIO_MODE_OUTPUT);	

}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);
    
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);	

//    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);	

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    // CLK_EnableModuleClock(TMR0_MODULE);
  	// CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);

    CLK_EnableModuleClock(TMR1_MODULE);
  	CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);

    CLK_EnableModuleClock(USCI0_MODULE);


    /*
        SPI CLK : USCIx_CLK 
        SPI SS  : USCIx_CTL0
        SPI MOSI : USCIx_DAT0 
        SPI MISO : USCIx_DAT1 
    */    
    /* Set USCI_SPI0 multi-function pins */
    SYS->GPA_MFPH = (SYS->GPA_MFPH & ~(SYS_GPA_MFPH_PA9MFP_Msk | SYS_GPA_MFPH_PA10MFP_Msk | SYS_GPA_MFPH_PA11MFP_Msk)) |
                    (SYS_GPA_MFPH_PA9MFP_USCI0_DAT1 | SYS_GPA_MFPH_PA10MFP_USCI0_DAT0 | SYS_GPA_MFPH_PA11MFP_USCI0_CLK);
    SYS->GPC_MFPH = (SYS->GPC_MFPH & ~(SYS_GPC_MFPH_PC14MFP_Msk)) | (SYS_GPC_MFPH_PC14MFP_USCI0_CTL0);    

    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_HIRC, MODULE_NoMsk);	
    CLK_EnableModuleClock(SPI0_MODULE);

    /* PA.3 is SPI0_SS,   PA.2 is SPI0_CLK,
       PA.1 is SPI0_MISO, PA.0 is SPI0_MOSI*/
   	SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk| SYS_GPA_MFPL_PA2MFP_Msk| SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA0MFP_SPI0_MOSI | SYS_GPA_MFPL_PA1MFP_SPI0_MISO | SYS_GPA_MFPL_PA2MFP_SPI0_CLK | SYS_GPA_MFPL_PA3MFP_SPI0_SS;


    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

   /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M031 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
    SYS_Init();

	GPIO_Init();
	UART0_Init();
	TIMER1_Init();
    check_reset_source();

    SysTick_enable(1000);
    #if defined (ENABLE_TICK_EVENT)
    TickSetTickEvent(1000, TickCallback_processA);  // 1000 ms
    TickSetTickEvent(5000, TickCallback_processB);  // 5000 ms
    #endif

    USPI_Master_Init();

    SPI_Slave_Init();

    /* Got no where to go, just loop forever */
    while(1)
    {
        loop();

    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
