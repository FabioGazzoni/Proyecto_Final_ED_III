#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include "lpc17xx_uart.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_clkpwr.h"

#define TIME_TIMER 49999999
#define DMA_CHANNEL 7
#define ADC_VAL_SIZE 10

void configGPIO();
void configTimer0();
void configUART();
void configADC();
void configDMA();

void calculate_info_prom();
void on_off_bomb();

GPDMA_Channel_CFG_Type GPDMACfg0;
uint32_t valores[ADC_VAL_SIZE];

uint16_t info = 0b00;
uint8_t status_bomb_uart = 1;

int main(void)
{
	SystemInit();
	configGPIO();
	configUART();
	configTimer0();
	configADC();
	configDMA();

	while (1)
	{
	}

	return 0;
}

//---------------------CONFIGURACIONES------------------------------------------

void configGPIO()
{
	//como GPIO como salida
	PINSEL_CFG_Type pin_gpio;
	pin_gpio.Portnum = 0;
	pin_gpio.Pinnum = 0;
	pin_gpio.Funcnum = 0;
	pin_gpio.Pinmode = 3;

	PINSEL_ConfigPin(&pin_gpio);

	GPIO_SetDir(0, 1, 1); // GPIO como salida
}

void configTimer0()
{
	TIM_TIMERCFG_Type timer0;
	timer0.PrescaleOption = TIM_PRESCALE_TICKVAL;
	timer0.PrescaleValue = 1;

	TIM_MATCHCFG_Type match0_1;
	match0_1.MatchChannel = 1;
	match0_1.IntOnMatch = ENABLE;
	match0_1.StopOnMatch = DISABLE;
	match0_1.ResetOnMatch = ENABLE;
	match0_1.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
	match0_1.MatchValue = TIME_TIMER;

	TIM_ConfigMatch(LPC_TIM0, &match0_1);
	TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &timer0); // Modifica el PCLK a cclk/4

	CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_TIMER0, CLKPWR_PCLKSEL_CCLK_DIV_2); // PCLK = cclk/2
	TIM_Cmd(LPC_TIM0, ENABLE);
	NVIC_SetPriority(TIMER0_IRQn, 1);
	NVIC_EnableIRQ(TIMER0_IRQn);
}

void configUART()
{
	PINSEL_CFG_Type pin_RxUART;
	pin_RxUART.Portnum = 0;
	pin_RxUART.Pinnum = 11;
	pin_RxUART.Funcnum = 1;
	pin_RxUART.Pinmode = 3;
	PINSEL_ConfigPin(&pin_RxUART);

	PINSEL_CFG_Type pin_TxUART;
	pin_TxUART.Portnum = 0;
	pin_TxUART.Pinnum = 10;
	pin_TxUART.Funcnum = 1;
	pin_TxUART.Pinmode = 3;
	PINSEL_ConfigPin(&pin_TxUART);

	UART_CFG_Type UARTConfigStruct;
	UART_FIFO_CFG_Type UARTFIFOConfigStruct;
	// configuracion por defecto:
	UART_ConfigStructInit(&UARTConfigStruct);
	// inicializa periferico
	UART_Init(LPC_UART2, &UARTConfigStruct);
	UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);
	// Inicializa FIFO
	UART_FIFOConfig(LPC_UART2, &UARTFIFOConfigStruct);
	// Habilita transmision
	UART_TxCmd(LPC_UART2, ENABLE);

	// Habilita interrupcion por el RX del UART
	UART_IntConfig(LPC_UART2, UART_INTCFG_RBR, ENABLE);
	// Habilita interrupcion por el estado de la linea UART
	UART_IntConfig(LPC_UART2, UART_INTCFG_RLS, ENABLE);

	NVIC_SetPriority(UART2_IRQn, 0);
	// Habilita interrupcion por UART2
	NVIC_EnableIRQ(UART2_IRQn);
}


void configADC()
{
	PINSEL_CFG_Type pin_adc;
	pin_adc.Portnum = 0;
	pin_adc.Pinnum = 23;
	pin_adc.Funcnum = 1;
	pin_adc.Pinmode = PINSEL_PINMODE_TRISTATE;
	PINSEL_ConfigPin(&pin_adc);

	ADC_Init(LPC_ADC, 200000);
	ADC_BurstCmd(LPC_ADC, 1);
	ADC_ChannelCmd(LPC_ADC, 0, ENABLE);
}

void configDMA()
{
	GPDMA_Init();

	GPDMA_LLI_Type LLI0;
	// P2M ADC to Memoria
	LLI0.SrcAddr = (uint32_t)&LPC_ADC->ADDR0;
	LLI0.DstAddr = (uint32_t)valores;
	LLI0.NextLLI = (uint32_t)0;
	LLI0.Control = ADC_VAL_SIZE // 10 datos
				   | 2 << 18	// Origen de 32 bits
				   | 2 << 21	// Dest de 32 bits
				   | 1 << 27	// Auto incrementa en destino
				   | 1 << 31;	// Interrumpe al terminar

	GPDMACfg0.ChannelNum = DMA_CHANNEL;
	GPDMACfg0.TransferSize = ADC_VAL_SIZE;
	GPDMACfg0.TransferWidth = 0;
	GPDMACfg0.SrcMemAddr = 0;
	GPDMACfg0.DstMemAddr = (uint32_t)valores;
	GPDMACfg0.TransferType = GPDMA_TRANSFERTYPE_P2M;
	GPDMACfg0.SrcConn = GPDMA_CONN_ADC;
	GPDMACfg0.DstConn = 0;
	GPDMACfg0.DMALLI = (uint32_t)&LLI0;

	GPDMA_Setup(&GPDMACfg0);

	GPDMA_ClearIntPending(GPDMA_STATCLR_INTTC, DMA_CHANNEL);
	GPDMA_ClearIntPending(GPDMA_STATCLR_INTERR, DMA_CHANNEL);
	GPDMA_ChannelCmd(DMA_CHANNEL, ENABLE); // Habilito DMA
	NVIC_SetPriority(TIMER0_IRQn, 2);
	NVIC_EnableIRQ(DMA_IRQn);
}

//---------------------INTERRUPCIONES------------------------------------------
/*
 * @brief Recibe el estado de bomba apagada o encendida segun su status
 */
void UART2_IRQHandler(void)
{
	uint32_t intsrc, tmp, tmp1; // Receive Data Available or Character time-out
	// Determina la fuente de interrupcion
	intsrc = UART_GetIntId(LPC_UART2);
	tmp = intsrc & UART_IIR_INTID_MASK;
	// Evalua Line Status
	if (tmp == UART_IIR_INTID_RLS)
	{
		tmp1 = UART_GetLineStatus(LPC_UART2);
		tmp1 &= (UART_LSR_OE | UART_LSR_PE | UART_LSR_FE | UART_LSR_BI | UART_LSR_RXFE);
		// ingresa a un Loop infinito si hay error
		if (tmp1)
		{
			while (1)
			{
			};
		}
	}
	uint8_t data = 3;

	if ((tmp == UART_IIR_INTID_RDA) || (tmp == UART_IIR_INTID_CTI))
	{
		UART_Receive(LPC_UART2, &data, 1, NONE_BLOCKING);

		if (data == 1)
		{
			status_bomb_uart = 1;
		}
		else if (data == 0)
		{
			LPC_GPIO0->FIOCLR |= 1;
			status_bomb_uart = 0;
		}
	}
}

/*
 * @brief Por cada interrupcion envia por UART2 el dato info y enciende o apaga la bomba dependiendo de su estado
 */
void TIMER0_IRQHandler()
{
	UART_Send(LPC_UART2, &info, sizeof(info), BLOCKING);

	on_off_bomb();

	TIM_ClearIntPending(LPC_TIM0, TIM_MR1_INT);
}

/*
 * @brief Interrumpe al recibir 10 nuevos datos del ADC y los promedia para ser guardado en info
 */
void DMA_IRQHandler()
{
	GPDMA_ChannelCmd(DMA_CHANNEL, DISABLE); // Habilito DMA
											// check GPDMA interrupt on channel 0
	if (GPDMA_IntGetStatus(GPDMA_STAT_INT, DMA_CHANNEL))
	{ // check interrupt status on channel 0
		// Check counter terminal status
		if (GPDMA_IntGetStatus(GPDMA_STAT_INTTC, DMA_CHANNEL))
		{
			// Clear terminate counter Interrupt pending
			GPDMA_ClearIntPending(GPDMA_STATCLR_INTTC, DMA_CHANNEL);

			calculate_info_prom();

			GPDMA_Setup(&GPDMACfg0);
			GPDMA_ChannelCmd(DMA_CHANNEL, ENABLE); // Habilito DMA
		}
		if (GPDMA_IntGetStatus(GPDMA_STAT_INTERR, DMA_CHANNEL))
		{
			// Clear error counter Interrupt pending
			GPDMA_ClearIntPending(GPDMA_STATCLR_INTERR, DMA_CHANNEL);
			while (1)
			{
			};
		}
	}
}

//---------------------FUNCIONES UTILES------------------------------------------

/*
 * @brief Calcula el promedio de los 10 valores del ADC y lo guarda en info
 */
void calculate_info_prom(){
	uint16_t prom = 0;
				for (int i = 0; i <= ADC_VAL_SIZE - 1; i++)
				{
					prom = +(valores[i] >> 4) & 0xFFF;
				}
				info = prom;
}

/*
 * @brief Enciende o apaga la bomba segun el valor de info y del estado de bomba apagada o encendida segun su estado
 * Si el valor es menor a 0b2000 la bomba se apaga, si es mayor a 0b3500 se encendera
 */
void on_off_bomb(){
	if (status_bomb_uart == 1)
		{
			if (info > 3500)
			{
				GPIO_SetValue(0, 0b1<<0);
			}
			else if (info < 2000)
			{
				GPIO_ClearValue(0, 0b1<<0);
			}
		}
}
