#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include "lpc17xx_uart.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_dac.h"
#include "lpc17xx_gpdma.h"

#define TIME_TIMER 49999999
#define DMA_SIZE 1

void configGPIO();
void configTimer0();
void configADC();
void configUART();

void configPin();
void configDAC();
void configDMA();
uint32_t getProportionalValue(uint32_t input);
//void pwmGenerator(uint8_t duty_cycle);
//uint8_t getDutyCycle(uint32_t value);

// void configTimer1();
// void TIMER1_IRQHandler();

uint16_t info = 0b00;
uint32_t output_dac[2] = {0,(1023<<6)};
uint8_t status_bomb_uart = 0;
GPDMA_Channel_CFG_Type GPDMACfg;

int main(void)
{
	SystemInit();
	configGPIO();
	configUART();
	configTimer0();
	configADC();

//	configPin();
//	configDAC();
//	configDMA();

	while (1)
	{
//				for (int var = 0; var < 10000000; ++var) {
//				}
//				ADC_StartCmd(LPC_ADC, ADC_START_NOW);
//				if (output_dac[1] == 0) {
//								output_dac[1] = (1023 <<6);
//							} else {
//								output_dac[1] = 0;
//							}
	}
	return 0;
}

// P0.0 como GPIO como salida
void configGPIO()
{
	LPC_PINCON->PINSEL0 &= ~(0b11);

	LPC_PINCON->PINMODE0 |= (0b11);

	LPC_GPIO0->FIODIR |= (0b1);
}

void configADC()
{
	LPC_PINCON->PINSEL1 |= (1 << 14);  // ADC0 Pin0.23
	LPC_PINCON->PINMODE1 |= (1 << 15); // Neither

	ADC_Init(LPC_ADC, 200000);						   // Frecuencia a la que convierte el ADC
	ADC_BurstCmd(LPC_ADC, DISABLE);					   // Sin brust
	ADC_StartCmd(LPC_ADC, ADC_START_ON_MAT01);		   // Va a inicar la conversion con el MAT0.1

	ADC_ChannelCmd(LPC_ADC, 0, ENABLE);				   // Canal 0 del ADC
	ADC_EdgeStartConfig(LPC_ADC, ADC_START_ON_RISING); // Inicia por bajo
	ADC_IntConfig(LPC_ADC, ADC_ADINTEN0, ENABLE);
	NVIC_EnableIRQ(ADC_IRQn);
}

void ADC_IRQHandler()
{
	info = ((LPC_ADC->ADDR0) >> 4) & 0xFFF;

//	output_dac[1] = (getProportionalValue(info)<<6);

	UART_Send(LPC_UART2, &info, sizeof(info), BLOCKING);

	if(status_bomb_uart == 0){
		if (info > 3100)
			{
				LPC_GPIO0->FIOSET |= 1;
			}
			else if (info < 3000)
			{
				LPC_GPIO0->FIOCLR |= 1;
			}
	}


	LPC_ADC->ADGDR &= LPC_ADC->ADGDR;
}

void configTimer0()
{
	LPC_PINCON->PINSEL3 |= (3 << 26); // P1.29 as MAT0.1

	LPC_SC->PCONP |= (1 << 1);
	LPC_SC->PCLKSEL0 |= (1 << 2); // PCLK = cclk

	LPC_TIM0->PR = 0;
	LPC_TIM0->MR1 = TIME_TIMER;	  // toggle cada 0.5s
	LPC_TIM0->MCR = (0b1 << 4);	  // Timer0 reset on Match1
	LPC_TIM0->EMR |= (0b11 << 6); // MAT0.1 toggle mode
	LPC_TIM0->IR |= (0x3F);		  // Clear all interrupt flag
	LPC_TIM0->TCR = 3;			  // Enable and Reset
	LPC_TIM0->TCR &= ~2;
}

void configUART()
{
	LPC_PINCON->PINSEL0 &= ~(0b1 << 21);
	LPC_PINCON->PINSEL0 |= (0b1 << 20);

	LPC_PINCON->PINSEL0 &= ~(0b1 << 23);
	LPC_PINCON->PINSEL0 |= (0b1 << 22);

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
	NVIC_SetPriority(UART2_IRQn, 1);
	//Habilita interrupcion por UART2
	NVIC_EnableIRQ(UART2_IRQn);

	return;
}

void UART2_IRQHandler(void){
        uint32_t intsrc, tmp, tmp1;// Receive Data Available or Character time-out
        //Determina la fuente de interrupcion
        intsrc = UART_GetIntId(LPC_UART2);
        tmp = intsrc & UART_IIR_INTID_MASK;
        // Evalua Line Status
        if (tmp == UART_IIR_INTID_RLS){
                tmp1 = UART_GetLineStatus(LPC_UART2);
                tmp1 &= (UART_LSR_OE | UART_LSR_PE | UART_LSR_FE \
                                | UART_LSR_BI | UART_LSR_RXFE);
                // ingresa a un Loop infinito si hay error
                if (tmp1) {
                        while(1){};
                }
        }
        uint8_t data=3;

        if ((tmp == UART_IIR_INTID_RDA) || (tmp == UART_IIR_INTID_CTI)){
                UART_Receive(LPC_UART2, &data, 1, NONE_BLOCKING);

                if(data == 1){
                	LPC_GPIO0->FIOSET |= 1;
                	status_bomb_uart = 1;
                }else if (data == 0) {
                	LPC_GPIO0->FIOCLR |= 1;
                	status_bomb_uart = 0;
				}
        }
        return;
}


void configPin()
{
	PINSEL_CFG_Type pinCfg;
	pinCfg.Funcnum = PINSEL_FUNC_2;
	pinCfg.OpenDrain = 0;
	pinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
	pinCfg.Portnum = 0;
	pinCfg.Pinnum = 26;
	PINSEL_ConfigPin(&pinCfg);
}

void configDAC()
{
	DAC_CONVERTER_CFG_Type dacCfg;
	dacCfg.CNT_ENA = SET;
	dacCfg.DMA_ENA = SET;
	DAC_Init(LPC_DAC);
	/*
	x = (PCLK_DAC [25] * BIAS_Frec [1x10⁶])/(Frec_salida[1] * N°_Muestr [1])
	x = 25000000
	 */

	/*Set timeout*/
	uint32_t tmp;
	tmp = 10000; // cada cuantos ticks pide el dato a la DMA;
	DAC_SetDMATimeOut(LPC_DAC, tmp);
	DAC_ConfigDAConverterControl(LPC_DAC, &dacCfg);
}

void configDMA()
{
	GPDMA_LLI_Type LLI1;
	LLI1.SrcAddr = (uint32_t)output_dac;
	LLI1.DstAddr = (uint32_t)&(LPC_DAC->DACR);
	LLI1.NextLLI = (uint32_t)&LLI1;
	LLI1.Control = 	2 			//Cant datos
					| (2 << 18) // source width 32 bits
					| (2 << 21) | (1<<26) ;// dest width 32 bits

	GPDMA_Init();

	GPDMACfg.ChannelNum = 7;
	GPDMACfg.SrcMemAddr = (uint32_t)(output_dac);
	GPDMACfg.DstMemAddr = 0;
	GPDMACfg.TransferSize = 2;
	GPDMACfg.TransferWidth = 0;
	GPDMACfg.TransferType = GPDMA_TRANSFERTYPE_M2P;
	GPDMACfg.SrcConn = 0;
	GPDMACfg.DstConn = GPDMA_CONN_DAC;
	GPDMACfg.DMALLI = (uint32_t)&LLI1;
	GPDMA_Setup(&GPDMACfg);

	GPDMA_ChannelCmd(7, ENABLE);
}

//void pwmGenerator(uint8_t duty_cycle)
//{
//	for (int i = 0; i < DMA_SIZE; i++)
//	{
//		if (i < duty_cycle)
//		{
//			output_dac[i] = (0xffffffff << 6);
//		}
//		else
//		{
//			output_dac[i] = 0;
//		}
//	}
//}
//
//uint8_t getDutyCycle(uint32_t value)
//{
//	uint32_t min = 1700;
//	uint32_t max = 3300;
//
//	if(value < min){
//		return 0;
//	}else if(value > max){
//		return 100;
//	}else {
//		return (value - min)/((max - min)/100);
//	}
//
//}

uint32_t getProportionalValue(uint32_t input){
	uint32_t min = 1700;
	uint32_t max = 3300;

	if(input < min){
			return 0;
		}else if(input > max){
			return 1023;
		}
	uint32_t num = (input - min)*1023;
	uint32_t den = (max - min);
	uint32_t val = num/den;
	return val;

}
