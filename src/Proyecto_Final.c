#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include "lpc17xx_uart.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_dac.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_clkpwr.h"

#define TIME_TIMER 49999999

void configGPIO();
void configTimer0();
void configADC();
void configUART();

uint16_t info = 0b00;
uint8_t status_bomb_uart = 0;

int main(void)
{
	SystemInit();
	configGPIO();
	configUART();
	configTimer0();
	configADC();

	while (1)
	{
	}
	return 0;
}

// P0.0 como GPIO como salida
void configGPIO()
{

	PINSEL_CFG_Type pin_gpio;
	pin_gpio.Portnum = 0;
	pin_gpio.Pinnum = 0;
	pin_gpio.Funcnum = 0;
	pin_gpio.Pinmode = 3;

	PINSEL_ConfigPin(&pin_gpio);

	GPIO_SetDir(0, 1, 1);//GPIO como salida
}

void configADC()
{
	PINSEL_CFG_Type pin_adc;
	pin_adc.Portnum = 0;
	pin_adc.Pinnum = 23;
	pin_adc.Funcnum = 1;
	pin_adc.Pinmode = 3;
	PINSEL_ConfigPin(&pin_adc);

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
	info = ADC_ChannelGetData(LPC_ADC, 0);


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
	TIM_TIMERCFG_Type timer0;
	timer0.PrescaleOption = TIM_PRESCALE_TICKVAL;
	timer0.PrescaleValue = 1;


	TIM_MATCHCFG_Type match0_1;
	match0_1.MatchChannel = 1;
	match0_1.IntOnMatch = DISABLE;
	match0_1.StopOnMatch = DISABLE;
	match0_1.ResetOnMatch = ENABLE;
	match0_1.ExtMatchOutputType = TIM_EXTMATCH_TOGGLE;
	match0_1.MatchValue = TIME_TIMER;

	TIM_ConfigMatch(LPC_TIM0,& match0_1);
	TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &timer0);//Modifica el PCLK a cclk/4

	CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_TIMER0, CLKPWR_PCLKSEL_CCLK_DIV_1); // PCLK = cclk
	TIM_Cmd(LPC_TIM0, ENABLE);
}

void configUART()
{
	PINSEL_CFG_Type pin_RxUART;
	pin_RxUART.Portnum = 0;
	pin_RxUART.Pinnum = 3;
	pin_RxUART.Funcnum = 1;
	pin_RxUART.Pinmode = 3;
	PINSEL_ConfigPin(&pin_RxUART);

	PINSEL_CFG_Type pin_TxUART;
	pin_TxUART.Portnum = 0;
	pin_TxUART.Pinnum = 2;
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
