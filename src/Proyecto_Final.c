#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include "lpc17xx_uart.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_adc.h"

void configGPIO();
void configTimer0();

void configADC();
void confUart();

uint16_t info = 0b00;

int main (void){

	confUart();
	configTimer0();
	configADC();

	while(1){
//		UART_Send(LPC_UART2, &info, sizeof(info), BLOCKING);
	}
	return 0;
}

//P0.0 como GPIO como salida
void configGPIO(){
	LPC_PINCON->PINSEL0 &= ~(0b11);

	LPC_PINCON->PINMODE0 |= (0b11);

	LPC_GPIO0->FIODIR |= (0b1);
}

void configADC(){
	LPC_PINCON->PINSEL1 |= (1<<14);  //ADC0 Pin0.23
	LPC_PINCON->PINMODE1 |= (1<<15); //Neither

	ADC_Init(LPC_ADC, 200000);//Frecuencia a la que convierte el ADC
	ADC_BurstCmd(LPC_ADC, DISABLE);//Sin brust
	ADC_StartCmd(LPC_ADC, ADC_START_ON_MAT01);//Va a inicar la conversion con el MAT0.1
	ADC_ChannelCmd(LPC_ADC, 0, ENABLE);// Canal 0 del ADC
	ADC_EdgeStartConfig(LPC_ADC, ADC_START_ON_RISING); //Inicia por bajo
	ADC_IntConfig(LPC_ADC, ADC_ADINTEN0, ENABLE);
	NVIC_EnableIRQ(ADC_IRQn);
}

void ADC_IRQHandler()
{
	info = ((LPC_ADC->ADDR0)>>4)&0xFFF;
	UART_Send(LPC_UART2, &info, sizeof(info), BLOCKING);

//	if(){
//
//	}
	LPC_ADC->ADGDR &= LPC_ADC->ADGDR;
return;
}


void configTimer0(){
	LPC_PINCON->PINSEL3 |= (3 << 26); // P1.29 as MAT0.1

   LPC_SC->PCONP |= (1 << 1);
   LPC_SC->PCLKSEL0 |= (1 << 2); // PCLK = cclk

   LPC_TIM0->PR = 0;
   LPC_TIM0->MR1 = 49999999; //toggle cada 0.5s
   LPC_TIM0->MCR = (0b1<<4);  // Timer0 reset on Match1
   LPC_TIM0->EMR |= (0b11<< 6); // MAT0.1 toggle mode
   LPC_TIM0->IR |= (0x3F);      // Clear all interrupt flag
   LPC_TIM0->TCR = 3;         // Enable and Reset
   LPC_TIM0->TCR &= ~2;
}


void confUart(){
	LPC_PINCON->PINSEL0 &= ~(0b1<<21);
	LPC_PINCON->PINSEL0 |= (0b1<<20);

	UART_CFG_Type UARTConfigStruct;
		UART_FIFO_CFG_Type UARTFIFOConfigStruct;
		//configuraci�n por defecto:
		UART_ConfigStructInit(&UARTConfigStruct);
		//inicializa perif�rico
		UART_Init(LPC_UART2, &UARTConfigStruct);
		UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);
		//Inicializa FIFO
		UART_FIFOConfig(LPC_UART2, &UARTFIFOConfigStruct);
		//Habilita transmisi�n
		UART_TxCmd(LPC_UART2, ENABLE);
	return;
}
