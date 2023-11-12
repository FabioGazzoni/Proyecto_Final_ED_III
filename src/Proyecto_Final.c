#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include "lpc17xx_uart.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_gpdma.h"

#define DMA_SIZE 100
#define MAT_REGISTER_0_05_seg 4999999

void configGPIO(void);
void configTimer0(void);
void configTimer1(void);
void configTimer2(void);

void configADC(void);
void configUart(void);
void configDMA(void);

void clearBank0();

//uint16_t info = 0b00;
uint16_t info[DMA_SIZE] = 0;

uint32_t *first_bank0 = 0x2007C000;

uint32_t count = 0;
uint32_t lapso_tiempo = 0;
// Error Counter flag for Channel 0
volatile uint32_t Channel0_Err = 0;

int main (void){

	confUart();
	configTimer0();
	configADC();
	clearBank0();

	while(1){
//		UART_Send(LPC_UART2, &info, sizeof(info), BLOCKING);
	}
	return 0;
}

/* Se limpian los espacios de memoria del bank0 que se usaran
 * 
 * */
void clearBank0(){
	for (uint16_t i = 0; i < DMA_SIZE; i++) {
		*(first_bank0 + i) = 0;
	}
}


/* Se configura la salida a la bomba
 * P0.0 salida GPIO pull-up
 * */
void configGPIO(){
	LPC_PINCON->PINSEL0 &= ~(0b11);							// P0.0 como GPIO

	LPC_PINCON->PINMODE0 |= (0b11);							// P0.0 como pull-up

	LPC_GPIO0->FIODIR |= (0b1);								// P0.0 como salida
}

/* Se configura DMA M-M
 * a medida que se corra el programa se hara una copia de la variable info al bank0
 * */
void configDMA(void){

	NVIC_DisableIRQ(DMA_IRQn);
	GPDMA_Channel_CFG_Type GPDMACfg1;
	GPDMA_LLI_Type list_1;

	list_1.SrcAddr = (uint32_t) info;
	list_1.DstAddr = (uint32_t) first_bank0;
	list_1.NextLLI =  0;
	list_1.Control = DMA_SIZE
			|(2<<18) //width 32
			|(2<<21) //width 32
			|(1<<26) //dest inc
			|(1<<27) //dest inc
			;

	/* Initialize GPDMA controller */
	GPDMA_Init();

	// channel 1
	GPDMACfg1.ChannelNum = 0;
	GPDMACfg1.SrcMemAddr = list_1.SrcAddr;
	GPDMACfg1.DstMemAddr = list_1.DstAddr;
	GPDMACfg1.TransferSize = 1;
	GPDMACfg1.TransferWidth = GPDMA_WIDTH_WORD;
	GPDMACfg1.TransferType = GPDMA_TRANSFERTYPE_M2M;
	GPDMACfg1.SrcConn = 0;
	GPDMACfg1.DstConn = 0;
	GPDMACfg1.DMALLI = (uint32_t) &list_1;
	// Setup channel with given parameter
	GPDMA_Setup(&GPDMACfg1);
	NVIC_EnableIRQ(DMA_IRQn);

}
void DMA_IRQHandler (void)
{
	// check GPDMA CH0
	if (GPDMA_IntGetStatus(GPDMA_STAT_INT, 0)){ 
		if (GPDMA_IntGetStatus(GPDMA_STAT_INTERR, 0)){
			// Limpia el contador de errores pendientes
			GPDMA_ClearIntPending (GPDMA_STATCLR_INTERR, 0);
			Channel0_Err++;
		}
	}
	while ((Channel0_Err != 0)); 							// frena el programa si existe un error en DMA
	return;
}

/* ADC config
 * P0.23 como entrada analogica
 * actua en conjunto con TIMER 0
 * */
void configADC(){
	LPC_PINCON->PINSEL1 |= (1<<14);  						// ADC0 Pin0.23
	LPC_PINCON->PINMODE1 |= (1<<15); 						// Neither

	ADC_Init(LPC_ADC, 200000);								// Frecuencia a la que convierte el ADC
	ADC_BurstCmd(LPC_ADC, DISABLE);							// Sin brust
	ADC_StartCmd(LPC_ADC, ADC_START_ON_MAT01);				// MAT0.1
	ADC_ChannelCmd(LPC_ADC, 0, ENABLE);						// CH0
	ADC_EdgeStartConfig(LPC_ADC, ADC_START_ON_RISING); 		// Inicia por flanco de bajada
	ADC_IntConfig(LPC_ADC, ADC_ADINTEN0, ENABLE);
	NVIC_EnableIRQ(ADC_IRQn);
}
void ADC_IRQHandler()
{	
	for (uint16_t i = DMA_SIZE - 1; i > 0; i++) {
		info[i] = info[i-1]; 
	}
	
	info[0] = ((LPC_ADC->ADDR0)>>4)&0xFFF;
	UART_Send(LPC_UART2, &info[0], sizeof(info[0]), BLOCKING);


	LPC_ADC->ADGDR &= LPC_ADC->ADGDR;
return;
}

/* TIMER 0 config
 * ejecuta la conversion del ADC cada 0.1 seg
 * */
void configTimer0(){
	LPC_PINCON->PINSEL3 |= (3 << 26); 				// P1.29 as MAT0.1

	LPC_SC->PCONP |= (1 << 1);
	LPC_SC->PCLKSEL0 |= (1 << 2); 					// PCLK = cclk
	
	LPC_TIM0->PR = 0;
	LPC_TIM0->MR1 = MAT_REGISTER_0_05_seg; 			// toggle cada 0.5s = 49999999, 0.05s = 4999999
	LPC_TIM0->MCR = (1<<4);  						// reset on MAT0.1
	LPC_TIM0->EMR |= (0b11<< 6); 					// MAT0.1 toggle mode
	LPC_TIM0->IR |= (0x3F);      					// Clear all interrupt flag
	LPC_TIM0->TCR = 3;         						// Enable and Reset
	LPC_TIM0->TCR &= ~2;
}

/* TIMER 1 config
 * disparará el canal 0 de la DMA para realizar la copia de los datos al bank 0llevara registro del tiempo que dura la bomba en encendido
 * */
void configTimer1(){

	LPC_SC->PCONP |= (1 << 2);
	LPC_SC->PCLKSEL0 |= (1 << 4); 					// PCLK = cclk
	
	LPC_TIM1->PR = 0;
	LPC_TIM1->MR1 = (MAT_REGISTER_0_05_seg+1) * 2 * DMA_SIZE -1; 			// 0.05s * 2 * 100 = 10 seg
	LPC_TIM1->MCR = (0b11);  						// INT y RESET en Match0
	LPC_TIM1->IR |= (0x3F);      					// Clear all interrupt flag
	LPC_TIM1->TCR = 3;         						// Enable and Reset
	LPC_TIM1->TCR &= ~2;
}

void TIMER1_IRQHandler(){
	GPDMA_ChannelCmd(0, ENABLE);					// habilito DMA para que copie los datos al bank0, lo hará cada 10seg
	
	
	LPC_TIM1->IR |= (0x3F);      					// Clear all interrupt flag
	LPC_TIM1->TCR = 3;         						// Enable and Reset
	LPC_TIM1->TCR &= ~2;
		
}


/* TIMER 2 config
 * llevara registro del tiempo que dura la bomba en encendido
 * PR configurado para que la resolucion de la cuenta sea de 1ms
 * */
void configTimer2(){
	LPC_PINCON->PINSEL0 |= (3 << 8); 				// P0.4 CAP2.0

	LPC_SC->PCONP |= (1 << 22);
	LPC_SC->PCLKSEL0 |= (1 << 12); 					// PCLK = cclk
	
	LPC_TIM2->PR = 99999;							// TC incr cada 1ms
	LPC_TIM1->CCR |= (0b111); 						// CAP2.0 por ambos flancos e interrumpe
	
	LPC_TIM2->IR |= (0x3F);      					// Clear all interrupt flag
	LPC_TIM2->TCR = 3;         						// Enable and Reset
	LPC_TIM2->TCR &= ~2;
}
void TIMER2_IRQHandler(void)
{

    uint32_t oldCount = count;
    count = LPC_TIM2->CR0;

    lapso_tiempo = (count - oldCount);

    LPC_TIM2->IR |= 1 << 4;
}


void configUart(){
	LPC_PINCON->PINSEL0 &= ~(0b1<<21);				//
	LPC_PINCON->PINSEL0 |= (0b1<<20);				//

	UART_CFG_Type UARTConfigStruct;
	UART_FIFO_CFG_Type UARTFIFOConfigStruct;
	//configuracion por defecto:
	UART_ConfigStructInit(&UARTConfigStruct);
	//inicializa perifirico
	UART_Init(LPC_UART2, &UARTConfigStruct);
	UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);
	//Inicializa FIFO
	UART_FIFOConfig(LPC_UART2, &UARTFIFOConfigStruct);
	//Habilita transmision
	UART_TxCmd(LPC_UART2, ENABLE);
	return;
}

