#include "tm4c123gh6pm.h"
#include "stdbool.h"
void UART_Init (void)
{	
SYSCTL_RCGCUART_R |= 0x01;
while ((SYSCTL_RCGCUART_R |= 0x01)==0);
SYSCTL_RCGCGPIO_R |= 0x01;
while ((SYSCTL_RCGCGPIO_R |= 0x01)==0);
UART0_CTL_R &= ~0x01;
UART0_IBRD_R = 104; // IBRD=int (16*1e6/ (16*9600) ) = int (104.16666)
UART0_FBRD_R = 11; // FBRD = int (0.1667* 64 + 0.5)
UART0_LCRH_R = 0x0070; // 8-bit word length, enable FIFO 001110000
UART0_CTL_R = 0x0301; // enable UARTenable,RXE, TXE and UART 001100000001
GPIO_PORTA_AFSEL_R |= 0x03; // enable alt function PAO , PA1
GPIO_PORTA_PCTL_R = 0x11; 
GPIO_PORTA_DEN_R |= 0x03; // enable digital I/O on PA0, PA1
GPIO_PORTA_AMSEL_R &= ~0x03; // disable analog function on PA0, PAl	
}

bool ready_to_recieve (void){
	
	return ((UART0_FR_R & 0x10)==0) ? 1:0; // when fifo is not empty,it will be ready to receive

}

bool ready_to_send (void){
	
	return ((UART0_FR_R & 0x20)==0) ? 1:0; // when fifo is not full,it will be ready to send

}


char UART_InChar(void){
while (!ready_to_recieve());  // will not read data till tiva is ready to receive (fifo is not empty)
	return (char) (UART0_DR_R);

}

void  UART_OutChar(char data) {

while ( ! ready_to_send() ); // will not write data till tiva is ready to send (fifo is not full)
	UART0_DR_R= data;
}

void UART_outString(char *pt){

while(*pt){
UART_OutChar(*pt);
	pt++;
}
}

void PORTF_init (void)
	{
SYSCTL_RCGCGPIO_R  |=0x20;
while ((SYSCTL_PRGPIO_R & 0x20)==0 );
	
GPIO_PORTF_LOCK_R =0x4C4F434B ;
GPIO_PORTF_CR_R = 0x1F;
GPIO_PORTF_AFSEL_R = 0x00;
GPIO_PORTF_AMSEL_R = 0x00;
GPIO_PORTF_DIR_R   = 0x0E;
GPIO_PORTF_DEN_R   = 0x1F;
GPIO_PORTF_DATA_R  = 0x11;
GPIO_PORTF_PUR_R = 0x11;
	}
	
	
	measure_distance (){
	// block of code here
	
	}
