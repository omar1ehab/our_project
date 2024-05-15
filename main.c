#include "our_F.h"
#include "tm4c123gh6pm.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h> 
#include <stdio.h>
#include <stdlib.h>

int main(void)
{
UART_Init0();
PORTD_Init();
SysTick_Init();
RGB_Init();
loopp();
UART_outString("programm is over ");
GPIO_PORTF_DATA_R =0x05;

	
	
	
}
