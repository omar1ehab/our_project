#include "stdio.h"
#include "functions.h"
#include "tm4c123gh6pm.h"
#include "stdbool.h"
int main()
{
char char_in;
UART_init();
PORTF_init ();
	
while (1)
{
	char distane =measure_distance();
	if (distane==0x64) GPIO_PORTF_DATA_R =0x02; // if distance =100m ...turn on red led
}}
