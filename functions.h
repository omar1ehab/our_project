#include "tm4c123gh6pm.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h> 
#include <stdio.h>
#include <stdlib.h>
#define PI 3.14159
char i;
float currentLong , currentLat ;
char GPS_format[12][20];
char * token;
char GPS_logName[]="$GPRMC,";
char GPS[80] ;



void UART_Init0 (void) // ineract with putty 
{	
SYSCTL_RCGCUART_R |= 0x01;//inti the clock of UART
while ((SYSCTL_RCGCUART_R |= 0x01)==0);//wait for the clock to be set
SYSCTL_RCGCGPIO_R |= 0x01;//THe clock of tiva
while ((SYSCTL_RCGCGPIO_R |= 0x01)==0);//ready to be set
UART0_CTL_R &= ~0x01;//Disable at the start
UART0_IBRD_R = 104; // IBRD=int (16*1e6/ (16*9600) ) = int (104.16666)
UART0_FBRD_R = 11; // FBRD = int (0.1667* 64 + 0.5)
UART0_LCRH_R = 0x0070; // 8-bit word length, enable FIFO 001110000
UART0_CTL_R = 0x0301; // enable UARTenable,RXE, TXE and UART 001100000001
GPIO_PORTA_AFSEL_R |= 0x03; // enable alt function PAO , PA1
GPIO_PORTA_PCTL_R = 0x11; 
GPIO_PORTA_DEN_R |= 0x03; // enable digital I/O on PA0, PA1
GPIO_PORTA_AMSEL_R &= ~0x03; // disable analog function on PA0, PAl	
}




void PORTD_Init() // interact with gps
	{ 
    // 7 SEGMENT
   SYSCTL_RCGCUART_R |= 0x04;  // UART2
    SYSCTL_RCGCGPIO_R |= 0x08;  // Port D
    // Wetr4srxsxait for clock to stabilize
    while ((SYSCTL_PRGPIO_R & 0x08) == 0);
    while ((SYSCTL_PRUART_R & 0x04) == 0);
    // Unlock Port D
    GPIO_PORTD_LOCK_R = GPIO_LOCK_KEY;
    // Configure UART2 pins (PD6 = U2Rx, PD7 = U2Tx)
    GPIO_PORTD_CR_R |= 0xCF;    // Allow changes to PD6, PD7
    GPIO_PORTD_AMSEL_R &= ~0xCF; // Disable analog functionality on PD6, PD7
    GPIO_PORTD_AFSEL_R |= 0xC0;  // Enable alternate function on PD6, PD7
    GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R & 0x00FFFFFF) + 0x11000000;  // Select UART mode for PD6, PD7
    GPIO_PORTD_DEN_R |= 0xCF;	// Enable digital I/O on PD6, PD7
    GPIO_PORTD_DIR_R |= 0x8F;
	  GPIO_PORTD_DATA_R &= ~ 0xF;
    // Configure UART2
    UART2_CTL_R &= ~0x00000001;  // Disable UART2
    UART2_IBRD_R = 104;          // Integer baud rate divisor
    UART2_FBRD_R = 11;           // Fractional baud rate divisor
    UART2_LCRH_R = 0x70;         // 8-bit word length, enable FIFO
    UART2_CTL_R = 0x301;         // Enable UART2, TX, and RX
}



bool ready_to_recieve0 (void){
	
	return ((UART0_FR_R & 0x10)==0) ? 1:0; // when fifo is not empty,it will be ready to receive

}


char UART_InChar0(void){
while (!ready_to_recieve0());  // will not read data till tiva is ready to receive (fifo is not empty)
	return (char) (UART0_DR_R);

}


bool ready_to_send0 (void){
	
	return ((UART0_FR_R & 0x20)==0) ? 1:0; // when fifo is not full,it will be ready to send

}


void  UART_OutChar0(char data) {

while ( ! ready_to_send0() ); // will not write data till tiva is ready to send (fifo is not full)
	UART0_DR_R= data;
}





bool ready_to_recieve1 (void){
	
	return ((UART2_FR_R & 0x10)==0) ? 1:0; // when fifo is not empty,it will be ready to receive

}


char UART_InChar1(void){
while (!ready_to_recieve1());  // will not read data till tiva is ready to receive (fifo is not empty)
	return (char) (UART2_DR_R);

}







void UART_outString(char *pt){

while(*pt){ //true as long as the pointer didnt reach the null terminator to indicate the end of the string
UART_OutChar0(*pt);
	pt++;
}
}



void UART_OutFloat(float number) {
    char buffer[20]; // Adjust the size as needed
    snprintf(buffer, 20, "%.14f", number); // Convert float to string with 14 decimal places and newline
    UART_outString(buffer);
}

void RGB_Init(void){ 
	SYSCTL_RCGCGPIO_R  |=0x20;
while ((SYSCTL_PRGPIO_R & 0x20)==0 );
GPIO_PORTF_LOCK_R  =0x4C4F434B ;
GPIO_PORTF_CR_R = 0x1F;
GPIO_PORTF_AFSEL_R = 0x00;
GPIO_PORTF_AMSEL_R = 0x00;
GPIO_PORTF_DIR_R  = 0x0E;
GPIO_PORTF_DEN_R  = 0x1F;
GPIO_PORTF_DATA_R = 0x11;
GPIO_PORTF_PUR_R = 0x11;
}




void SysTick_Init(void){
NVIC_ST_CTRL_R = 0; // 1) disable SysTick during setup
NVIC_ST_RELOAD_R = 0x00FFFFFF; // 2) maximum reload value
NVIC_ST_CURRENT_R = 0; // 3) any write to CURRENT clears it
NVIC_ST_CTRL_R = 0x00000005; // 4) enable SysTick with core clock
}
// The delay parameter is in units of the 80 MHz core clock(12.5 ns)
void SysTick_Wait(int delay){
NVIC_ST_RELOAD_R = delay-1; // number of counts
while((NVIC_ST_CTRL_R&0x00010000)==0){ // wait for flag
}
}
// Call this routine to wait for delay*10ms
void SysTick_Wait10ms(int delay){
unsigned long j;
for(j=0; j<delay; j++){
SysTick_Wait(160000); // wait 10ms
}

}


float GPStoDeg(float val)
{
	return (int)(val / 100) + (val - (int)(val / 100) * 100) / 60.0;
    
} //Convert lat and long to phi in degrees 

float ToRad (float angle){
		return angle *PI/180 ;
}//convert phi into radians

void getCommandd(char *command , int len ){ 
		char character ;
		int i ;
		for(i=0 ; i<len ; i++){
				character=UART_InChar1();
				if( character!='\r'){
						command[i]=character ;
				}else if (character=='\r' || i==len ){
						break ;
				}
				
		}
	
}


 char lat [20]={0}    ;
char longi [20]={0}  ;
char speedd [10]={0} ;
 void getCommand(char *command , char stopchar ){ 
		char character [1] ;
		int i = 0  ;
		while(1){
				getCommandd(character,1);
				if( character[0]!=stopchar){
						command[i]=character[0] ;
					  i++ ;
						//UART_OutChar(command[i]) ;
				}else if (character[0]==stopchar ){
						break ;
				}
				
		}
	
}



char Valid [1] ; 
char N_or_S [1];
char E_or_W [1];

double long_final=31.2806451,lat_final=30.0648936; 
void GPS_read2(){
	char counter =0 ;
	char  recievedChar [1];
	char i=0 ;
	char flag =1 ;
	char c [1];
	do{
		getCommandd(c,1);
		while(c[0]!=GPS_logName[i]){
			memset(c,0,1);
			getCommandd(c,1);
		}
		i++;
		}while(i!=7);
	while(1){
			if(flag){
				
				
				getCommandd(recievedChar,1);
				if(recievedChar[0]==','){counter++;}
			}
			switch(counter){
				case 1 : getCommandd(Valid,1); break ;
				case 2 : getCommand(lat,',');counter++;flag=0;break;
				case 3 : getCommandd(N_or_S,1);flag=1;break;
				case 4 : getCommand(longi,',');counter++;flag=0;break;
				case 5 : getCommandd(E_or_W,1);flag=1;break;
				case 6 : getCommand(speedd,',');counter++;flag=0;break;
			
			}
			if(counter==7) break;
	
	}
	if(N_or_S[0]=='N')
						currentLat=atof(lat);
					else
						currentLat=-atof(lat) ;
	if(E_or_W[0]=='E')
						currentLong=atof(longi);
					else
						currentLong=-atof(longi);

}






float GPS_getDistance(float currentLong , float currentLat , float destLong , float destLat){
		//Get Radian 
	 float currentLongRad=ToRad(currentLong) ;//phiA
	 float currentLatRad=ToRad(currentLat) ;//phiB
	 float destLongRad = ToRad(destLong) ;//lambdaA
	 float destLatRad=ToRad(destLat) ;//lambdaB
	 //Get Difference 
	 float longDiff = destLongRad- currentLongRad ;
	 float latDiff = destLatRad - currentLatRad ;
	 // calculate Distance 
	 float a = pow (sin(latDiff/2),2) +cos(currentLatRad)*cos(destLatRad)*pow(sin(longDiff/2),2) ; 
		float c = 2*atan2(sqrt(a) , sqrt(1-a) ) ;
	 return 6371000*c ; 
}






//



float prev_lat=0;
float current_lat=0;
float prev_long=0;
float current_long=0;
float distacne=0 ;
float temp_distance ;
char check_U ;
char test1;
void loopp(void) {
	
while(1){

GPIO_PORTF_DATA_R  =0x02; // turn red led as start 
SysTick_Wait10ms(500);
GPS_read2(); // calling the function to overright on the global variables currentLong,currentLat

prev_long=GPStoDeg(currentLong) ;
UART_OutFloat(prev_long ) ; 
UART_outString(",") ;   
prev_lat=GPStoDeg(currentLat);
UART_OutFloat(prev_lat) ; 
UART_outString("\n");
GPIO_PORTF_DATA_R  =0x11;
SysTick_Wait10ms(500);
	
GPS_read2();
current_long=GPStoDeg(currentLong) ;
UART_OutFloat(current_long ) ;
UART_outString(",") ; 	
  
current_lat=GPStoDeg(currentLat);
UART_OutFloat(current_lat) ; 
UART_outString("\n");
GPIO_PORTF_DATA_R  =0x11;
	
	

distacne+=GPS_getDistance(prev_long,prev_lat,current_long,current_lat);
	


if (distacne>= 100){
	
	    GPIO_PORTF_DATA_R =0x18;
      UART_outString("you have reached 100m elhamdullah , if you want to continue please enter U ");
      check_U =UART_InChar0();
      UART_OutChar0(check_U);
	    if (check_U=='U') {distacne=0; loopp();}
	
      else break ;
	


} 

else loopp();

break;

}

}

