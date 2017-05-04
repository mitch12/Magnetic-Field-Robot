//  square.c: Uses timer 2 interrupt to generate a square wave in pin
//  P2.0 and a 75% duty cycle wave in pin P2.1
//  Copyright (c) 2010-2015 Jesus Calvino-Fraga
//  ~C51~

#include <C8051F38x.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#define SYSCLK    48000000L // SYSCLK frequency in Hz
#define BAUDRATE  115200L   // Baud rate of UART in bps

#define L_CW  P2_2 //motor1going CW
#define L_CCW P2_3 //motor1going CCW
#define R_CW  P2_4 //motor2 CW
#define R_CCW P2_5 //motor2 CCW
#define right_turn_light P2_6
#define left_turn_light P2_7

int coil1_L=0.200; //coil_1 reading to turn left
int coil2_L=0.100;//coil_2 reading to turn left
int coil1_R=0.200; //coil_1 reading to turn right
int coil2_R=0.100; //coil_2 reading to turn right

volatile float V[4];
unsigned char overflow_count;
volatile unsigned char pwm_count=0;
volatile unsigned char pwm_count2=0;
int leftmotor_forward;//CCW
int leftmotor_backward;//CW
int rightmotor_forward;//CW
int rightmotor_backward;//CCW
int WAIT_TIME=10;
int stop_flag=0;

double stop_time=215;
double go_time=300;
double reverse_time=360;
double left_time=460;
double right_time=530;
double oneeighty_time=620;
double TOLERANCE=30;
double V_TOLERANCE=0.05;
double straight_tolerance;
char _c51_external_startup (void)
{
	PCA0MD&=(~0x40) ;    // DISABLE WDT: clear Watchdog Enable bit
	VDM0CN=0x80; // enable VDD monitor
	RSTSRC=0x02|0x04; // Enable reset on missing clock detector and VDD

	// CLKSEL&=0b_1111_1000; // Not needed because CLKSEL==0 after reset
	#if (SYSCLK == 12000000L)
		//CLKSEL|=0b_0000_0000;  // SYSCLK derived from the Internal High-Frequency Oscillator / 4 
	#elif (SYSCLK == 24000000L)
		CLKSEL|=0b_0000_0010; // SYSCLK derived from the Internal High-Frequency Oscillator / 2.
	#elif (SYSCLK == 48000000L)
		CLKSEL|=0b_0000_0011; // SYSCLK derived from the Internal High-Frequency Oscillator / 1.
	#else
		#error SYSCLK must be either 12000000L, 24000000L, or 48000000L
	#endif
	OSCICN |= 0x03; // Configure internal oscillator for its maximum frequency

	// Configure UART0
	SCON0 = 0x10; 
#if (SYSCLK/BAUDRATE/2L/256L < 1)
	TH1 = 0x10000-((SYSCLK/BAUDRATE)/2L);
	CKCON &= ~0x0B;                  // T1M = 1; SCA1:0 = xx
	CKCON |=  0x08;
#elif (SYSCLK/BAUDRATE/2L/256L < 4)
	TH1 = 0x10000-(SYSCLK/BAUDRATE/2L/4L);
	CKCON &= ~0x0B; // T1M = 0; SCA1:0 = 01                  
	CKCON |=  0x01;
#elif (SYSCLK/BAUDRATE/2L/256L < 12)
	TH1 = 0x10000-(SYSCLK/BAUDRATE/2L/12L);
	CKCON &= ~0x0B; // T1M = 0; SCA1:0 = 00
#else
	TH1 = 0x10000-(SYSCLK/BAUDRATE/2/48);
	CKCON &= ~0x0B; // T1M = 0; SCA1:0 = 10
	CKCON |=  0x02;
#endif
	TL1 = TH1;      // Init Timer1
	TMOD &= ~0xf0;  // TMOD: timer 1 in 8-bit autoreload
	TMOD |=  0x20;                       
	TR1 = 1; // START Timer1
	TI = 1;  // Indicate TX0 ready
	
	// Configure the pins used for square output
	P2MDOUT|=0b_0000_0011;
	P0MDOUT |= 0x10; // Enable UTX as push-pull output
	XBR0     = 0x01; // Enable UART on P0.4(TX) and P0.5(RX)                     
	XBR1     = 0x40; // Enable crossbar and weak pull-ups

	// Initialize timer 2 for periodic interrupts
	TMR2CN=0x00;   // Stop Timer2; Clear TF2;
	CKCON|=0b_0001_0000;
	TMR2RL=(-(SYSCLK/(2*48))/(100L)); // Initialize reload value
	TMR2=0xffff;   // Set to reload immediately
	ET2=1;         // Enable Timer2 interrupts
	TR2=1;         // Start Timer2

	EA=1; // Enable interrupts
	
	return 0;
}
void InitADC (void)
{
	// Init ADC
	ADC0CF = 0xF8; // SAR clock = 31, Right-justified result
	ADC0CN = 0b_1000_0000; // AD0EN=1, AD0TM=0
  	REF0CN = 0b_0000_1000; //Select VDD as the voltage reference for the converter
}

void InitPinADC (unsigned char portno, unsigned char pinno)
{
	unsigned char mask;
	
	mask=1<<pinno;
	
	switch (portno)
	{
		case 0:
			P0MDIN &= (~mask); // Set pin as analog input
			P0SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 1:
			P1MDIN &= (~mask); // Set pin as analog input
			P1SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 2:
			P2MDIN &= (~mask); // Set pin as analog input
			P2SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 3:
			P3MDIN &= (~mask); // Set pin as analog input
			P3SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		default:
		break;
	}
}

unsigned int ADC_at_Pin(unsigned char pin)
{
	AMX0P = pin;             // Select positive input from pin
	AMX0N = LQFP32_MUX_GND;  // GND is negative input (Single-ended Mode)
	// Dummy conversion first to select new pin
	AD0BUSY=1;
	while (AD0BUSY); // Wait for dummy conversion to finish
	// Convert voltage at the pin
	AD0BUSY = 1;
	while (AD0BUSY); // Wait for conversion to complete
	return (ADC0L+(ADC0H*0x100));
}

float Volts_at_Pin(unsigned char pin)
{
	 return ((ADC_at_Pin(pin)*3.30)/1024.0);
} 
// Uses Timer3 to delay <us> micro-seconds. 
void Timer3us(unsigned char us)
{
	unsigned char i;               // usec counter
	
	// The input for Timer 3 is selected as SYSCLK by setting T3ML (bit 6) of CKCON:
	CKCON|=0b_0100_0000;
	
	TMR3RL = (-(SYSCLK)/1000000L); // Set Timer3 to overflow in 1us.
	TMR3 = TMR3RL;                 // Initialize Timer3 for first overflow
	
	TMR3CN = 0x04;                 // Sart Timer3 and clear overflow flag
	for (i = 0; i < us; i++)       // Count <us> overflows
	{
		while (!(TMR3CN & 0x80));  // Wait for overflow
		TMR3CN &= ~(0x80);         // Clear overflow indicator
	}
	TMR3CN = 0 ;                   // Stop Timer3 and clear overflow flag
}
void waitms (unsigned int ms)
{
	unsigned int j;
	for(j=ms; j!=0; j--)
	{
		Timer3us(249);
		Timer3us(249);
		Timer3us(249);
		Timer3us(250);
	}
}

void Timer2_ISR (void) interrupt 5
{
	TF2H = 0; // Clear Timer2 interrupt flag
	
	pwm_count++;
	pwm_count2++;
	if(pwm_count>100) pwm_count=0;
	L_CCW=pwm_count>leftmotor_forward?0:1;
	L_CW=pwm_count>leftmotor_backward?0:1;
	if(pwm_count2>100) pwm_count2=0;
	R_CW=pwm_count2>rightmotor_forward?0:1;
	R_CCW=pwm_count2>rightmotor_backward?0:1;
	
	
}

void forward(void){//same speed
//	printf("forward\n");
	leftmotor_forward=50;
	leftmotor_backward=0;
	rightmotor_forward=50;
	rightmotor_backward=0;
}
void turnleft(void){//the car is turning left
	leftmotor_forward=50;
	leftmotor_backward=0;
	rightmotor_forward=0;
	rightmotor_backward=50;
	}

void turnright(void) {//the car is turning
	leftmotor_forward=0;
	leftmotor_backward=50;
	rightmotor_forward=50;
	rightmotor_backward=0;
	
}

void stop(void) {
//	printf("stop\n");
	leftmotor_forward=0;
	leftmotor_backward=0;
	rightmotor_forward=0;
	rightmotor_backward=0;
}

void stopspecial(void) {
//	printf("stop\n");
	leftmotor_forward=0;
	leftmotor_backward=0;
	rightmotor_forward=0;
	rightmotor_backward=0;
	waitms(1000);
}


void reverse_straight(void){

	leftmotor_forward=0;
	leftmotor_backward=50;
	rightmotor_forward=0;
	rightmotor_backward=50;

}


void reverse_left(void){

	leftmotor_forward=0;
	leftmotor_backward=0;
	rightmotor_forward=0;
	rightmotor_backward=100;

}


void reverse_right(void){

	leftmotor_forward=0;
	leftmotor_backward=100;
	rightmotor_forward=0;
	rightmotor_backward=0;

}


////////////////////////////////////////GUIDEWIRE INSTRUCTION FUNCTIONS///////////////////////////////////////

void left_atintersection(void){

	turnleft();
	waitms(1250);
	left_turn_light=1;//this line turns off the light
	
	
}

void right_atintersection(void){
	
	turnright();
	waitms(1250);
	right_turn_light=1;
	
}

void forward_atintersection(void){

	forward();
	waitms(500);
	
}

void turnoneeighty(){

	turnleft();
	waitms(2300);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void main (void)
{
	int i=0;
	
	int reverse_flag =0;
	int turn_flag =0;
	float ratio=0;
	//volatile float V[4];
	PCA0MD &= ~0x40;
	_c51_external_startup();
	InitPinADC(2, 1); // ADC, coil#1
	InitPinADC(2, 0); // ADC, coil#2
	InitPinADC(1, 7); //ADC, COIL#3
	// Initialize the ADC
	InitADC();
	printf("\x1b[2J"); // Clear screen using ANSI escape sequence.
	stop();
	
	while(1)
	{

		V[0]=Volts_at_Pin(LQFP32_MUX_P2_1);// ADC, coil#1
		V[1]=Volts_at_Pin(LQFP32_MUX_P2_0);// ADC, coil#2
		V[2]=Volts_at_Pin(LQFP32_MUX_P1_7);//ADC, COIL#3
		ratio=V[0]/V[1];
				
		printf("v0: %5.3f    v1: %5.3f   v2: %5.3f    ratio: %5.3f\r", V[0], V[1], V[2], ratio);
		
		
		/////////////////// Intersection CONTROL SUBROUTINE////////////////////////////////////////////
	
		
		/////////////////// Driving CONTROL SUBROUTINE////////////////////////////////////////////
		// Turning
		
		
	if(stop_flag){
		stop();
	}
	else{
	
		if(V[0] > 0.270){
			if(reverse_flag){
				if(V[2] > 0.100){
					turnright(); //turn steering wheel (backwards)
					//reverse_straight();
				//	waitms(10);//turnright();
				}
				else
					turnleft(); //turn steering wheel right (backwards)
				
			}
			else //turn steering wheel right (backwards)
				turnleft();
		}
			
		else if( V[1]>0.11){
			if(reverse_flag){
				if(V[2] > 0.100){
					turnleft();
				//	reverse_straight();
				//	waitms(10);//turnleft();
				}
				else {
					turnright();
				}		
			}
			else 
			turnright();		
		}
		
		
		// Forward
		else if(ratio<8 && ratio>0.950 ){
			if(reverse_flag){
				reverse_straight();
				waitms(50);
				//reverse_flag =0; 
			}
			else 
				forward();
		}
	}
	
		
		
		if(!reverse_flag && V[2] >0.3 ){
			stop();
		//	waitms(3000);
		//	printf("\n\nintersection\n\n");
		
			if (turn_flag ==1){
				left_atintersection();
				turn_flag = 0;
			}
			else if (turn_flag ==2){
				right_atintersection();
				turn_flag = 0;
			}
			else
				forward_atintersection(); 
				
		}
	
	///////////////////////////////////////////////////////////////////	
	/*if(V[2] > 0.15) {
		stop();
		waitms(3000);
			if (turn_flag ==1){
				left_atintersection();
				turn_flag = 0;
			}
			else if (turn_flag ==2){
				right_atintersection();
				turn_flag = 0;
			}
			else
				forward_atintersection(); 
		}
		*/
////////////////////////////////////////////////////////////////////////////////////

/////////////////////INSTRUCTION TIMER SUBROUTINE/////////////////////////////////////

	while(V[0]<V_TOLERANCE && V[1]<V_TOLERANCE)
	{
	V[0]=Volts_at_Pin(LQFP32_MUX_P2_1);// ADC, coil#1
	V[1]=Volts_at_Pin(LQFP32_MUX_P2_0);// ADC, coil#2
	V[2]=Volts_at_Pin(LQFP32_MUX_P1_7);//ADC, COIL#3
	waitms(WAIT_TIME);
	i=i+WAIT_TIME;
	printf("time since on: %d\n", i);

	}
	


	if(i<(stop_time+TOLERANCE)&& i>(stop_time-TOLERANCE)){
	//stopspecial();
	stop_flag=1;
	printf("stopping \n");
	}
	else if (i<(go_time+TOLERANCE)&& i>(go_time-TOLERANCE)){
	forward_atintersection();
	stop_flag=0;
	printf("going \n");
	}
	else if(i<(reverse_time+TOLERANCE)&& i>(reverse_time-TOLERANCE)){
	if(!reverse_flag)
		reverse_flag=1;
	else 
		reverse_flag=0;
	printf("reverse \n");
	}
	else if(i<(left_time+TOLERANCE)&& i>(left_time-TOLERANCE)){
	turn_flag=1;
	left_turn_light=0;
	printf("left \n");
	}
	else if(i<(right_time+TOLERANCE)&& i>(right_time-TOLERANCE)){
	turn_flag=2;
	right_turn_light=0;
	printf("right \n");
	}
	else if(i<(oneeighty_time+TOLERANCE)&& i>(oneeighty_time-TOLERANCE)){
	turnoneeighty();
	printf("one eighty \n");			//make a oneeighty function
	}
	else{
	forward(); //continue with line tracking algorithm
	}
	
	i=0.0;
	
//////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	
	
	}//end of while(1) loop
		
		
}


