#include "stm32f05xxx.h"

#define	BUTTON_1		0b1
#define BUTTON_2		0b100
#define	BUTTON_3		0b1000
#define BUTTON_4		0b100000
#define BUTTON_5		0b10000000
#define BUTTON_6		0b1000000
#define BOUNCE_DELAY	50000
#define	STOP			208334
#define	GO				291668
#define	REVERSE			375001
#define	LEFT			458335
#define RIGHT			541668
#define TURNAROUND		625002


void ToggleLED(void);
volatile int Count = 0;
volatile int flag;


// Interrupt service routines are the same as normal
// subroutines (or C funtions) in Cortex-M microcontrollers.
// The following should happen at a rate of 1kHz.
// The following function is associated with the TIM1 interrupt 
// via the interrupt vector table defined in startup.s
void delay(int dly){

	while(dly--);

}


void command_delay(int c){
	TIM1_DIER &= ~BIT0; // enable update event (reload event) interrupt  
	GPIOA_ODR &= ~BIT8; // Toggle PA8
	delay(c);
	TIM1_DIER |= BIT0;
}

void command(int a, int b, int c){
	if (GPIOA_IDR &a){ 
		delay(b); 
		if (GPIOA_IDR &a){
			//command_delay(c);
			flag = c;
			
		}
	}
}



void Timer1ISR(void) 
{
	TIM1_SR &= ~BIT0; // clear update interrupt flag
	Count++;
	if (Count > 0)
	{ 
		Count = 0;
		ToggleLED(); // toggle the state of the LED every second
	}  
	
	if (flag !=0 ){
		command_delay(flag);
		flag = 0;
	}
	command(BUTTON_1,BOUNCE_DELAY,STOP);
	command(BUTTON_2,BOUNCE_DELAY,GO);
	command(BUTTON_3,BOUNCE_DELAY,REVERSE);
	command(BUTTON_4,BOUNCE_DELAY,LEFT);
	command(BUTTON_5,BOUNCE_DELAY,RIGHT);
	command(BUTTON_6,BOUNCE_DELAY,TURNAROUND);

	
	
}

void SysInit(void)
{
	// Set up output port bit for blinking LED
	RCC_AHBENR |= 0x00020000;  // peripheral clock enable for port A
	GPIOA_MODER |= 0x00000001; // Make pin PA0 output
	GPIOA_MODER |= BIT16;
	GPIOA_MODER |= BIT4;
	GPIOA_MODER |= BIT6;
	GPIOA_MODER |= BIT10;
	GPIOA_MODER |= BIT14;
	GPIOA_MODER |= BIT12;
	
	
	// Set up timer
	RCC_APB2ENR |= BIT11; // turn on clock for timer1
	TIM1_ARR = 560;      // reload counter with 8000 at each overflow (equiv to 1ms)
	ISER |= BIT13;        // enable timer interrupts in the NVIC
	TIM1_CR1 |= BIT4;     // Downcounting    
	TIM1_CR1 |= BIT0;     // enable counting    
	TIM1_DIER |= BIT0;    // enable update event (reload event) interrupt  
	enable_interrupts();
}

void ToggleLED(void) 
{    
	GPIOA_ODR ^= BIT8; // Toggle PA8
}



int main(void)
{
	SysInit();
	
	
while(1){


}
	return 0;
}
