//CS684 Lab 02 : Use of Timer and Timer Interrupt to detect Switch press with debouncing using state chart

//Team members
//1. Jagat Pati Singh(173074014)	2. Kamlesh Kumar Sahu(173074010)	3. Dhananjay kr. sharma(173050046) 

//Implement state chart for Sw-1 and Sw-2 key press detect. Turn-on 1 LED till key- is pressed, alternate LED for each keypress. For sw-2 press increment a variable.
//Use timer for key debouncing and use statechart for sw press detect

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "inc/hw_ints.h"

#define key_Idle 	0
#define key_Press 	1
#define key_Release 	2
#define sw_close	0
#define sw_open	1


uint8_t Key_1_state=0, key_2_state=0, led_pointer=2;
uint8_t led_sts_flag=0, sw2_sts_flag;
uint8_t key_1_sts, key_1_flag=0, key_2_sts, key_2_flag=0;
uint8_t key_1_chk, key_2_chk;
uint32_t sw2_counter;

void setup()
{
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK)= GPIO_LOCK_KEY;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK)= 0 ;
	GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0);
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
}

void Timer0_init()
{
	uint32_t debounce_time;
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	debounce_time = (SysCtlClockGet() / 67) / 2;
	TimerLoadSet(TIMER0_BASE, TIMER_A, debounce_time-1);
	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	IntMasterEnable();
	TimerEnable(TIMER0_BASE, TIMER_A);
}

uint8_t detectKeyPress_1(void)
{
	key_1_sts = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4);
	key_1_sts = (key_1_sts & 0x10)>>4;

	switch(Key_1_state)
	{
	case key_Idle :
		if(key_1_sts==sw_close)
		{
			Key_1_state = key_Press;
		}
		break;

	case key_Press :
		if(key_1_sts==sw_close)
		{
			Key_1_state = key_Release;
			key_1_flag = 1;
		}
		else
		{
			Key_1_state = key_Idle;
		}
		break;

	case key_Release :

		if(key_1_sts==sw_open)
		{
			Key_1_state = key_Idle;
			key_1_flag = 0;
		}
		break;


	default :
		Key_1_state = key_Idle;
		key_1_flag=0;

	}

	return key_1_flag;
}
//////////////////////////////////////////////////////////////////////////////////////////

uint8_t detectKeyPress_2(void)
{
	key_2_sts = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0);
	key_2_sts = (key_2_sts & 0x01);

	switch(key_2_state)
	{
	case key_Idle :
		if(key_2_sts==sw_close)
		{
			key_2_state = key_Press;
		}
		break;

	case key_Press :
		if(key_2_sts==sw_close)
		{
			key_2_state = key_Release;
			key_2_flag = 1;
		}
		else
		{
			key_2_state = key_Idle;
		}
		break;

	case key_Release :

		if(key_2_sts==sw_open)
		{
			key_2_state = key_Idle;
			key_2_flag = 0;
		}
		break;


	default :
		key_2_state = key_Idle;
		key_2_flag=0;

	}

	return key_2_flag;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Timer0IntHandler(void)
{
	// Clear the timer interrupt
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	key_1_chk= detectKeyPress_1();
	if(key_1_chk==1)
	{
		if(led_sts_flag==0)
		{
			led_sts_flag=1;
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, led_pointer);
			if (led_pointer >= 8)
			{
				led_pointer = 0x02;
			}
			else
			{
				led_pointer = led_pointer * 2;
			}
		}
	}

	else
	{
		led_sts_flag=0;
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
	}
	/*******************************************************************************************/
	key_2_chk= detectKeyPress_2();
	if(key_2_chk==1)
	{
		if(sw2_sts_flag==0)
		{
			sw2_sts_flag=1;
			sw2_counter++;
		}
	}

	else
	{
		sw2_sts_flag=0;
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(void)
{
	setup();
	Timer0_init();

	while(1)
	{

	}
}


