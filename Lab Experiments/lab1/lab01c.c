//CS684 Lab 01 Part-3: LED sequence and on-off period control by two switches cycle cont

//Team members
//1. Jagat Pati Singh(173074014)	2. Kamlesh Kumar Sahu(173074010)	3. Dhananjay kr. sharma(173050046) 

//Blink one LED at default rate. For each key-press of sw-1 update the blinking rate in sequence as 0.5 Sec, 1 Sec, 2 Sec and then back to 0.5 Sec.
//For Each Sw-2 key-press change the LED in sequence R, G, B

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "inc/hw_ints.h"
#include <time.h>

uint8_t led_pointer= 2, sw1_count=0;
uint8_t sw1_sts, sw2_sts;
uint32_t updated_delay_count=6700000;

void setup(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
}

void pin_config(void)
{
	GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);

	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK)= GPIO_LOCK_KEY;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK)= 0 ;
	GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0);
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);

	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
}
/***************************************************************************************/

// update the delay of LED blinking
void update_delay_count(void)
{
	sw1_count++;
	if(sw1_count>2)
	{
		sw1_count = 0;
	}
	if(sw1_count==0)
	{
		updated_delay_count = 6700000;	//delay of approx 0.5 sec
	}
	else if(sw1_count==1)
	{
		updated_delay_count = 6700000*2;	//delay of approx 1 sec
	}
	else
		updated_delay_count = 6700000*4;	//delay of approx 2 sec
}

// update the color of LED 
void update_led_pointer(void)
{
	led_pointer= led_pointer*2;
	if(led_pointer>8)
	{
		led_pointer = 2;
	}
}

main(void)
{
	uint32_t delay_count;
	setup();
    pin_config();

    while(1)
    {
    	delay_count = updated_delay_count;
    	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, led_pointer);
		SysCtlDelay(delay_count);

		sw1_sts = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4);
		sw2_sts = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0);
		//SysCtlDelay(670000);

		//update delay of LED blinking on pressing switch1
		if(!(sw1_sts & 0x10))
		{
			update_delay_count();
		}
		//update color of LED on pressing switch2
		if(!(sw2_sts & 0x01))
		{
			update_led_pointer();
		}
	
	
    	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
    	SysCtlDelay(delay_count);

    	sw1_sts = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4);
    	sw2_sts = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0);
    	//SysCtlDelay(670000);

    	if(!(sw1_sts & 0x10))
    	{
    		update_delay_count();
    	}
		if(!(sw2_sts & 0x01))
		{
			update_led_pointer();
		}
    }
}
//
/*Note: Here I have used polling of sw-1 and sw-2 (not interrupt) and hence it requires long press of switches to take effect.
as it is executing two delay loops in between. To minimize the effect I have used the polling after every on and again after off the LED.*/
