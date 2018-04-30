//CS684 Lab 01 Part-1: Read Input Switch and Blink LEDs

//Team members
//1. Jagat Pati Singh(173074014)	2. Kamlesh Kumar Sahu(173074010)	3. Dhananjay kr. sharma(173050046) 

//Read Sw-1 at Pot pin PF-4, Turn On one LED out of 3 LEDs till the Sw-1 is kept pressed. Every Next press turn on another LED in sequence RGB

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

//Fuction for clock settings and Port-F enable
void setup(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);	
}

//Fuction for configuration of Port-F pins as In/Out
void pin_config(void)
{
	GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);    
}
/***************************************************************************************/

main(void)
{
    uint8_t sw1_sts, led_pointer=0x02;	//variable to point the LED Red Green or Blue; default/starting is Red
	setup();
    pin_config();    

    while(1)
    {
		//wait and keep checking sw1 untill it is pressed
    	do
    	{
    		sw1_sts = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4);

    	} while((sw1_sts & 0x10));	
		
		//On detection of switch-1 press turn on the currently pointed LED
    	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, led_pointer);
    	SysCtlDelay(670000); // wait for approx 50 mSec; key debounce time

		//Update the LED pointer so that it cycle through Red,Blue and Green
    	if (led_pointer >= 8)
    	{
    		led_pointer = 0x02;
    	}
    	else
    	{
    		led_pointer = led_pointer * 2;
    	}
		
		//wait and keep checking sw1 untill it is released
    	do
		{
			sw1_sts = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4);

		} while(!(sw1_sts & 0x10));

		//On detection of switch-1 release turn off the LED
    	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
    	SysCtlDelay(670000); // wait for approx 50 mSec; key debounce time
    }
}
