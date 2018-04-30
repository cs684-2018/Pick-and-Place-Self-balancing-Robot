//CS684 Lab 00: Blinking LED
//This program makes the R, G B LEDs connected to Port pin PF1, 2 & 3 to blink in sequence

//Team members
//1. Jagat Pati Singh(173074014)	2. Kamlesh Kumar Sahu(173074010)	3. Dhananjay kr. sharma(173050046) 


#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
/*
* Function Name: setup()
* Input: none
* Output: none
* Description: Set crystal frequency and enable GPIO Peripherals  
* Example Call: setup();
*/
void setup(void)
{
	SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
}

/*

* Function Name: led_pin_config()
* Input: none
* Output: none
* Description: Set PORTF Pin 1, Pin 2, Pin 3 as output. On this pin Red, Blue and Green LEDs are connected. 
* Example Call: led_pin_config();
*/

void led_pin_config(void)
{
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
}


int main(void)
{
	uint8_t ui8LED = 8;

	setup();
	led_pin_config();

	while(1)
	{
		// Turn on the LED
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, ui8LED);
		// Delay for a bit
		SysCtlDelay(20000000);
		// Cycle through Red, Green and Blue LEDs
		if (ui8LED == 2)
		{
			ui8LED = 8;
		}
		else
		{
			ui8LED = ui8LED/2;
		}
	}
}
