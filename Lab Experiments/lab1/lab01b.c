///CS684 Lab 01 Part-2: Increment a variable for each key-press for Sw-2

//Team members
//1. Jagat Pati Singh(173074014)	2. Kamlesh Kumar Sahu(173074010)	3. Dhananjay kr. sharma(173050046) 


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

void setup(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	//unlock PF0 based on requirement
}

void pin_config(void)
{
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK)= GPIO_LOCK_KEY;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK)= 0 ;

	GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0);
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
}
/***************************************************************************************/
uint8_t sw2_sts, sw2_count=0;

main(void)
{    
	setup();
    pin_config();

    while(1)
    {
    	//wait and keep checking sw2 untill it is pressed
    	do
    	{
    		sw2_sts = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0);

    	} while((sw2_sts & 0x01));

    	sw2_count++;	//increment the sw2 press count
    	SysCtlDelay(670000); // wait for approx 50 mSec; key debounce time

    	//wait and keep checking sw2 untill it is released
    	do
		{
			sw2_sts = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0);

		} while(!(sw2_sts & 0x01));
    	SysCtlDelay(670000); // wait for approx 50 mSec; key debounce time

    }
}



