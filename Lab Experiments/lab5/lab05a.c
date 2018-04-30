//CS684 Lab 5, Part-1 Interfacing GLCD

//Team members
//1. Jagat Pati Singh(173074014)	2. Kamlesh Kumar Sahu(173074010)	3. Dhananjay kr. sharma(173050046) 

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/debug.h"
#include "driverlib/pin_map.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "logo.h"
#include "mickey.h"
#include "one.h"
#include "two.h"
#include "three.h"
#include "four.h"
#include "five.h"
#include "six.h"
#include "seven.h"
#include "eight.h"


/*To display image include an array with hex values and index it accordingly*/

/* void glcd_cmd(cmd)
 * This function sends commands to the GLCD.
 * Value of RS is 0
 * Command is written on data lines.
 * Enable is made 1 for a short duration.
 */
void glcd_cmd(unsigned char cmd)
{
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0x00);
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x00);

	/* RS = 0 */
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00);

	/* Put command on data lines */
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, cmd);
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, cmd);

	/* Generate a high to low pulse on enable */
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x01);
	SysCtlDelay(1340);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x00);

}

/* void glcd_data(data)
 * This function sends data to the GLCD.
 * Value of RS is 1
 * Data is written on data lines.
 * Enable is made 1 for a short duration.
 */
void glcd_data(unsigned char data)
{
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0x00);
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x00);

	/* RS = 1 */
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x40);

	/* Put command on data lines */
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, data);
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, data);

	/* Generate a high to low pulse on enable */
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x01);
	SysCtlDelay(1340);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x00);
}

/* void glcd_init()
 * This function initializes the GLCD.
 * Always call this function at the beginning of main program after configuring the port pins.
 */
void glcd_init()
{
	SysCtlDelay(6700000/50);                            // creates ~10ms delay - TivaWare fxn
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0x00);    //cbi(GPORTC_2,GLCD_RST);
	SysCtlDelay(6700000/50);
	/* Set RST */
	GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_5,0x20);

	/* Set CS1 (CS1=1 and CS2=0) The right side is selected(column>64) */
	GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_3,0x00);

	/* Select the start line */
	glcd_cmd(0xC0);
	//SysCtlDelay(6700);

	/* Send the page */
	glcd_cmd(0xB8);
	//  SysCtlDelay(6700);

	/*Send the column */
	glcd_cmd(0x40);
	//SysCtlDelay(6700);

	/* Send glcd on command */
	glcd_cmd(0x3F);


	/* Initialize the right side of GLCD */
	/* Set CS2 (CS2=1 and CS1=0) The right side is selected(column>64) */
	GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_3, 0x08);

	/* Select the start line */
	glcd_cmd(0xC0);
	SysCtlDelay(6700);

	/* Send the page */
	glcd_cmd(0xB8);
	//  SysCtlDelay(6700);

	/*Send the column */
	glcd_cmd(0x40);
	//  SysCtlDelay(6700);

	/* Send glcd on command */
	glcd_cmd(0x3F);
	//  SysCtlDelay(6700);
}

/* void glcd_setpage(page)
 * This function selects page number on GLCD.
 * Depending on the value of column number CS1 or CS2 is made high.
 */
void glcd_setpage(unsigned char page)
{
	/* Set CS1 (CS1=1 and CS2=0) The right side is selected(column>64) */
	GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_3, 0x00);
	glcd_cmd(0xB8 | page);
	SysCtlDelay(1000);

	/* Set CS2 (CS2=1 and CS1=0) The right side is selected(column>64) */
	GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_3, 0x08);
	glcd_cmd(0xB8 | page);
	SysCtlDelay(1000);
}

/* void glcd_setcolumn(column)
 * This function selects column number on GLCD.
 * Depending on the value of column number CS1 or CS2 is made high.
 */
void glcd_setcolumn(unsigned char column)
{
	if(column < 64)
	{
		/* Set CS1 (CS1=1 and CS2=0) The right side is selected(column>64) */
		GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_3,0x00);
		glcd_cmd(0x40 | column);
		SysCtlDelay(6700);
	}
	else
	{
		/* Set CS2 (CS2=1 and CS1=0) The right side is selected(column>64) */
		GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_3,0x08);
		glcd_cmd(0x40 | (column-64));
		SysCtlDelay(6700);
	}
}

/* void glcd_cleardisplay()
 * This function clears the data on GLCD by writing 0 on all pixels.
 */
void glcd_cleardisplay()
{
	unsigned char i,j;
	for(i=0;i<8;i++)
	{
		glcd_setpage(i);
		for(j=0;j<128;j++)
		{
			glcd_setcolumn(j);
			glcd_data(0x00);
		}
	}
}

int main()
{
	uint8_t page, column;
	uint16_t data_index;
	/* Enable all the peripherals */
	/* PORTS A,E,F,C,D,B */
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	/* Unlock pin PF0 */
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK)= GPIO_LOCK_KEY;    // unlocking sw2 switch
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK)= 0;

	/* Configure Enable pin as output */
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);

	/* Configure PE5 (RST), PE0 to PE3 (D0 to D3) and PB4 to PB7(D4 to D7) as output pins */
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5);
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
	/* Configure RS = PC6 as output */
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE,  GPIO_PIN_4|GPIO_PIN_6);

	/*This ensures buzzer remains OFF, since PC4 when logic 0 turns ON buzzer */
	GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_4,0x10);

	/* Configure CS1 or CS2 as output */
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE,GPIO_PIN_3); //CS1 & 2 are always comlplementary so single line PD3 has been used for both (in H/W direct and through NOT gate

	/*initialize glcd*/
	glcd_init();

	/* Select a page and display lines on it from column 0 to 127 */
	while(1)
	{
		page=0;
		column=0;
		data_index=0;
		glcd_cleardisplay();
		for(page=0; page<8; page++)
		{
			glcd_setpage(page);
			for(column=0; column<128;column++)
			{
				glcd_setcolumn(column);
				glcd_data(logo[data_index]);
				data_index++;
			}
		}

		SysCtlDelay(67000000);

		page=0;
		column=0;
		data_index=0;
		glcd_cleardisplay();
		for(page=0; page<8; page++)
		{
			glcd_setpage(page);
			for(column=0; column<128;column++)
			{
				glcd_setcolumn(column);
				glcd_data(mickey[data_index]);
				data_index++;
			}
		}
		SysCtlDelay(67000000);

	}
}

