//CS684 Lab 5, Part-2 Read Joysticck position through ADC and display on GLCD

//Team members
//1. Jagat Pati Singh(173074014)	2. Kamlesh Kumar Sahu(173074010)	3. Dhananjay kr. sharma(173050046) 


//Standard include files
#include <stdio.h>
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
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "driverlib/interrupt.h"
#include "driverlib/systick.h"
#include "driverlib/adc.h"
#include "driverlib/rom.h"
#include "driverlib/debug.h"
#include <time.h>
#include "driverlib/ssi.h"
#include "driverlib/uart.h"
#include "logo.h"


uint32_t left_right_val[4], up_down_val[4];
uint32_t left_right_avg, up_down_avg;
uint8_t left_right_pos, up_down_pos;


/*To display image include an array with hex values and index it accordingly*/
/* void glcd_cmd(cmd)
 * This function sends commands to the GLCD.
 * Value of RS is 0
 * Command is written on data lines.
 * Enable is made 1 for a short duration.
 */
void glcd_cmd(unsigned char cmd)
{
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,0x00);
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7,0x00);

	/* RS = 0 */
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6,0x00);

	/* Put command on data lines */
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,cmd);
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7,cmd);

	/* Generate a high to low pulse on enable */
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0,0x01);
	SysCtlDelay(100);//1340);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0,0x00);
}

/* void glcd_data(data)
 * This function sends data to the GLCD.
 * Value of RS is 1
 * Data is written on data lines.
 * Enable is made 1 for a short duration.
 */
void glcd_data(unsigned char data)
{
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,0x00);
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7,0x00);

	/* RS = 1 */
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6,0x40);

	/* Put command on data lines */
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,data);
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7,data);

	/* Generate a high to low pulse on enable */
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0,0x01);
	SysCtlDelay(100);//1340);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0,0x00);
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
	GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_3,0x08);

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
	GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_3,0x00);
	glcd_cmd(0xB8 | page);
	SysCtlDelay(100);

	/* Set CS2 (CS2=1 and CS1=0) The right side is selected(column>64) */
	GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_3,0x08);
	glcd_cmd(0xB8 | page);
	SysCtlDelay(100);
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
		SysCtlDelay(100);//6700);
	}
	else
	{
		/* Set CS2 (CS2=1 and CS1=0) The right side is selected(column>64) */
		GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_3,0x08);
		glcd_cmd(0x40 | (column-64));
		SysCtlDelay(100);//6700);
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

void main()
{
	unsigned char left_right_pos_prev,up_down_pos_prev;
	unsigned char i;
	unsigned char upper,lower,partial_data=0;

	uint8_t page, column;
	uint16_t data_index;

	/* Enable all the peripherals */
	/* PORTS A,E,F,C,D,B  and ADC0, ADC1 */
	SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

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

	/* Configure PE5 (RST), PE0 to PE3 (D0 to D3) and PB4 to PB7(D4 to D7) as output pins */
	//GLCD D0-D3
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |GPIO_PIN_5 );
	//GLCD D4-D7
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

	/* Configure RS = PC6 as output */
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE,  GPIO_PIN_4|GPIO_PIN_6);

	/*This ensures buzzer remains OFF, since PC4 when logic 0 turns ON buzzer */
	GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_4,0x10);
	//GLCD Enable PIN
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);

	/* Configure CS1 or CS2 as output */
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_3);

	/* Configure ADCs  */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
	GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_0);
	GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_1);

	//ADC CH-6 input - PD1, mapped to left-right joystic sense of Game console
	ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH6);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH6);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH6);
	ADCSequenceStepConfigure(ADC0_BASE,1,3,ADC_CTL_CH6|ADC_CTL_IE|ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 1);

	//ADC CH-7 input - PD1, mapped to up-down joystic sense of Game console
	ADCSequenceConfigure(ADC1_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC1_BASE, 1, 0, ADC_CTL_CH7);
	ADCSequenceStepConfigure(ADC1_BASE, 1, 1, ADC_CTL_CH7);
	ADCSequenceStepConfigure(ADC1_BASE, 1, 2, ADC_CTL_CH7);
	ADCSequenceStepConfigure(ADC1_BASE,1,3,ADC_CTL_CH7|ADC_CTL_IE|ADC_CTL_END);
	ADCSequenceEnable(ADC1_BASE, 1);



	/*initialize glcd*/
	glcd_init();
	/* Select a page and display lines on it from column 0 to 127 */
	//glcd_setpage(0);
	//glcd_setcolumn(0);
	//glcd_cleardisplay();

	//Display logo for few seconds and then display the cursor
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

	//SysCtlDelay(67000000/5);

	while(1)
	{
		ADCIntClear(ADC0_BASE, 1);
		ADCProcessorTrigger(ADC0_BASE, 1);
		ADCIntClear(ADC1_BASE, 1);
		ADCProcessorTrigger(ADC1_BASE, 1);
		while(!ADCIntStatus(ADC0_BASE, 1, false))
		{

		}
		while(!ADCIntStatus(ADC1_BASE, 1, false))
		{

		}
		ADCSequenceDataGet(ADC0_BASE, 1, left_right_val);
		ADCSequenceDataGet(ADC1_BASE, 1, up_down_val);

		left_right_avg = (left_right_val[0] + left_right_val[1] + left_right_val[2] + left_right_val[3] + 2)/4;
		up_down_avg = (up_down_val[0] + up_down_val[1] + up_down_val[2] + up_down_val[3] + 2)/4;

		left_right_pos = 120 - (left_right_avg / 34.125) ;
		up_down_pos = 56 - (up_down_avg / 73.125);

		if((left_right_pos_prev != left_right_pos) || (up_down_pos_prev != up_down_pos))
		{
			glcd_cleardisplay();

			//glcd_cursordisplay();
			if(up_down_pos % 8 == 0 )
			{
				glcd_setpage(up_down_pos/8);
				for(i=0;i<8;i++)
				{
					glcd_setcolumn(left_right_pos+i);
					glcd_data(0xFF);
				}
			}
			else
			{
				upper = up_down_pos % 8;
				lower = 8 - upper;
				partial_data = 0;
				for(i=0;i<upper;i++)
					partial_data = partial_data | 1<<i;
				glcd_setpage(up_down_pos/8);
				for(i=0;i<8;i++)
				{
					glcd_setcolumn(left_right_pos+i);
					glcd_data(~partial_data);
				}

				partial_data = 0;
				for(i=0;i<lower;i++)
					partial_data = partial_data | 1<<(7-i);
				glcd_setpage((up_down_pos/8)+1);
				for(i=0;i<8;i++)
				{
					glcd_setcolumn(left_right_pos+i);
					glcd_data(~partial_data);
				}
			}
		}

		left_right_pos_prev = left_right_pos;
		up_down_pos_prev = up_down_pos;
		SysCtlDelay(670000);
	}
}
//-------------------------------------------------------------------------------------------------------------------//

