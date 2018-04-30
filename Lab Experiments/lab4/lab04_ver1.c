//CS684 Lab 04: Use of ADC and UART
//Read the joystick position through ADCs and send it to pc through UART

//Team members
//1. Jagat Pati Singh(173074014)	2. Kamlesh Kumar Sahu(173074010)	3. Dhananjay kr. sharma(173050046) 


#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/systick.h"
#include "driverlib/adc.h"
#include "driverlib/rom.h"
#include "driverlib/debug.h"
#include <time.h>
#include <inc/hw_gpio.h>
#include "driverlib/ssi.h"
#include "driverlib/uart.h"


uint32_t left_right_val[4];
uint32_t up_down_val[4];

volatile uint32_t left_right_avg, up_down_avg;
volatile char left_right_pos_ascii[4], up_down_pos_ascii[4];

int main(void)
{
	uint8_t i;

	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);//This is to disable the speaker
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4);
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x10);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
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

	//ADC CH-7 input - PD0, mapped to up-down joystic sense of Game console
	ADCSequenceConfigure(ADC1_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC1_BASE, 1, 0, ADC_CTL_CH7);
	ADCSequenceStepConfigure(ADC1_BASE, 1, 1, ADC_CTL_CH7);
	ADCSequenceStepConfigure(ADC1_BASE, 1, 2, ADC_CTL_CH7);
	ADCSequenceStepConfigure(ADC1_BASE,1,3,ADC_CTL_CH7|ADC_CTL_IE|ADC_CTL_END);
	ADCSequenceEnable(ADC1_BASE, 1);


	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 9600,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	while (1)
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

		for(i=0; i<4; i++)
		{
			left_right_pos_ascii[i] =left_right_avg%10+48;
			left_right_avg=left_right_avg/10;
		}

		for(i=4; i>0; i--)
		{
			UARTCharPut(UART0_BASE, left_right_pos_ascii[i-1]);
			SysCtlDelay(10);
		}
		UARTCharPut(UART0_BASE, ',');

		for(i=0; i<4; i++)
		{
			up_down_pos_ascii[i]=up_down_avg%10+48;
			up_down_avg=up_down_avg/10;
		}

		for(i=4; i>0; i--)
		{
			UARTCharPut(UART0_BASE, up_down_pos_ascii[i-1]);
			SysCtlDelay(100);
		}

		UARTCharPut(UART0_BASE, '\n');

		SysCtlDelay(5000000);
		//SysCtlDelay(670000);
	}
}
//----------------------------------------------------------------------------------//



