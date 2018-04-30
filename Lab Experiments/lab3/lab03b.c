//CS684 Lab 03 Part-2: Servomotor control using PWM

//Team members
//1. Jagat Pati Singh(173074014)	2. Kamlesh Kumar Sahu(173074010)	3. Dhananjay kr. sharma(173050046) 

//Default servo position is middle i.e. 0 deg. For each sw-1 press move -10 deg and for sw-2 +10 deg. Limit within range -80 to +80 deg
//Use PD0 for Servomotor PWM input
// PWM Pulse width ranges from 1mS to 2mS, For 1 mS servo angle is -90 deg, while for 2mS it +90 deg, for 1.5 sec angle is 0 deg


#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/pin_map.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"

#define PWM_FREQUENCY 55
#define DEBOUNCE_TIME 10	//in mS


#define key_Idle 	0
#define key_Press 	1
#define key_Release 	2
#define sw_close	0
#define sw_open		1


volatile uint8_t Key_1_state=0, key_2_state=0;
volatile uint8_t key_1_sts, key_1_flag=0, key_2_sts, key_2_flag=0;

volatile uint32_t pwm_period_count, pwm_pulse_width_count;
volatile uint32_t pwm_clock_freq;
volatile uint32_t N; // This variable will be used for controlling the PWM pulse width i.e. servo angle

// initialize clock setting and configure input/output port 
void setup()
{

	//use main oscillator with crystal 16MHZ and PLL 
	//PLL freq. = 200MHz divide by 5 = 40MHz
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	
	//configure PF4 as input with weak pull-up with current limit 2mAmp
	GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	//Open hardware lock on PF0
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK)= GPIO_LOCK_KEY;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK)= 0 ;

	//configure PF0 as input with weak pull-up with current limit 2mAmp
	GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0);
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}
// initialize PWM 
// this is used for servo motor movement control
void PWM_init()
{
	SysCtlPWMClockSet(SYSCTL_PWMDIV_8);	//PWM clock frequency input is system freq/64
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
	GPIOPinConfigure(GPIO_PD0_M1PWM0);
}

// initialize timer 0 with time out interrupt
// this timer is used for key-debouncing 
void timer0init(void)
{
	uint32_t debounce_time_count;

	key_chk_enable_flag=0;
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); // enable timer 0
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC); // enable 32 bit periodic timer
	debounce_time_count = (SysCtlClockGet() * DEBOUNCE_TIME / 1000) / 2; // timer 0 count as per required debounce time
	TimerLoadSet(TIMER0_BASE, TIMER_A, debounce_time_count - 1); //load timer 0 count 
	IntEnable(INT_TIMER0A); // enable timer 0 time-out interrupt 
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	IntMasterEnable(); // enable global interrupt
	TimerEnable(TIMER0_BASE, TIMER_A); // start timer 0
}

// FSM implementation to detect status of key 1 
void detectKeyPress_1(void)
{
	//read key1 status
	key_1_sts = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4);
	key_1_sts = (key_1_sts & 0x10)>>4;

	switch(Key_1_state)
	{
	case key_Idle : 				// if key1 state is in Idle state
		if(key_1_sts==sw_close) 		// if switch press/close is detected
		{
			Key_1_state = key_Press;        // change state to key_Press
		}
		break;

	case key_Press :                               // if key1 is in press state
		if(key_1_sts==sw_close)                // if switch press/close is detected
		{
			Key_1_state = key_Release;     // change state to key_release
			key_1_flag = 1;                // mark as valid key1 pressed detected
			
		}
		// otherwise change key1 state to Idle
		else
		{
			Key_1_state = key_Idle; 
		}
		break;
	
	case key_Release :  				// if key1 is in key_release state 

		if(key_1_sts==sw_open) 			// if status of key1 is open 
		{
			Key_1_state = key_Idle; 	// change state to key_Idle
			key_1_flag = 0; 		// mark as valid key1 released
		}
	}
}


//////////////////////////////////////////////////////////////////////////////////////////
void detectKeyPress_2(void)
{
	key_2_sts = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0);
	key_2_sts = (key_2_sts & 0x01);

	switch(key_2_state)
	{
	case key_Idle :					// if key2 state is in Idle state
		if(key_2_sts==sw_close)			// if switch press/close is detected
		{
			key_2_state = key_Press;	 // change state to key_Press
		}
		break;

	case key_Press :				// if key2 is in press state
		if(key_2_sts==sw_close)			// if switch press/close is detected
		{
			key_2_state = key_Release;	// change state to key_release
			key_2_flag = 1;			// mark as valid key2 pressed detected
			
		}
		else
		{
			key_2_state = key_Idle;
		}
		break;

	case key_Release :				// if key2 is in key_release state 
		if(key_2_sts==sw_open)			// if status of key2 is open 
		{
			key_2_state = key_Idle;		// change state to key_Idle
			key_2_flag = 0;			// mark as valid key2 released
			
		}
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

main(void)
{
	N = 18; // N = 180 corresponds to zero degree of servo angle
	uint8_t i;
	setup();
	PWM_init();
	Timer0_init();

	pwm_clock_freq = SysCtlClockGet()/8 ;
	pwm_period_count = (pwm_clock_freq / PWM_FREQUENCY) - 1;	//pwm_period_count correspons to the Time period of the output PWM
	PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, pwm_period_count);

	/*Count correponds to 1 mS duration = pwm_period_count * PWM_FREQ/ 1000, and for 2 mS it will be twice.
	 * Resolution: for 1 to 2 mS pulse width, total angle = 180 deg
	 * formula can be derived for the change in count i.e. pulse width for 5 deg resolution
	 * pwm_pulse_width_count = (36+N)* pwm_period_count * PWM_FREQ/(36 * 1000), where N ranges from 0 to 36 corresponds to servo angle -90 to +90
	 * We need to limit movement from -80 to + 80 thus range of N will be from 2 to 34 */


	pwm_pulse_width_count =  (pwm_period_count+1)*(36 + N)*PWM_FREQUENCY/36000;
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, pwm_pulse_width_count);
	PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
	PWMGenEnable(PWM1_BASE, PWM_GEN_0);

	while(1)
	{

		//if key1 is pressed, rotate servo in anti-clock by 10 degree in step of 5 degree /10 milli-sec
		if(key_1_flag)      
		{
			key_1_flag =0;
			for(i=0; i<2; i++)
			{
				N--;
				if (N<2)   // limit minimum angle to -80 degree
				   N=2;
				
				pwm_pulse_width_count =  (pwm_period_count+1)*(36 + N)*PWM_FREQUENCY/36000;
				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, pwm_pulse_width_count);
				SysCtlDelay(133333);
				

			}
		}

		//if key2 is pressed, rotate servo in clockwise by 10 deg in step of 5 degree /10 milli-sec
		if(key_2_flag)
		{
			key_2_flag =0;
			for(i=0; i<2; i++)
			{
				N++;
				if (N>34)       // limit maximum angle to 80 degree
				 N=34;
				pwm_pulse_width_count =  (pwm_period_count+1)*(36 + N)*PWM_FREQUENCY/36000;
				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, pwm_pulse_width_count);
				SysCtlDelay(133333);
			
			}

		}
	}
}
// timer 0 interrupt handler 
// this timer interrupts in every 10 ms and checks and updates the status of both the keys 
void Timer0IntHandler(void)
{
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	detectKeyPress_1();
	detectKeyPress_2();
}

