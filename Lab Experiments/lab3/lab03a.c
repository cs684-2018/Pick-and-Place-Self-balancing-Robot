//CS684 Lab 03 Part-1: Use of PWM for LED colour wheel
//Implement LED colour wheel using PWM. Control brightness and mix of LED. For Sw-1 press make 
//transition fast with a lower limit and for Sw-2 slow with higher limit.
//Long press of sw-2 and no of time press in sw-1 (1 to 3) will switch the operation mode to manual mode-1, 2 or 3.
//Mode 1- intensity of Red LED can be controlled using SW1 and SW2. Mode-2 for Blue and Mode-3 for Green LED

//Team members
//1. Jagat Pati Singh(173074014)	2. Kamlesh Kumar Sahu(173074010)	3. Dhananjay kr. sharma(173050046) 


#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "inc/hw_ints.h"

#define PWM_FREQUENCY 40	//Hz
#define DEBOUNCE_TIME 30	//in mS
#define LONG_PRESS_TIME_THR 3	//in Sec


#define key_Idle 	0
#define key_Press 	1
#define key_Release 	2
#define sw_close	0
#define sw_open	1

volatile uint8_t Key_1_state=0, key_2_state=0;
volatile uint8_t key_1_sts, key_1_flag=0, key_2_sts, key_2_flag=0;
volatile uint8_t sw2_long_press_detect_flag, sw1_press_count, key_chk_enable_flag;
volatile uint32_t long_press_count;

volatile uint8_t transition_speed;
volatile uint8_t timer_2_flag,TempCounter =1;
volatile uint32_t timer_2_load_cnt;

volatile uint32_t pwm_clock_freq, pwm_period_count, pwm_pulse_width_count;

uint8_t R_brightness, G_brightness, B_brightness;

enum MODE{auto_mode, mode_1, mode_2, mode_3};
enum MODE operation_mode;
enum LED_STATE{R_up, G_dn, B_up, R_dn, G_up, B_dn};
enum LED_STATE led_state;

void auto_prog();
//void mode_1_prog();
//void mode_2_prog();
//void mode_3_prog();
void auto_mode_key_func();
void mode_1_key_func();
void mode_2_key_func();
void mode_3_key_func();
/////////////////////////////////////////////////////////////////////////


// initialize clock setting and configure input/output port 
void setup()
{
	//use main oscillator with crystal 16MHZ and PLL 
	//PLL freq. = 200MHz divide by 5 = 40MHz
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // enable port F

	//Open hardware lock on PF0
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

	//configure PF0 and PF4 as input with weak pull-up with current limit 2mAmp
	GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_DIR_MODE_IN);
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}
//////////////////////////////////////////////////////////////////////

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
///////////////////////////////////////////////////////////////////

// initialize timer 1 with time out interrupt
// this timer is used to detect long press
void timer1init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); // enable timer 1
	TimerConfigure(TIMER1_BASE, TIMER_CFG_ONE_SHOT); // enable 32 bit periodic timer
	long_press_count = (SysCtlClockGet() * LONG_PRESS_TIME_THR);// timer 1 count as per required long press time threshold
	TimerLoadSet(TIMER1_BASE, TIMER_A, long_press_count - 1);//load timer 1 count 
	IntEnable(INT_TIMER1A);// enable timer 1 time-out interrupt 
	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	IntMasterEnable();
	//TimerEnable(TIMER1_BASE, TIMER_A);
}

///////////////////////////////////////////////////////////////////

// initialize timer 2 with time out interrupt
// this timer is used control transition speed of LED wheel
void timer2init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
	TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
	timer_2_load_cnt = (SysCtlClockGet() / transition_speed) / 2;
	TimerLoadSet(TIMER2_BASE, TIMER_A, timer_2_load_cnt);
	IntEnable(INT_TIMER2A);
	TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	IntMasterEnable();
	TimerEnable(TIMER2_BASE, TIMER_A);
}
//////////////////////////////////////////////////////////////////
// initialize PWM 
// this is used for brightness control of LED
void PWM_init()
{
	SysCtlPWMClockSet(SYSCTL_PWMDIV_64);	//PWM clock frequency input is system freq/64

	//Calculation of load count for PWM
	pwm_clock_freq = SysCtlClockGet() / 64;   
	pwm_period_count = (pwm_clock_freq / (PWM_FREQUENCY)) - 1; //pwm_period_count correspons to the Time period of the output PWM

	//Port pin PF1 configure for PWM; RED LED
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);
	GPIOPinConfigure(GPIO_PF1_M1PWM5);
	PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, pwm_period_count);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, R_brightness * pwm_period_count / 256);
	PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, true);
	PWMGenEnable(PWM1_BASE, PWM_GEN_2);

	//Port pin PF2 configure for PWM; Blue LED
	GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
	GPIOPinConfigure(GPIO_PF2_M1PWM6);
	PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, pwm_period_count);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, B_brightness * pwm_period_count / 256);
	PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true);
	PWMGenEnable(PWM1_BASE, PWM_GEN_3);

	//Port pin PF2 configure for PWM; Green LED
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);
	GPIOPinConfigure(GPIO_PF3_M1PWM7);
	PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, pwm_period_count);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, G_brightness * pwm_period_count / 256);
	PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true);
	PWMGenEnable(PWM1_BASE, PWM_GEN_3);
}
///////////////////////////////////////////////////////////////////////////////////////
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
			if(sw2_long_press_detect_flag) // if switch key is long pressed
			{
				sw1_press_count++;     // change the mode
			}
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
			
			//start timer1 for long press detection of key2
			TimerLoadSet(TIMER1_BASE, TIMER_A, long_press_count);
			TimerEnable(TIMER1_BASE, TIMER_A);
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
			TimerDisable(TIMER1_BASE, TIMER_A);//stop timer1
		}
	}
}
////////////////////////////////////////////////////////////////////////////////

//timer0 interrupt handler
void Timer0IntHandler(void)
{
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	detectKeyPress_1();
	detectKeyPress_2();
	key_chk_enable_flag=1;
}
///////////////////////////////////////////////////////////////////////////////
//timer1 interrupt handler
void Timer1IntHandler(void)
{
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	sw2_long_press_detect_flag = 1;

}
///////////////////////////////////////////////////////////////////////////////

//timer2 interrupt handler
void Timer2IntHandler(void)
{
	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	//TempCounter++;
	timer_2_flag =1;
	//if(counter>254)
		//counter =1;
}
//////////////////////////////////////////////////////////////////////////////


int main(void)
{
	uint8_t sw2_key_chk;

	//initialize with default mode and default values of LED brightness and transition speed
	operation_mode = auto_mode; 	
	led_state = R_up;
	R_brightness = G_brightness = B_brightness = 2;
	transition_speed = 50;

	//clear all flags
	timer_2_flag=0; 
	sw2_long_press_detect_flag =0;
	sw1_press_count=0;

	setup();
	timer0init(); // initialize timer 0
	timer1init(); // initialize timer 1
	timer2init(); // initialize timer 2
	PWM_init();   // initialize PWM
	
	
	while(1)
	{
		while(!sw2_long_press_detect_flag)
		{
			if(timer_2_flag)
			{
				timer_2_flag=0;
				switch(operation_mode)
				{
					case auto_mode:
						auto_prog();
						break;
					case mode_1:	//Control RED LED brightness
						//mode_1_prog();
						//G_brightness = 1;
						//B_brightness = 1;
						PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, true);		//Enable R LED PWM output
						PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, false);	//Disableable B LED PWM output
						PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, false);	//Disable G LED PWM output
						break;
					case mode_2:	//Control Blue LED brightness
						//mode_2_prog();
						//G_brightness = 1;
						//R_brightness = 1;
						PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, false);	//Disable R LED PWM output
						PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true);		//Enable B LED PWM output
						PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, false);	//Disable G LED PWM output
						break;
					case mode_3:	////Control Green LED brightness
						//mode_3_prog();
						//R_brightness = 1;
						//B_brightness = 1;
						PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, false);	//Disable R LED PWM output
						PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, false);	//Disable B LED PWM output
						PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true);		//Enable G LED PWM output
				}

				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, R_brightness * pwm_period_count / 256);
				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, B_brightness * pwm_period_count / 256);
				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, G_brightness * pwm_period_count / 256);
			}
			switch(operation_mode)
			{
				case auto_mode:
					auto_mode_key_func();
					break;
				case mode_1:
					//mode_1_prog();
					mode_1_key_func();
					break;
				case mode_2:	//Control RED LED brightness
					//mode_2_prog();
					mode_2_key_func();
					break;
				case mode_3:
					//mode_3_prog();
					mode_3_key_func();
			}
		}

		//Indicate long press detect by put off all LEDs
		PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, false);
		PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, false);
		PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, false);

		do
		{
			sw2_key_chk = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0);
			sw2_key_chk = (sw2_key_chk & 0x01);
		}while(!sw2_key_chk);

		switch(sw1_press_count)
		{
			case 1:
				operation_mode = mode_1;
				break;
			case 2:
				operation_mode = mode_2;
				break;
			case 3:
				operation_mode = mode_3;
		}
		sw1_press_count =0;
		sw2_long_press_detect_flag =0;

		//Enable all LEDs output
		PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, true);
		PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true);
		PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true);
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////

// implementation of FSM for LED wheel
void auto_prog()
{
	switch(led_state)
	{
		case R_up :
			R_brightness++;
			if(R_brightness>=254)
			{
				led_state  = G_dn;
			}
			break;

		case G_dn :
			G_brightness--;
			if(G_brightness <= 1)
			{
				led_state  = B_up;
			}
			break;

		case B_up :
			B_brightness++;
			if(B_brightness>=254)
			{
				led_state  = R_dn;
			}
			break;

		case R_dn :
			R_brightness--;
			if(R_brightness <= 1)
			{
				led_state  = G_up;
			}
			break;

		case G_up :
			G_brightness++;
			if(G_brightness>=254)
			{
				led_state  = B_dn;
			}
			break;

		case B_dn :
			B_brightness--;
			if(B_brightness <= 1)
			{
				led_state  = R_up;
			}
		}
}
//////////////////////////////////////////////////////////////////

//implementation of key1 and key2 functions in auto mode
void auto_mode_key_func()
{
	if(key_chk_enable_flag)
	{
		key_chk_enable_flag=0;
		if(key_1_flag)    // if key1 on the decrease transition speed of LED wheel
		{  
			transition_speed--;
			if (transition_speed < 10)   // limit minimum transition speed
			{
				transition_speed = 10;
			}
			timer_2_load_cnt = (SysCtlClockGet() / transition_speed) / 2;
			TimerLoadSet(TIMER2_BASE, TIMER_A, timer_2_load_cnt);
		}

		if(key_2_flag)   // if key2 on the increase transition speed of LED wheel
		{
			transition_speed++;
			if (transition_speed > 250)   // limit maximum transition speed
			{
				transition_speed = 250;
			}
			timer_2_load_cnt = (SysCtlClockGet() / transition_speed) / 2;
			TimerLoadSet(TIMER2_BASE, TIMER_A, timer_2_load_cnt);
		}
	}
}
//////////////////////////////////////////////////////////////////
//implementation of key1 and key2 functions in mode1
void mode_1_key_func()
{
	if(key_chk_enable_flag)
	{
		key_chk_enable_flag=0;
		if(key_1_flag)      // if key1 on the increase red LED brightness
		{
			R_brightness++;
			if(R_brightness>=254)   // limit maximum brightness
			{
				R_brightness=254;
			}
		}

		if(key_2_flag)        // if key2 on the decrease red LED brightness
		{
			R_brightness--;
			if(R_brightness<=2)	// limit minimum brightness
			{
				R_brightness=2;
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////////
//implementation of key1 and key2 functions in mode2
void mode_2_key_func()
{
	if(key_chk_enable_flag)
	{
		key_chk_enable_flag=0;
		if(key_1_flag)			 // if key1 on the increase blue LED brightness 
		{
			B_brightness++;
			if(B_brightness>=254)   // limit maximum brightness
			{
				B_brightness=254;
			}
		}

		if(key_2_flag)			// if key2 on the decrease blue LED brightness 
		{
			B_brightness--;
			if(B_brightness<=2)    // limit minimum brightness
			{
				B_brightness=2;
			}
		}
	}
}

//////////////////////////////////////////////////////
//implementation of key1 and key2 functions in mode3
void mode_3_key_func()
{
	if(key_chk_enable_flag)
	{
		key_chk_enable_flag=0;
		if(key_1_flag)              // if key1 on the increase green LED brightness 
		{
			G_brightness++;
			if(G_brightness>=254)  // limit maximum brightness
			{
				G_brightness=254;
			}
		}

		if(key_2_flag)                // if key2 on the decrease green LED brightness
		{
			G_brightness--;
			if(G_brightness<=2)    // limit minimum brightness
			{
				G_brightness=2;
			}
		}
	}
}

