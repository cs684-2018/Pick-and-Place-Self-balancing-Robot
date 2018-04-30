#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <L3G.h>
#include <ADXL345.h>
#include <PID_v1.h>
#include <MovingAverageFilter.h>


/******************variables for accelerometer and gyro****************/

ADXL345 acc;
L3G gyro;

unsigned long task1_time;
unsigned long interval1=15;

int sampleNum=500;
int dc_offset=0;
double noise=0;
int t=0,f=1;

double fXg = 0;
double fYg = 0;
double fZg = 0;

int rate;
int prev_rate=0;

const float alpha = 0.6;		//determined experimentaly for bst peformance
double angle=0, angle_corrected=0, combined_angle=0;

/*******************variables for Xbee commanding *************************/

unsigned char data_recv;   // data received through serial communication
unsigned long task4_time;
unsigned long interval4=100; // time interval at which serial data is being received

/*******************variables and functions for motor *************************/

int right_PWM = 7; 		//right motor is connected to pin number 4 of port H(PH4)
int right_forward = 8; 	//PH5
int right_backward = 9; 	//PH6

int left_PWM = 10;   		//right motor is connected to pin number 4 of port B(PB4)
int left_forward = 11;  	//PB5
int left_backward = 12;  	//PB6

int speed=0; 				// initial speed of motor is set ot zero
unsigned char motor_on=0;
int throttle = 0;			//Forward/reverse movement
int steering = 0;			//clockwise/anticlockwise rotation
float steeringOffset=0;

float motor1SteeringOffset, motor2SteeringOffset;

void motor_init()  			// initializing motors
{
    pinMode(right_PWM, OUTPUT);  		// set right motor PWM pin as output pin 
    pinMode(right_forward, OUTPUT);		//forward/reverse direction control line as output
    pinMode(right_backward, OUTPUT);

    pinMode(left_PWM, OUTPUT);  		// left motor 
    pinMode(left_forward, OUTPUT);
    pinMode(left_backward, OUTPUT);
}

void motor1forward()  		//controlling forward motion of motor1(right-motor)
{
   digitalWrite(right_forward, HIGH); 	// setting forward movement of right motor to high
   digitalWrite(right_backward, LOW); 	// setting backward movement of right motor to low 
}

void motor1backward() 		//controlling backward motion of motor1(right-motor)
{
   digitalWrite(right_forward, LOW);  	//setting forward movement of right motor to low
   digitalWrite(right_backward, HIGH); 	// setting backward movement of right motor to high
}

void motor2forward()		//controlling forward motion of motor2(left-motor)
{
   digitalWrite(left_forward, HIGH);  	// setting forward movement of left motor to high
   digitalWrite(left_backward, LOW);   // setting backward movement of left motor to low
    
}

void motor2backward()		//controlling backward motion of motor2(left-motor)
{
   digitalWrite(left_forward, LOW);   	//setting forward movement of left motor to low
   digitalWrite(left_backward, HIGH); 	// setting backward movement of left motor to high
    
}

void stop()					//stop both motors
{
   digitalWrite(right_forward, LOW);  	
   digitalWrite(right_backward, LOW);  	

   digitalWrite(left_forward, LOW);   	
   digitalWrite(left_backward, LOW);  	
}

void velocity_move(int motor, int speed)  	//Control movement of motors as per calculation using PID
{
  if (motor == 1)						//Motor-1 control
  {   
    if (speed >= 0 && speed <= 255)		//check speed for direction forward
     {
		speed = speed + 25; 			// added offset for initial  speed 
		if (speed>255)
			speed = 255;				//Limit max speed

       analogWrite(right_PWM, speed); 	// give new speed forward direction to motor-1
       motor1forward();          		
     }

    else if(speed < 0 && speed >= -255)	//check speed for direction backward
     { 
       int speed_abs = abs(speed);
       speed_abs = speed_abs + 25;		// added offset for initial  speed 
       if (speed_abs>255)				//Limit min speed
		speed_abs = 255;

       analogWrite(right_PWM, speed_abs);	// give new speed backward direction to motor-1
       motor1backward(); 
     }
 }
 
 else 				//Motor-2 control similar as above
   {
     if (speed >= 0 && speed <= 255)
       {
         speed = speed + 25; 
         if (speed>255)
			speed = 255;
		
         analogWrite(left_PWM, speed);
         motor2forward();
       }

     else if(speed < 0 && speed >= -255)
       {
         int speed_abs = abs(speed);
         speed_abs = speed_abs + 25;
         if (speed_abs>255)
			speed_abs = 255;
		
         analogWrite(left_PWM, speed_abs);
         motor2backward();
       }
   }
 }

/*****************variables and function for encoder***************************/

unsigned long task2_time;
unsigned long interval2=15;
float speed_m1, speed_m2, speed_combined;

static int global_counts_m1;
static int global_counts_m2;

static int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};	//For quadrature encoder direction
static uint8_t enc1_val = 0;
static uint8_t enc2_val = 0;

MovingAverageFilter speedMovingAverageFilter(20);


void encoder_init()
{
  cli();			//Disable global interrupt
  
  // initialize the global state
  global_counts_m1 = 0;
  global_counts_m2 = 0;
  
  //set interrupts for encoder pins
  PCICR |= (1<<PCIE2);		
  PCMSK2 |= (1<<PK1) | (1<<PK0) | (1<<PK2) | (1<<PK3);  

  enc1_val = enc1_val | ((PINK & 0b0011));
  enc2_val = enc2_val | ((PINK & 0b1100) >> 2 );  
  
  PCIFR = 0xFF;		// clear interrupts initially if any
  
  sei();			// enable global interrupt
}

ISR(PCINT2_vect)		//Encoder interrupt service routine to update encoder counts whenever there is change
{
  enc1_val = enc1_val << 2;
  enc1_val = enc1_val | (PINK & 0b0011);

  enc2_val = enc2_val << 2;
  enc2_val = enc2_val | ((PINK & 0b1100) >> 2) ;
  
  global_counts_m1 = global_counts_m1 + lookup_table[enc1_val & 0b1111];
  global_counts_m2 = global_counts_m2 + lookup_table[enc2_val & 0b1111];
}


int getCountsM1() //get the current count of motor1
{
  cli();
  int tmp = global_counts_m1;
  sei();
  return tmp;
}

int getCountsM2() //get the current count of motor2
{
  cli();
  int tmp = global_counts_m2;
  sei();
  return tmp;
}

int getCountsAndResetM1() // to get current count of motor1 and reset its count to zero
{
  cli();
  int tmp = global_counts_m1;
  global_counts_m1 = 0;
  sei();
  return tmp;
}

int getCountsAndResetM2() // to get current count of motor2 and reset its count to zero
{
  cli();
  int tmp = global_counts_m2;
  global_counts_m2 = 0;
  sei(); 
  return tmp;
}

/*******************variables for servo motor *************************/

Servo armservo,gripservo;   // Two servo motor for arm rotation and gripper rotation

int arm_servoangle=1500;  	// initial position of arm servo moter 
int grip_servoangle=1800; 	// initial position of gripper motor 

/***************variables of PID controller*************************/
 
double anglePIDSetpoint, anglePIDInput, anglePIDOutput, sci_angle;
double speedPIDInput, speedPIDOutput, speedPIDSetpoint;

//Specify the links and initial tuning parameters
double angleKp=15, angleKi=0.2, angleKd=0.09;
double speedKp=5.0, speedKi=2.0, speedKd=0.01;

//Function prototype 
PID anglePID(&anglePIDInput, &anglePIDOutput, &anglePIDSetpoint, angleKp, angleKi, angleKd, REVERSE);
PID speedPID(&speedPIDInput, &speedPIDOutput, &speedPIDSetpoint, speedKp, speedKi, speedKd, DIRECT);

/**********************************************************************************/


/*********************************************************************************/
/***************************initialization of main program************************/
/*********************************************************************************/

void setup() 
{
  Serial.begin(115200);			//Inialize serial communication, it is connected via Xbee module
  
  armservo.attach(2);			//initialize arm servo motor connected at pin PE4
  gripservo.attach(4); 			//initialize gripper servo motor connected at pin PG5
  
  armservo.writeMicroseconds(arm_servoangle);	//Move both srvo motor to initial default position
  gripservo.writeMicroseconds(grip_servoangle); 

  Wire.begin();					//Enable I2C communication to communicate to accelerameter and gyro
  acc.begin();					// Initialize accelerometer 
  while (!gyro.init());			// Inialize gyro
  gyro.enableDefault();
      
  //Initialize both PID loop (angle and speed)   
  anglePIDSetpoint = 0;					//Initial set-point
  anglePID.SetOutputLimits(-255,255);	//Range limit
  anglePID.SetSampleTime(10);			//sample time in milliseconds
   
  speedPIDSetpoint = 0;
  speedPID.SetOutputLimits(-7,7);
  speedPID.SetSampleTime(10);   

  //Initialize motors and encoder
  motor_init();
  encoder_init();
  motor_on=1;
}

void loop() 
{  
	  // Process-1: Gyro and accelerometer reading every 'interval1' time (here interval1 = 15 mSec )  
	  if((millis() - task1_time) >= interval1)
	  {
		 task1_time = millis();  		// mark current time so that remember when to take next sample      
			   
		 int raw_Xg, raw_Yg, raw_Zg;
		 double pitch, roll, Xg, Yg, Zg;
		 
		 acc.read(&Xg, &Yg, &Zg);     	//read accelerameter 

		 //Low Pass Filtering
		 fXg = Xg * alpha + (fXg * (1.0 - alpha));
		 fYg = Yg * alpha + (fYg * (1.0 - alpha));
		 fZg = Zg * alpha + (fZg * (1.0 - alpha));

		 //Roll & Pitch calculation
		 roll  = (atan2(-fYg, fZg)*180.0)/M_PI;
		 pitch = (atan2(fXg, sqrt(fYg*fYg + fZg*fZg))*180.0)/M_PI;    
		  
		 gyro.read();					//read gyro
		 
		 rate=((int)gyro.g.y)/100; 		//calculate rate
		
		 angle = combined_angle + ((double)(rate) * interval2) / 1000;	//calculate angle
		 
		 prev_rate = rate;				//remember current speed for next loop rate integration.
	  
		 // Limit value of angle between 0-359 degrees
		 if (angle < 0)
			angle += 360;
		 else if (angle >= 360)
			angle -= 360;
		 if (angle >=180)
			angle = angle - 360;

		 angle_corrected = angle; 		//angle_corrected is the value of angle in the range 0-359 deg
		 
		 //Calculate combined angle 
		 //co-efficients 0.974 and 0.026 have been determined experimentally
		 combined_angle= 0.974*(angle_corrected)+0.026*(pitch) ;	    
	  }
	  
	  

	// Process-2: encoder reading every 'interval2' time (here interval2 = 15 mSec ) 

    if ((millis() - task2_time) >= interval2)
    {
		int current_count_m1, current_count_m2;     

		task2_time = millis();		// mark current time so that remember when to take next sample

		// get the current count(speed & direction) of motor1 & motor2
		current_count_m1 = getCountsAndResetM1(); 
		current_count_m2 = getCountsAndResetM2(); 
	  
		//calculate speed of both motors
		speed_m1 = (current_count_m1 * 1000.0)/(840*interval2);	
		speed_m2 = (current_count_m2 * 1000.0)/(840*interval2);

		//Calculate combined speed using moving average filter method
		speed_combined = speedMovingAverageFilter.process((speed_m1+speed_m2)/2);
   
    }
      

 
	// Process-3: angle and speed PID loop run continuously	
	
	//If error within the dead band stop PID action, This is to minimize oscillation
	 if ( ((anglePIDSetpoint-combined_angle)<0.20) && ((anglePIDSetpoint-combined_angle)>-0.20) )
	 {   
		anglePID.SetMode(MANUAL);	//turn  PID off  
	 }
	 //else run the PID loops to determine required motor speed 
	 else 
	 {	   
		anglePID.SetMode(AUTOMATIC);
		speedPID.SetMode(AUTOMATIC);
		speedPID.SetTunings(speedKp, speedKi, speedKd);
		anglePID.SetTunings(angleKp, angleKi, angleKd);
	 }
	 
	 speedPIDSetpoint = throttle/10.0;			//speed set point adaptive tuning
	 speedPIDInput = speed_combined; 			// current speed input for PID calculation 
	 speedPID.Compute(); 						// compute new speed using PID function
	 
	 anglePIDInput=combined_angle;				//current angle as input
	 anglePIDSetpoint= 4.5 + speedPIDOutput; 	// angle set point adaptive tuning
	 anglePID.Compute();  						// compute new angle

	 speed = (int) anglePIDOutput;				//Compute final speed for the motors


	if (steering == 0) 
	{
		motor1SteeringOffset = 0;
		motor2SteeringOffset = 0;
	}	
	else if (steering < 0) 
	{
		// anti-clock rotation
		motor1SteeringOffset = steering;
		motor2SteeringOffset = -steering;
	}
	else 
	{
		// clock-wise rotation
		motor1SteeringOffset = steering;
		motor2SteeringOffset = -steering;
	}

	//Control the speed of motors as per the PID calculation output
	if (((anglePIDSetpoint-combined_angle)>0.20) || ((anglePIDSetpoint-combined_angle)<-0.20))
	 {
		velocity_move(1, speed+motor1SteeringOffset);
		velocity_move(2, speed+motor2SteeringOffset);
	 }
	 else
	 {
		anglePIDOutput = 0; 		//output to zero in the dead band
		velocity_move(1, motor1SteeringOffset);
		velocity_move(2, motor2SteeringOffset);
	 }
	 
	 //Stop the motor if error is beyond 45 deg
	 //This is because more than this threshold cannot be corrected 
	 //and this time motors driven at full speed even if the robot fell down
	 if ((combined_angle>45)||(combined_angle<-45))
	 {
		stop();
	 }
	 
	 if (motor_on == 0)
	 {
		stop();
	 }
 

	// Process-4: Xbee communiation for remote commanding the robot and arm movement control
	//check for commands every 100 ms
	if(millis() - task4_time >= interval4)
	{
		task4_time = millis();		
		
		if (Serial.available())
		{
			data_recv = Serial.read(); 	// read the serial data from Xbee
			
			if((data_recv == 'G') || (data_recv == 'g'))  // open the gripper on receiving 'G' or 'g' from Xbee-terminal
			{
				grip_servoangle=grip_servoangle+6; 		// open the gripper angle in step of app 1 deg/100mS
				if(grip_servoangle>1850)  				//Limit the maximum angle of gripper movement
				{
					grip_servoangle=1850;
				}
				gripservo.writeMicroseconds(grip_servoangle);
			
			}
			else if((data_recv == 'H') || (data_recv == 'h'))	// If gripper close command from Xbee
			{	
				grip_servoangle=grip_servoangle-6; 			// close gripper in step of 1 deg/ 100mS
				if(grip_servoangle<250) 					//Limit minimum angle of gripper movement
				{
					grip_servoangle=250;
				}
				gripservo.writeMicroseconds(grip_servoangle);				
			}
			
			if((data_recv == 'A') || (data_recv == 'a')) // if Move arm up command from Xbee			
			{				
				arm_servoangle=arm_servoangle+6;  			//move arm up in step of 1 deg/ 100mS 
				if(arm_servoangle>1800) 					//Limit maximum arm movement
				{
					arm_servoangle=1800;
				}
				armservo.writeMicroseconds(arm_servoangle);				
			}
			else if((data_recv == 'B') || (data_recv == 'b')) // if Move arm down command from Xbee
			{
				
				arm_servoangle=arm_servoangle-6; 		//move arm down in step of 1 deg/ 100mS 
				if(arm_servoangle<1350) /				//Limit maximum arm movement
				{
					arm_servoangle=1350;
				}
				armservo.writeMicroseconds(arm_servoangle);		
			}
				
			if (data_recv == '1')	// if Move robot fwd command from Xbee			
			{
				throttle++;			//Move forward	
				if (throttle>6) 	//Limit max speed
				{
				   throttle = 6;
				}
			}  
			else if (data_recv == '2')	// if Move robot backward command from Xbee
			{
				throttle--;			//Move backward	
				if (throttle<-6) 	//Limit max speed
				{
				   throttle = -6;
				}				
			}  
  
			if(data_recv == '4')		// if Move robot left command from Xbee
			{
				steering++;
			}
			else if(data_recv == '8')  // if Move robot right command from Xbee
			{
				steering--;
			}		  
		}
	}
}
