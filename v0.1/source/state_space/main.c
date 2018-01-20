/*
** main.c
** state space controller [x x-dot] & u = angle with physical units 
** -- includes the following control designs:
** controller & observer using pole placement
** controller using steady state LQR & observer using steady state Kalman filter
**
** matlab file digitalBOBB.m calculates the controller
** sample/loop time 8 ms with some initial filtering

**ISSUES with setup:
1. sometimes doesn't measure ADC voltage on startup?
---> has bee working lately
2. ADC needs a delay ADC_delay to work properly ...
---> need to research this
3. Too much physical noise in beam measurement ... need an amp
---> use LM317 regulator with large filter caps & then op-amp to extend range.
this should also eliminate the digital noise in the position measurement
right now only using ~100 mV out of 700, or 145 levels out of 1023 ... so 2mm resolution.
Should be able to do 10 times better.
**
*/

/*****connections:
ATMega88
FUSES: SPIEN, EESAVE
BRown out disabled
Bootflash 128, address $0f80
SUTCDSEL Ext xtal osc. 3 - 8 MHz, startup PWRDN/RESEt 258CK/
Vtarget 5V
Aref 0.7V
PB1 -- servo output
PC5 -- ADC input
PC0 -- RST 
PC1 -- CEL
PC2 -- D/L
PC3 -- MOSI
PC4 -- CLK
PDxx -- LEDs
PB2, PB3 - switches
VCC & GND ...
**************/



#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include "lcd.h"
#include "switchio.h"
#include "adc.h"
#include "servo.h"
#include "time.h"
#include "datatypes.h"

/********************************************************************************
Global Variables
********************************************************************************/

//static FILE usart_out = FDEV_SETUP_STREAM(usart_putchar_printf, usart_getchar_printf, _FDEV_SETUP_RW);
static FILE lcd_out = FDEV_SETUP_STREAM(lcd_chr_printf, NULL, _FDEV_SETUP_WRITE);

// defines
//#define F_CPU 4000000UL

const static double beam_length = 33.5;
const static int ADC_delay = 25;
const static int ADC_avg_counts = 100;
double ball_position,ball_target_position, voltage_level,LED_level;
double est_ball_position, est_ball_velocity;
double beam_target_angle;
int l,wait,setup;
unsigned int result;
double ADC_sum;
TIME t;
double elapsed_ms; 
int k;
uint16_t rand_pos, set_point,j;
uint32_t elapsed_time, count, temp_sum;
/*definitions
double M1[2][2] //phi-gamma*K-L*H*phi+L*H*gamma*K 
double M3[2] //(gamma+L*H*gamm)*Nbar
double KK[2] //K
*/


//no observer, no love
/*
double M1[2][2] = {{0.9998,0.0080},{-0.0403,0.9891}};
double L[2] = {0.0000,0.0000};
double M3[2] = {0.0002,0.0403};
double KK[2] = {-0.5414,-0.1465};
double N_bar = -0.5414;
*/


//designed by pole placement, with L poles = 3X K poles
//excellent performance
//starting point: digitalBOBBv2(0.44,0.77,3.8,0,3,1,0,3,1,1,1)
//ending point: digitalBOBBv2(0.55,0.85,3.5,0,3,1,0,3,1,1,1)
/*
double M1[2][2] = {{0.8477,0.0066},{-1.0548,0.9485}};
double L[2] = {0.1520,0.9713};
double M3[2] = {0.0004,0.0841};
double KK[2] = {-1.1264,-0.5896};
double N_bar = -1.1264;
*/

//designed by dlqe/kalman
//
//starting point: digitalBOBBv2(0.55,0.85,3.5,1,3,1,1,3,1,0.1,1)
//ending point: digitalBOBBv2(0.55,0.85,3.5,1,1,2,1,3,1,0.1,0.9)

double M1[2][2] = {{0.7947,0.0063},{-1.4413,0.9602}};
double L[2] = {0.2051,1.3898};
double M3[2] = {0.0002,0.0521};
double KK[2] = {-0.6969,-0.3871};
double N_bar = -0.6969;


double state_vec[2] = {0.0, 0.0};

/********************************************************************************
                                Main
********************************************************************************/
int main(void)
{	
    	  
	// Setup LCD
	lcd_init();
	lcd_contrast(0x36);
	
	//setup ADC
	adc_init();	// Initialize the ADC		
	_delay_ms(5);	// wait a bit

	
    //setup switches
	switchio_init();	// Initialize the switches		
	_delay_ms(5);	// wait a bit

    //setup servo
       servo_init();
	           
	//setup LEDs
    //DDRD = 0xFF;
	//PORTD = ~0x00;
	LED_level = beam_length/8.0;
	
	//setup timer
	TimeInit();	//start timer routine
	sei(); 	// enable interrupts
    connect_timer(1);
	GetTime(&t);
	
    // setup the screen
	lcd_goto_xy(1,1);
	fprintf_P(&lcd_out,PSTR("BOBB SAYS:"));
    lcd_goto_xy(1,2);
	fprintf_P(&lcd_out,PSTR("HAVE FUN!"));

	lcd_goto_xy(1,4);
	fprintf_P(&lcd_out,PSTR("Press Key>>"));
	
	wait = 1;	
         while (wait)                       
        {
		   if (switch_is_pressed())
                {
                        lcd_clear();
						wait = 0;
                }
        }

 
 
 //setup = 1 to calibrate ... set values in servo.c
 setup = 0;
 if (setup == 1)
 {
 lcd_clear();
 _delay_ms(1000);
 lcd_goto_xy(1,1);
 fprintf_P(&lcd_out,PSTR("POSITION"));
 lcd_goto_xy(1,4);
 fprintf_P(&lcd_out,PSTR("VELOCITY"));
 lcd_goto_xy(9,5);
 fprintf_P(&lcd_out,PSTR("cm/s"));    	
 ball_position = 33.5/2.0;
 GetTime(&t);
 
 while (1)
 {
    temp_sum = 0;	
	for (k = 0;k<ADC_avg_counts;k++)
	{
	//delay code, seems to do the trick;
	for (l = 0;l<ADC_delay;l++) {}
	read_adc();
	temp_sum += result;
	}
	ADC_sum = (double) temp_sum;
    voltage_level = ADC_sum*((0.6/ADC_avg_counts)/1023.0);
    elapsed_ms = (double)GetElaspMs(&t);//get elapsed time of loop
	GetTime(&t);
	lcd_goto_xy(1,2);
	fprintf_P(&lcd_out,PSTR("%-5.3f V"),voltage_level);
    ball_position = ball_pos_calc(voltage_level);     		
	lcd_goto_xy(1,3);
	fprintf_P(&lcd_out,PSTR("%-5.1f cm"),ball_position);

    lcd_goto_xy(1,6);
    fprintf_P(&lcd_out,PSTR("%-6.0f"),elapsed_ms);
 }
 }

//setup code:initial conditions - first read the position
		temp_sum = 0;	
		for (k = 0;k<ADC_avg_counts;k++) {
			for (l = 0;l<ADC_delay;l++) {} //delay code, seems to do the trick;
			read_adc();
			temp_sum += result;
	        }
	    ADC_sum = (double) temp_sum;
		voltage_level = ADC_sum*((0.6/ADC_avg_counts)/1023.0); // current position
		ball_position = ball_pos_calc(voltage_level);
	    state_vec[0] = ball_position; //find initial position
    	state_vec[1] = 0.0;//assume it is not moving
		//ball_target_position = beam_length/2.0; //use this code to servo to an absolute position along beam	

        ball_target_position = 27.0;
        lcd_goto_xy(1,1);
        fprintf_P(&lcd_out,PSTR("TARGET:"));
 	    lcd_goto_xy(8,2);
        fprintf_P(&lcd_out,PSTR("cm"));
        lcd_goto_xy(1,2);
        fprintf_P(&lcd_out,PSTR("%-5.1f"),ball_target_position); 	 
        GetTime(&t); //start timer
    
count = 0;

while (1) //main loop for servo
{   

    
	if (count > 400) {
	count = 0;
	if (ball_target_position < 20.0) 
	{ball_target_position = 27.0;}
	else {ball_target_position = 6.5;}
    lcd_goto_xy(1,2);
    fprintf_P(&lcd_out,PSTR("%-5.1f"),ball_target_position); 	
	lcd_goto_xy(1,4);
    fprintf_P(&lcd_out,PSTR("%-5.1f"),elapsed_ms); 	
	 }
	
	 	
	//Measure the position in cm along the beam
		temp_sum = 0;	
		for (k = 0;k<ADC_avg_counts;k++) {
			for (l = 0;l<ADC_delay;l++) {} //delay code, seems to do the trick;
			read_adc();
			temp_sum += result;
		}
		ADC_sum = (double) temp_sum;
		voltage_level = ADC_sum*((0.6/ADC_avg_counts)/1023.0); // current position
		ball_position = ball_pos_calc(voltage_level);     	
	elapsed_ms = (double)GetElaspMs(&t);//get elapsed time of loop in ms
	GetTime(&t);//reset timer
	
	
/*	//Optional LED output of position
	if((ball_position>0.0) && (ball_position<=LED_level)) 
	   {PORTD = ~0x01;}
	     else if ((ball_position>LED_level) && (ball_position<=(2.0*LED_level))) 
			{PORTD = ~0x02;} 
			else if ((ball_position>(2.0*LED_level)) && (ball_position<=(3.0*LED_level))) 
				{PORTD = ~0x04;}
            else if ((ball_position>(3.0*LED_level)) && (ball_position<=(4.0*LED_level))) 
				{PORTD = ~0x08;} 
            else if ((ball_position>(4.0*LED_level)) && (ball_position<=(5.0*LED_level))) 
				{PORTD = ~0x10;}
			else if ((ball_position>(5.0*LED_level)) && (ball_position<=(6.0*LED_level))) 
				{PORTD = ~0x20;}
		    else if ((ball_position>(6.0*LED_level)) && (ball_position<=(7.0*LED_level))) 
				{PORTD = ~0x40;}
			else if ((ball_position>(7.0*LED_level)) && (ball_position<=(8.0*LED_level))) 
				{PORTD = 0x7F;}	
*/	
	
/* FOR REFERENCE, THIS IS THE OLD PID CONTROLLER CODE		
    //calculate the ball velocity in cm/s
	ball_velocity = (ball_position-ball_position_old)/(elapsed_ms/1000.0);
	if (abs(ball_velocity) < 0.001) ball_velocity = 0.0;

	//start with a flat beam
	beam_target_angle = 0.0;	
	
   //Proportional Term: Based on position error 
	beam_target_angle=beam_target_angle+P_gain*9.0*((ball_position-ball_target_position)/(beam_length/2.0)); 
   //Derivative Term: Based on velocity  
    beam_target_angle=beam_target_angle+D_gain*9.0*((ball_velocity)/(beam_length/1.0)); //normalized to traveling beam in 1s	
*/

   //implement state space controller
    est_ball_position = M1[0][0]*state_vec[0]+M1[0][1]*state_vec[1]+L[0]*ball_position+M3[0]*ball_target_position;
	est_ball_velocity = M1[1][0]*state_vec[0]+M1[1][1]*state_vec[1]+L[1]*ball_position+M3[1]*ball_target_position;
	state_vec[0] = est_ball_position;
	state_vec[1] = est_ball_velocity;
	beam_target_angle =-1*KK[0]*state_vec[0]-1*KK[1]*state_vec[1]+N_bar*ball_target_position;
	 
   //tell servo to go to new setpoint 
     j = servo_target(beam_target_angle);
	 servo_set(j); 	 
	 count++;
}
	
}



   
