/*
** main.c
** PID code with physical units working 12/31/13
** sample/loop time 60 ms

**ISSUES:
1. sometimes doesn't measure ADC voltage on startup?
2. ADC needs a delay ADC_delay to work properly ...
3. Too much physical noise in beam measurement ... need an amp

UPDATES: 4/27/15
1. make adc reference 0.65V, but add more resolution & clean up signal
2. added extra filtering in the loop to get rid of any spikes
**
*/

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
#define ADC_REFV 0.6 //ADC reference voltage

const static double beam_length = 33.5;
const static int ADC_delay = 25; //anything lower than this & it crashes
const static int ADC_avg_counts = 1000; //can play with this a bit ... # of averages to get a good signal
double ball_position,ball_position_old,ball_target_position, voltage_level,ball_velocity,P_gain,D_gain, LED_level;
double beam_target_angle;
int l,wait,setup, manual;
unsigned int result;
double ADC_sum, ADC_sum_old;
TIME t;
double elapsed_ms; 
int k;
uint16_t rand_pos, set_point,j, ADC_counter, adc_value_read;
uint32_t elapsed_time, count, temp_sum,jj;

/********************************************************************************
                                Main
********************************************************************************/
int main(void)
{	
    	  
	// Setup LCD
	lcd_init();
	lcd_contrast(0x3A);
	
	//setup ADC
	adc_init();	// Initialize the ADC		
	_delay_ms(5);	// wait a bit

	
    //setup switches
	switchio_init();	// Initialize the switches		
	_delay_ms(5);	// wait a bit

    //setup servo
       servo_init();
	           
	//setup LEDs
   // DDRD = 0xFF;
	//PORTD = ~0x00;
	LED_level = beam_length/8.0;
	
	//setup timer
	TimeInit();	//start timer routine
	sei(); 	// enable interrupts
    connect_timer(1);
	GetTime(&t);
	
    // setup the screen
	lcd_goto_xy(1,1);
	fprintf_P(&lcd_out,PSTR("BOBBy SAYS:"));
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

 
P_gain = 0.85; //0.85
D_gain = 1.15; //1.15

 
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
    ADC_counter = 0;	
	for (k = 0;k<ADC_avg_counts;k++)
	{
	//delay code, seems to do the trick;
	for (l = 0;l<ADC_delay;l++) {}
	read_adc();
	if ((result < 10) || (result > 1000))
	{}
	else {
	temp_sum += result;
	ADC_counter++;
	}
	}
	if (ADC_counter <10) 
	{ADC_sum = ADC_sum_old;}
	
	else
	{ADC_sum = (double) temp_sum;
	ADC_sum = ADC_sum/ADC_counter;
	ADC_sum_old = ADC_sum;
	}
    ball_position_old = ball_position; //old position
    voltage_level = ADC_sum*(ADC_REFV/1023.0);
    elapsed_ms = (double)GetElaspMs(&t);//get elapsed time of loop
	GetTime(&t);
	lcd_goto_xy(1,2);
	fprintf_P(&lcd_out,PSTR("%-5.3f V"),voltage_level);
    ball_position = ball_pos_calc(voltage_level);     		
	lcd_goto_xy(1,3);
	fprintf_P(&lcd_out,PSTR("%-5.1f cm"),ball_position);

	
	ball_velocity = (ball_position-ball_position_old)/(elapsed_ms/1000.0);
	if (abs(ball_velocity) < 0.001) ball_velocity = 0;
	lcd_goto_xy(1,5);
    fprintf_P(&lcd_out,PSTR("%-8.2f"),ball_velocity);
    lcd_goto_xy(1,6);
    fprintf_P(&lcd_out,PSTR("%-6.0f"),ADC_sum);
 }
 }
 
  manual = 0;
 if (manual == 1)
 {
  //setup ADC
  	//setup ADC
 	adc_init_read();// Initialize the ADC		
	_delay_ms(5);	// wait a bit

	while (1)
	{
	//read ADC
	adc_value_read = read_adc();
	lcd_goto_xy(1,4);    
	fprintf_P(&lcd_out,PSTR("%4d"),adc_value_read);
	//write ADC
	//set servo
	jj = (uint32_t)adc_value_read*32767/1023;
	lcd_goto_xy(1,5);    
	fprintf_P(&lcd_out,PSTR("%5d"),jj);
	 servo_set(jj); 	//j between 0 and 32767 
	}
 }
 
        lcd_goto_xy(1,1);
        fprintf_P(&lcd_out,PSTR("TARGET:"));
 	    lcd_goto_xy(8,2);
        fprintf_P(&lcd_out,PSTR("cm"));
        //setup the initial condition - first read the position
		temp_sum = 0;
	    ADC_counter = 0;	
		for (k = 0;k<ADC_avg_counts;k++) {
			for (l = 0;l<ADC_delay;l++) {} //delay code, seems to do the trick;
			read_adc();
				if ((result < 509) || (result > 974))
					{}
				else {
					temp_sum += result;
					ADC_counter++;
				}
		}
		if (ADC_counter < 0.9*ADC_avg_counts) 
		{ADC_sum = ADC_sum_old;}
		else
		{ADC_sum = (double) temp_sum;
		ADC_sum = ADC_sum/ADC_counter;
		ADC_sum_old = ADC_sum;
		}		
		voltage_level = ADC_sum*(ADC_REFV/1023.0); // current position
		ball_position = ball_pos_calc(voltage_level);
        ball_target_position = beam_length/2.0;
        lcd_goto_xy(1,2);
        fprintf_P(&lcd_out,PSTR("%-5.1f"),ball_target_position); 	 
        GetTime(&t); //start timer
    
count = 0;

while (1) //main loop for servo
{   

	//ball_target_position = beam_length/2.0; //use this code to servo to an absolute position along beam

    
	if (count > 100) {
	count = 0;
	_delay_ms(100);


	if (ball_target_position < 20.0) 
	{ball_target_position = 26.5;}
	else {ball_target_position = 7.0;}
  

    lcd_goto_xy(1,2);
    fprintf_P(&lcd_out,PSTR("%-5.1f"),ball_target_position); 	
	lcd_goto_xy(1,4);
    fprintf_P(&lcd_out,PSTR("%-5.1f"),elapsed_ms); 	
	 }
	

	 
    //reset variables for new loop
	ball_position_old = ball_position; //old position
	
	//Measure the position in cm along the beam
		temp_sum = 0;
        ADC_counter = 0;
		for (k = 0;k<ADC_avg_counts;k++) {
			for (l = 0;l<ADC_delay;l++) {} //delay code, seems to do the trick;
			read_adc();
			if ((result < 509) || (result > 974))
					{}
				else {
					temp_sum += result;
					ADC_counter++;
				}
			}
			if (ADC_counter < 0.9*ADC_avg_counts) 
			{ADC_sum = ADC_sum_old;}
			else
			{ADC_sum = (double) temp_sum;
			ADC_sum = ADC_sum/ADC_counter;
			ADC_sum_old = ADC_sum;
			}		
		voltage_level = ADC_sum*(ADC_REFV/1023.0); // current position
		ball_position = ball_pos_calc(voltage_level);     	
	elapsed_ms = (double)GetElaspMs(&t);//get elapsed time of loop in ms
	GetTime(&t);//reset timer
	
/*	
	//Optional LED output of position
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
			
    //calculate the ball velocity in cm/s
	ball_velocity = (ball_position-ball_position_old)/(elapsed_ms/1000.0);
	if (abs(ball_velocity) < 0.001) ball_velocity = 0.0;

	//start with a flat beam
	beam_target_angle = 0.0;	
	
   //Proportional Term: Based on position error 
	beam_target_angle=beam_target_angle+P_gain*9.0*((ball_position-ball_target_position)/(beam_length/2.0)); 
   //Derivative Term: Based on velocity  
    beam_target_angle=beam_target_angle+D_gain*9.0*((ball_velocity)/(beam_length/1.0)); //normalized to traveling beam in 1s	
	 
   //tell servo to go to new setpoint 
     j = servo_target(beam_target_angle);
	 servo_set(j); 	 
	 count++;
}
	
}



   
