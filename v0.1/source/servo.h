/*
** servo.h
**
*/
#ifndef _SERVO_H_
#define _SERVO_H_

 
 // Some macros that make the code more readable
 #define output_low(port,pin) port &= ~(1<<pin)
 #define output_high(port,pin) port |= (1<<pin)
 #define set_input(portdir,pin) portdir &= ~(1<<pin)
 #define set_output(portdir,pin) portdir |= (1<<pin) 

void servo_init(void);
void servo_set(uint16_t p);// Function Declarations
double ball_pos_calc (double voltage);
uint16_t servo_target (double angle) ;

#endif



