/*	SERVO motor & calibration code
	For an Atmega 88
*/	


#include <avr/io.h>
#include <util/delay.h>
#include "servo.h"


uint16_t servo_max = 1983;
uint16_t servo_min = 1103;
double servo_target_value, servo_center_value;
const static double beam_length = 33.5;
const static double voltage_min = 0.35;
const static double voltage_max = 0.57;


uint16_t rand_pos;
uint8_t new_pos;
int j;

void servo_set(uint16_t p) {
uint16_t pos = 0.5*(servo_max - servo_min)*(p/32767.0) +0.5*servo_min;
OCR1A = pos;
}


uint16_t servo_target (double angle) {
double intermediate = 32767.0*(-46.2963*(angle - 33.338) - (double) servo_min)/((double)(servo_max-servo_min));
if (intermediate < 0.0) intermediate = 0.0;
if (intermediate > 32767) intermediate = 32767.0;
return (uint16_t)intermediate;
}

double ball_pos_calc (double voltage) {
return beam_length*(voltage-voltage_max)/(voltage_min-voltage_max);
}

void servo_init(void){
//set_output(DDRB, PB1);  
DDRB |= (1<<PB1);
ICR1 = 20000;
TCCR1A = (0<<WGM10) | (1<<WGM11) | (1<<COM1A1);
TCCR1B = (1<<WGM12) | (1<<WGM13) | (1<<CS11);

servo_center_value = 0.5*(servo_max+servo_min);
servo_target_value = 32767*(servo_center_value-servo_min)/(servo_max-servo_min);
servo_set((uint16_t)servo_target_value);
_delay_ms(1000);

}



