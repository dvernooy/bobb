/*
** adc.h
**
*/
#ifndef _ADC_H_
#define _ADC_H_


#include <inttypes.h>

 
 // Some macros that make the code more readable
 #define output_low(port,pin) port &= ~(1<<pin)
 #define output_high(port,pin) port |= (1<<pin)
 #define set_input(portdir,pin) portdir &= ~(1<<pin)
 #define set_output(portdir,pin) portdir |= (1<<pin) 

void adc_init(void);
uint32_t read_adc(void);		// Function Declarations

#endif



