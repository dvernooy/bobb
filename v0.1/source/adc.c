/*	ADC Test
	For an Atmega 88
	Atmega88 DIP ADC5 PC5 (PIN28) */
	
#include "adc.h"
#include <avr/io.h>

uint32_t result;

/* INIT ADC */
void adc_init(void)
{
	/** Setup and enable ADC **/
	ADMUX = (0<<REFS1)|	// Reference Selection Bits
			(0<<REFS0)|		// AVcc - external cap at AREF
			(0<<ADLAR)|		// ADC Left Adjust Result
			(1<<MUX2)|		// ANalog Channel Selection Bits
			(0<<MUX1)|		// ADC5 (PC5 PIN28)
			(1<<MUX0);
	
	ADCSRA = (1<<ADEN)|	// ADC ENable
			(0<<ADSC)|		// ADC Start Conversion
			(0<<ADATE)|		// ADC Auto Trigger Enable
			(0<<ADIF)|		// ADC Interrupt Flag
			(0<<ADIE)|		// ADC Interrupt Enable
			(1<<ADPS2)|		// ADC Prescaler Select Bits
			(0<<ADPS1)|
			(0<<ADPS0);
							// Timer/Counter1 Interrupt Mask Register
//	TIMSK1 |= (1<<TOIE1);	// enable overflow interrupt
	
	
//	TCCR1B |= (1<<CS11)|
//			(1<<CS10);  // native clock
}


/* READ ADC PINS */
uint32_t read_adc(void) {
		ADCSRA |= (1<<ADSC);
		while(ADCSRA & (1<<ADIF)) {};
		result = ADC;
		ADCSRA |= (1<<ADIF);
    return result;
}

 













