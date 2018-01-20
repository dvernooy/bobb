/*
** lcd.h
**
** LCD 3310 driver
** Target: ATMEGA :: AVR-GCC
**
** Written by Tony Myatt - 2007
** Quantum Torque - www.quantumtorque.com
*/
#ifndef _LCD_H_
#define _LCD_H_

/* Lcd screen size */
#define LCD_X_RES 84
#define LCD_Y_RES 48
#define LCD_CACHE_SIZE ((LCD_X_RES * LCD_Y_RES) / 8)


/* Pinout for LCD */
#define LCD_CLK_PIN 	(1<<PD0)
#define LCD_DATA_PIN 	(1<<PD1)
#define LCD_DC_PIN 		(1<<PD2)
#define LCD_CE_PIN 		(1<<PD3)
#define LCD_RST_PIN 	(1<<PD4)
#define LCD_PORT		PORTD
#define LCD_DDR			DDRD

/* Special Chars */
#define ARROW_RIGHT	ICON(0)
#define ARROW_LEFT 	ICON(1)
#define ARROW_UP 	ICON(2)
#define ARROW_DOWN 	ICON(3)
#define STOP 		ICON(4)
#define PLAY	 	ICON(5)

/* Function for my special characters */
#define	ICON(x)		'z'+1+x

void lcd_init(void);
void lcd_contrast(unsigned char contrast);
void lcd_clear(void);
void lcd_clear_area(unsigned char line, unsigned char startX, unsigned char endX);
void lcd_clear_line(unsigned char line);
void lcd_goto_xy(unsigned char x, unsigned char y);
void lcd_goto_xy_exact(unsigned char x, unsigned char y);
void lcd_chr(char chr);
void lcd_int(unsigned int chr);
void lcd_str(char* str);
int lcd_chr_printf(char chr, FILE *stream);
#endif



