/*
 * lcd.c
 *
 *  Created on: Jan 11, 2025
 *      Author: hp
 */

#include "lcd.h"

/* some private helper functions prototypes */
static void write_4_bits(uint8_t value);
static void lcd_enable(void);



void lcd_send_command(uint8_t cmd){

	/* RS = 0 for lcd commands */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);

	/* RW = 0, for write */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	write_4_bits(cmd >> 4);
	write_4_bits(cmd & 0x0F);
}

/*
 * This function sends a character to the lcd, here we used 4 bit data transmission
 * First higher nibbles of data, send on the data lines D4, D5, D6, D7
 * Then lower nibbles of data, will be send on RS, RS
 */

void lcd_print_char(uint8_t data){
	/* RS = 1, for LCD user data */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_SET);

	/* RW = 0, for write */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_SET);

	/* higher nibble */
	write_4_bits(data >> 4);
	/* lower nibble */
	write_4_bits(data & 0x0F);
}

void lcd_print_string(char *message){

	do{
		lcd_print_char((uint8_t)*message);

	}while(*message != '\0');
}


void lcd_init(void){
	//1. Configure the gpio pins which are used for lcd connections
	GPIO_Handle_t lcd_signal;

	lcd_signal.pGPIOx = LCD_GPIO_PORT;
	lcd_signal.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RS;
	lcd_signal.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	lcd_signal.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NOPUPD;
	lcd_signal.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RW;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_EN;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D4;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D5;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D6;
	GPIO_Init(&lcd_signal);

	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D7;
	GPIO_Init(&lcd_signal);

	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, GPIO_PIN_RESET);

	//2. do the lcd initialization
	mdelay(40);

	/* RS = 0, for LCD Command */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);

	/* RW = 0, Writing to LCD */
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	write_4_bits(0x3);

	mdelay(5);

	write_4_bits(0x3);

	udelay(150);

	write_4_bits(0x3);

	write_4_bits(0x2);

	/* function set command */
	lcd_send_command(LCD_CMD_4DL_2N_5X8F);

	/* display on and cursor on */
	lcd_send_command(LCD_CMD_4DL_2N_5X8F);

	lcd_display_clear();

	/* entry mode set */
	lcd_send_command(LCD_CMD_INCADD);
}

/* writes 4 bits of data/command on to D4, D5, D6 and D7 */
static void write_4_bits(uint8_t value){

	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, ((value >> 0) & 0x1));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, ((value >> 1) & 0x2));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, ((value >> 2) & 0x3));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, ((value >> 3) & 0x4));

	lcd_enable();

}

void lcd_display_clear(void){

	lcd_send_command(LCD_CMD_DIS_CLEAR);
	/*
	 * check page number 24, display clear command is 24 and return home command execution is
	 * wait time around 2 ms
	 */
	mdelay(2);
}

/* cursor return home position */

void lcd_display_return_home(void){

	lcd_send_command(LCD_CMD_DIS_RETURN_HOME);
	/*
	* check page number 24, display clear command is 24 and return home command execution is
	* wait time around 2 ms
	*/
	mdelay(2);

}

 /*
  *   Set Lcd to a specified location given by row and column information
  *   Row Number (1 to 2)
  *   Column Number (1 to 16) Assuming a 2 X 16 characters display
  */

void lcd_set_cursor(uint8_t row, uint8_t column){

	column--;
	switch(row){
	case 1:
		/* set cursor to 1st row and add index */
		lcd_send_command((column |= 0x80));
		break;
	case 2:
		/* set cursor to 2nd row address and add index */
		lcd_send_command((column |= 0xC0));
		break;
	default:
		break;

	}
}

static void lcd_enable(void){

	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_SET);
	udelay(10);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	udelay(100); /* execution time is 34 micro seconds */
}

void mdelay(uint32_t cnt){

	for(uint32_t i = 0; i < (cnt * 1000); i++);
}

void udelay(uint32_t cnt){
	for(uint32_t i = 0; i < (cnt * 1); i++);
}

