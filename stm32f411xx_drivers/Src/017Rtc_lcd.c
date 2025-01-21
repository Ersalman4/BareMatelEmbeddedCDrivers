/*
 * 017Rtc_lcd.c
 *
 *  Created on: Jan 11, 2025
 *      Author: hp
 */
#include <stdio.h>
#include "lcd.h"
#include "DS1307.h"

#define  SYSTICK_TIM_CLK		1600000U

//enable this macros to test the project on the hardware
#define PRINT_LCD


void number_to_string(uint8_t num, char* buff){

	if(num < 10){
		buff[0] = '0';
		buff[1] = num + 48;
	}else if(num >= 10 && num < 99){
		buff[0] = ((num / 10) + 48);
		buff[1] = ((num % 10) + 48);
	}
}


char* get_day_of_week(uint8_t i){
	char* days[] = {"SUNDAY", "MONDAY", "TUESDAY", "WEDNESDAY", "THURSDAY", "FRIDAY", "SATURDAY"};

	return days[i - 1];
}

char* time_to_string(RTC_time_t *rtc_time){
	//format would be HH:MM:SS
	static char buff[9];
	buff[2] = ':';
	buff[5] = ':';
	number_to_string(rtc_time->hours, buff);
	number_to_string(rtc_time->minutes, &buff[3]);
	number_to_string(rtc_time->seconds, &buff[6]);

	buff[8] = '\0';

	return buff;
}



char* date_to_string(RTC_date_t *rtc_date){
	static char buff[9];
	buff[2] = '/';
	buff[5] = '/';

	number_to_string(rtc_date->date, buff);
	number_to_string(rtc_date->month, &buff[3]);
	number_to_string(rtc_date->year, &buff[6]);

	buff[8] = '\0';

	return buff;

}

void init_systick_timer(uint32_t tick_hz)
{
	uint32_t *pSRVR = (uint32_t*)0xE000E014;
	uint32_t *pSCSR = (uint32_t*)0xE000E010;

    /* calculation of reload value */
    uint32_t count_value = (SYSTICK_TIM_CLK/tick_hz)-1;

    //Clear the value of SVR
    *pSRVR &= ~(0x00FFFFFFFF);

    //load the value in to SVR
    *pSRVR |= count_value;

    //do some settings
    *pSCSR |= ( 1 << 1); //Enables SysTick exception request:
    *pSCSR |= ( 1 << 2);  //Indicates the clock source, processor clock source

    //enable the systick
    *pSCSR |= ( 1 << 0); //enables the counter

}

int main(void){

	printf("RTC Testing....\n\r");

	RTC_time_t current_time;
	RTC_date_t current_date;
#ifndef PRINT_LCD
	printf("RTC Testing..\n");
#else
	lcd_init();

	lcd_print_string("RTC Testing...\n");
	mdelay(200);

	lcd_display_clear();
	lcd_display_return_home();
#endif

	if(ds1307_init()){
		printf("RTC initialization failed....\n");
		while(1);
	}
	init_systick_timer(1);

	current_date.day = SUNDAY;
	current_date.date = 12;
	current_date.month = 1;
	current_date.year = 25;

	current_time.hours = 2;
	current_time.minutes = 42;
	current_time.seconds = 25;
	current_time.time_format = TIME_FORMAT_12HRS_PM;

	ds1307_set_current_date(&current_date);
	ds1307_set_current_time(&current_time);

	//init_systick_timer(1);

	ds1307_get_current_date(&current_date);
	ds1307_get_current_time(&current_time);

	char *am_pm;
	if(current_time.time_format != TIME_FORMAT_24HRS){
		am_pm = (current_time.time_format) ? "PM" : "AM";

#ifndef PRINT_LCD
		printf("Current Time = %s %s\n", time_to_string(&current_time), am_pm);
#else
		lcd_print_string(time_to_string(&current_time));
		lcd_print_string(am_pm);
#endif

	}else{

#ifndef PRINT_LCD
		printf("Current Time = %s\n", time_to_string(&current_time));
#else
		//lcd_set_cursor(2, 1);
		lcd_print_string(time_to_string(&current_time));
#endif
	}

#ifndef PRINT_LCD
		printf("Current Date = %s <%s>\n", date_to_string(&current_date), get_day_of_week(current_date.day));
#else
		lcd_set_cursor(2, 1);
		lcd_print_string(date_to_string(&current_date));
#endif

	while(1);

	return 0;
}


void SysTick_Handler(){

	RTC_time_t current_time;
	RTC_date_t current_date;

	ds1307_get_current_time(&current_time);
	char *am_pm;
	if(current_time.time_format != TIME_FORMAT_24HRS){
		am_pm = (current_time.time_format) ? "PM" : "AM";
#ifndef PRINT_LCD
		printf("Current Time = %s %s\n", time_to_string(&current_time), am_pm);
#else
		lcd_set_cursor(1, 1);
		lcd_print_string(time_to_string(&current_time));
		lcd_print_string(am_pm);
#endif

	}else{

#ifndef PRINT_LCD
		printf("Current Time = %s\n", time_to_string(&current_time));
#else
		lcd_set_cursor(1, 1);
		lcd_print_string(time_to_string(&current_time));
#endif
	}

	ds1307_get_current_date(&current_date);
#ifndef PRINT_LCD
		printf("Current Date = %s <%s>\n", date_to_string(&current_date), get_day_of_week(current_date.day));
#else
		lcd_set_cursor(2, 1);
		lcd_print_string(date_to_string(&current_date));
		lcd_print_char('<');
		lcd_print_string(get_day_of_week(current_date.day));
		lcd_print_char('>');
#endif

}

