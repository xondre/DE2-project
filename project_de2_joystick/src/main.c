/***********************************************************************
 * 
 * Use Analog-to-digital conversion to read push buttons on LCD keypad
 * shield and display it on LCD screen.
 * 
 * ATmega328P (Arduino Uno), 16 MHz, PlatformIO
 *
 * Copyright (c) 2018 Tomas Fryza
 * Dept. of Radio Electronics, Brno University of Technology, Czechia
 * This work is licensed under the terms of the MIT license.
 * 
 **********************************************************************/


/* Includes ----------------------------------------------------------*/
#include <avr/io.h>         // AVR device-specific IO definitions
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include <gpio.h>           // GPIO library for AVR-GCC
#include "timer.h"          // Timer library for AVR-GCC
#include <lcd.h>            // Peter Fleury's LCD library
#include <stdlib.h>         // C library. Needed for number conversions

/* defining global variables      */
uint8_t x_enable = 0; 
uint8_t y_enable = 0;
uint16_t x_value = 0;
uint16_t y_value = 0;


/*             */
/* Function definitions ----------------------------------------------*/
/**********************************************************************
 * Function: Main function where the program execution begins
 * Purpose:  Use Timer/Counter1 and start ADC conversion every 100 ms.
 *           When AD conversion ends, send converted value to LCD screen.
 * Returns:  none
 **********************************************************************/
int main(void)
{
    // Initialize display
    lcd_init(LCD_DISP_ON);
     //lcd_gotoxy(1, 0); lcd_puts("value:");
    // lcd_gotoxy(3, 1); lcd_puts("key:");
    // lcd_gotoxy(8, 0); lcd_puts("a");  // Put ADC value in decimal
    // lcd_gotoxy(13,0); lcd_puts("b");  // Put ADC value in hexadecimal
    // lcd_gotoxy(8, 1); lcd_puts("c");  // Put button name here

    // Configure Analog-to-Digital Convertion unit
    // Select ADC voltage reference to "AVcc with external capacitor at AREF pin"
    ADMUX |= (1<<REFS0);  //setting REFS0 to 1
    ADMUX &= ~(1<<REFS1); //setting REFS1 to 0
    // // Select input channel ADC0 (voltage divider pin)
    // ADMUX &= ~((1<<MUX0) | (1<<MUX1) | (1<<MUX2) | (1<<MUX3));
    // Enable ADC module
    ADCSRA |= (1<<ADEN);
    // Enable conversion complete interrupt
    ADCSRA |= (1<<ADIE);
    // Set clock prescaler to 128
    ADCSRA |= ((1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2));

    // Configure 16-bit Timer/Counter1 to start ADC conversion
    // Set prescaler to 33 ms and enable overflow interrupt
    TIM0_overflow_16ms();
    TIM0_overflow_interrupt_enable();

    // Enables interrupts by setting the global interrupt mask
    sei();

    // Infinite loop
    while (1)
    {
        /* Empty loop. All subsequent operations are performed exclusively 
         * inside interrupt service routines ISRs */
    }

    // Will never reach this
    return 0;
}


/* Interrupt service routines ----------------------------------------*/
/**********************************************************************
 * Function: Timer/Counter1 overflow interrupt
 * Purpose:  Use single conversion mode and start conversion every 100 ms.
 **********************************************************************/
ISR(TIMER0_OVF_vect)
{
    if (x_enable == 0)
    {
      if (y_enable == 0)
      {
        x_enable = 1;  
        // Select input channel ADC0 for x axis
        ADMUX &= ~((1<<MUX0) | (1<<MUX1) | (1<<MUX2) | (1<<MUX3));
        // Start ADC conversion
        ADCSRA |= (1<<ADSC);
      }
    }
    else if (y_enable == 0)
    {
    y_enable = 1;
    // Select input channel ADC1 for y axis
    ADMUX |= (1<<MUX0);  ADMUX &= ~((1<<MUX1) | (1<<MUX2) | (1<<MUX3));
    // Start ADC conversion
    ADCSRA |= (1<<ADSC);
    }else if (x_enable == 1 & y_enable == 1)
    {
     x_enable = 0;
     y_enable = 0; 
    }

}

/**********************************************************************
 * Function: ADC complete interrupt
 * Purpose:  Display converted value on LCD screen.
 **********************************************************************/
ISR(ADC_vect)
{   
    char string[4];


    if (x_enable == 1)
    {
      if (y_enable == 0)
      {
        x_value = ADC;    //save value from pin A0
      }else if (y_enable==1)
      {
        y_value = ADC;  //save value from pin A1
      }
    }
    lcd_gotoxy(1, 0); lcd_puts("x:");
    itoa(x_value, string, 10);
    lcd_gotoxy(8, 0); lcd_puts("    ");
    lcd_gotoxy(8, 0); lcd_puts(string);

    lcd_gotoxy(1, 1); lcd_puts("y:");
    itoa(y_value, string, 10);
    lcd_gotoxy(8, 1); lcd_puts("    ");
    lcd_gotoxy(8, 1); lcd_puts(string);




    // uint16_t value;
    // uint16_t voltage;
    // char string[4];  // String for converted numbers by itoa()
    // char string_hex[3];
    // char string_volt[4];
    // // Read converted value
    // // Note that, register pair ADCH and ADCL can be read as a 16-bit value ADC
    // value = ADC;
    // // Convert "value" to "string" and display it
    // itoa(value, string, 10);
    // itoa(value, string_hex, 16);

    // voltage = value * 5;
    // voltage *= 1000;
    // voltage /= 1023;
    // itoa(voltage, string_volt, 10);

    // if(value < 50)
    // {
    //   lcd_gotoxy(8, 1); lcd_puts("right ");
    // }else if(value < 200){
    //   lcd_gotoxy(8, 1); lcd_puts("up    ");
    // }else if(value < 400){
    //   lcd_gotoxy(8, 1); lcd_puts("down  ");
    // }else if(value < 600){
    //   lcd_gotoxy(8, 1); lcd_puts("left  ");
    // }else if(value < 800){
    //   lcd_gotoxy(8, 1); lcd_puts("select");
    // }else{
    //   lcd_gotoxy(8, 1); lcd_puts("none  ");
    // }
    // lcd_gotoxy(8, 0); lcd_puts("     ");
    // lcd_gotoxy(8, 0); lcd_puts(string);
    // lcd_gotoxy(13, 0); lcd_puts("   ");
    // lcd_gotoxy(13, 0); lcd_puts(string_hex);
    // //lcd_gotoxy(12, 1); lcd_puts("    ");
    // //lcd_gotoxy(12, 1); lcd_puts(string_volt);
}