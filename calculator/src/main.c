/***********************************************************************
 * 
 * Blink LEDs in Arduino-style and use function from the delay library.
 * ATmega328P (Arduino Uno), 16 MHz, PlatformIO
 *
 * Copyright (c) 2022 Tomas Fryza
 * Dept. of Radio Electronics, Brno University of Technology, Czechia
 * This work is licensed under the terms of the MIT license.
 * 
 **********************************************************************/


/* Defines -----------------------------------------------------------*/
#define CLK PD0                     
#define DT PD1 
#define SW PD2
#define SHORT_DELAY 250 // Delay in milliseconds
#ifndef F_CPU
# define F_CPU 16000000 // CPU frequency in Hz required for delay funcs
#endif




/* Includes ----------------------------------------------------------*/
#include <avr/io.h>     // AVR device-specific IO definitions
#include <util/delay.h> // Functions for busy-wait delay loops
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include <gpio.h>           // GPIO library for AVR-GCC
#include "timer.h"          // Timer library for AVR-GCC
#include <lcd.h>            // Peter Fleury's LCD library
#include <stdlib.h>         // C library. Needed for number conversions
#include <uart.h>           // Peter Fleury's UART library

/*                */
uint8_t R = 0;
uint16_t x_axis = 512;
uint16_t y_axis = 512;
uint8_t axis_select = 0;
/*                 */

// -----
// This part is needed to use Arduino functions but also physical pin
// names. We are using Arduino-style just to simplify the first lab.
//#include "Arduino.h"
//#define PB5 13          // In Arduino world, PB5 is called "13"
//#define PB0 8
// -----


/* Function definitions ----------------------------------------------*/
/**********************************************************************
 * Function: Main function where the program execution begins
 * Purpose:  Toggle LEDs and use delay library.
 * Returns:  none
 **********************************************************************/
int main(void)
{   
    GPIO_mode_input_nopull(&DDRD, CLK);
    GPIO_mode_input_nopull(&DDRD, DT);
    uint8_t encoder_ref = GPIO_read(&PIND, CLK);

    int8_t get_encoder(void)
    {
      int8_t step = 0;
      uint8_t clk_val = GPIO_read(&PIND, CLK);

      if(clk_val != encoder_ref){ 
        if(GPIO_read(&PIND, DT) != clk_val)
          step = -1;
        else
          step = 1;
      }else
        step = 0;
      
      encoder_ref = clk_val;
      return step;
    }

    uint8_t get_joystick(void)
    { 
      uint8_t position = 0;
      for(uint8_t i = 0; i < 2; i++)
      {
        if(i == 0)
        {
        // set ADC output variable to x axis
          axis_select = 0;
        // Select input channel ADC0 for x axis
          ADMUX &= ~((1<<MUX0) | (1<<MUX1) | (1<<MUX2) | (1<<MUX3));
        // Start ADC conversion
          ADCSRA |= (1<<ADSC);
          _delay_us(125);
        }
        else
        {
        // set ADC output variable to y axis
          axis_select = 1;
        // Select input channel ADC1 for y axis
          ADMUX |= (1<<MUX0);  ADMUX &= ~((1<<MUX1) | (1<<MUX2) | (1<<MUX3));
        // Start ADC conversion
          ADCSRA |= (1<<ADSC);
        }
      }
      if(x_axis < 400)
        position = 4;
      else if(y_axis > 625)
        position = 1;
      else if(y_axis < 400)
        position = 3;
      else if(x_axis > 625)
        position = 2;
      else
        position = 0;

      return position;
    }

    //
    lcd_init(LCD_DISP_ON);
    
    // Configure Analog-to-Digital Convertion unit
    // Select ADC voltage reference to "AVcc with external capacitor at AREF pin"
    ADMUX |= (1<<REFS0);  //setting REFS0 to 1
    ADMUX &= ~(1<<REFS1); //setting REFS1 to 0
    // Enable ADC module
    ADCSRA |= (1<<ADEN);
    // Enable conversion complete interrupt
    ADCSRA |= (1<<ADIE);
    // Set clock prescaler to 128
    ADCSRA |= ((1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2));

    // Enables interrupts by setting the global interrupt mask
    sei();
     
    int8_t counter = 0;
    
    // Infinite loop
    while (1)
    {
      char string[4];
      itoa(get_joystick(), string, 10);
      lcd_gotoxy(0,0); lcd_puts(string);
      
      counter += get_encoder();
 
      itoa(counter, string, 10);
      lcd_gotoxy(0,1);
      lcd_puts("   ");
      lcd_gotoxy(0,1);
      lcd_puts(string);
      
    }    
    return 0;

    

}

/* Interrupt service routines ----------------------------------------*/
/**********************************************************************
 * Function: Timer/Counter1 overflow interrupt
 * Purpose:  Use single conversion mode and start conversion every 100 ms.
 **********************************************************************/



/**********************************************************************
 * Function: ADC complete interrupt
 * Purpose:  Display converted value on LCD screen.
 **********************************************************************/
ISR(ADC_vect)
{   
  if(axis_select == 0)
    x_axis = ADC;
  else 
    y_axis = ADC;
}
