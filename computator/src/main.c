/* Defines -----------------------------------------------------------*/
#define CLK PD2   //main output from rotary encoder                   
#define DT PB4    //reference output from rotary encoder
#define SW_JS PD3   //button output from joystick

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

/*  Global variables         */
uint16_t x_axis = 512;  //
uint16_t y_axis = 512;  //
uint8_t js_axis_select = 0;    //
int8_t line1_pos = 0;
int8_t result_type = 0;
int8_t a1 = 0;
int8_t b1 = 0;

uint8_t custom_char[64] = {
    //rectangle perimeter icon 
	    0b00000,
	    0b11111,
	    0b10001,
	    0b10001,
	    0b10001,
	    0b10001,
	    0b11111,
	    0b00000,
    //rectangle area icon
      0b00000,
	    0b11111,
	    0b11111,
	    0b11111,
	    0b11111,
	    0b11111,
	    0b11111,
	    0b00000,
    //rectangle horizontal side
      0b00000,
	    0b11111,
	    0b10001,
	    0b10001,
	    0b10001,
	    0b11111,
	    0b11111,
	    0b00000,
    //rectangle vertical side
      0b00000,
	    0b11111,
	    0b11001,
	    0b11001,
	    0b11001,
	    0b11001,
	    0b11111,
	    0b00000,
    //circle perimeter icon
      0b00000,
	    0b00000,
	    0b01110,
	    0b10001,
	    0b10001,
	    0b10001,
	    0b01110,
	    0b00000,
    //circle area icon
      0b00000,
	    0b00000,
	    0b01110,
	    0b11111,
	    0b11111,
	    0b11111,
	    0b01110,
	    0b00000,
    //circle radius icon
      0b00000,
	    0b00000,
	    0b01110,
	    0b10001,
	    0b11101,
	    0b10001,
	    0b01110,
	    0b00000,
    //empty icon
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000
    };

/* Function definitions ----------------------------------------------*/
/**********************************************************************
 * Function: Main function where the program execution begins
 * Purpose:  Toggle LEDs and use delay library.
 * Returns:  none
 **********************************************************************/
void round_and_decimal(uint16_t *result, uint16_t *decimal)
{
  //uint8_t remainder = *decimal % 10;
  if((*decimal % 10) < 5)
    *decimal = *decimal / 10;
  else
    *decimal = *decimal / 10 + 1;
  if(*decimal > 9)
    *result += *decimal / 10;
  *decimal = *decimal % 10;
}

void update_input(uint8_t col, uint8_t row, int8_t *value)
{
  if(*value > 9)
    *value = 0;
  if(*value < 0)
    *value = 9;
  lcd_gotoxy(col, row);
  lcd_putc(0x30 + *value);
  //lcd_gotoxy(col, row);
}

void update_result(void)
{
  uint16_t P = 0;   //perimeter
  uint16_t S = 0;   //surface
  char result[5];
  uint16_t decimal_P = 0;
  uint16_t decimal_S = 0;
  lcd_command(1<<LCD_CGRAM);
  for(uint8_t i = 0; i < 32; i++)
    lcd_data(custom_char[i + result_type*32]);
  for(uint8_t i = 0; i < 32; i++)
    lcd_data(0x00);
  lcd_command(1<<LCD_DDRAM);
  switch (result_type)
  {
  case 0:
    P = 2*a1 + 2*b1;
    S = a1 * b1;
    if((a1 == 0) | (b1 == 0))
      P = 0;
    break;
  
  case 1:
    P = 6*a1;
    decimal_P = 28 * a1;   //6,28*R --> 28 * R  
    round_and_decimal(&P, &decimal_P);
    
    S = 3*a1*a1;
    decimal_S = 14*a1*a1;    //3.14*R^2 --> 14 * R^2
    round_and_decimal(&S, &decimal_S);
    break;
  default:
    break;
  }
    lcd_gotoxy(0,0); lcd_putc(0x02);
    lcd_gotoxy(0,1); lcd_putc(0x03);
    lcd_gotoxy(6,0);
    lcd_putc(0x00);
    lcd_puts("P=     ");
    lcd_gotoxy(9,0);
    itoa(P, result, 10);
    lcd_puts(result);
    lcd_putc('.');
    lcd_putc(0x30 + decimal_P);
    lcd_gotoxy(6,1);
    lcd_putc(0x01);
    lcd_puts("S=     ");
    lcd_gotoxy(9,1);
    itoa(S, result, 10);
    lcd_puts(result);
    lcd_putc('.');
    lcd_putc(0x30 + decimal_S);
  
  switch (line1_pos)
  {
  case 1:
    lcd_gotoxy(3,1);
    break;
  default:
    lcd_gotoxy(3,0);
    break;
  }
}


uint8_t get_joystick(void)
{ 
  uint8_t position = 0; 
  for(uint8_t i = 0; i < 2; i++)
  {
    if(i == 0)
    {
    // set ADC output variable to x axis
      js_axis_select = 0;
    // Select input channel ADC0 for x axis
      ADMUX &= ~((1<<MUX0) | (1<<MUX1) | (1<<MUX2) | (1<<MUX3));
    // Start ADC conversion
      ADCSRA |= (1<<ADSC);
      _delay_us(125);
    }
    else
    {
    // set ADC output variable to y axis
      js_axis_select = 1;
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


int main(void)
{  
  
    GPIO_mode_input_nopull(&DDRB, DT);
    GPIO_mode_input_pullup(&DDRD, SW_JS);
    
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

    //Configure external interrupt INT0 for sensing rising edge
    EICRA |= ((1<<ISC01) | (1<<ISC00));
    //enable external interrupt INT0
    EIMSK |= (1<<INT0);

    // Enables interrupts by setting the global interrupt mask
    sei();

    lcd_init(LCD_DISP_ON_BLINK);
    lcd_gotoxy(1,0);
    lcd_puts("A=0");
    lcd_gotoxy(1,1);
    lcd_puts("B=0");
    update_result();


    // Infinite loop
    while (1)
    {   
      static uint8_t last_js_state = 0;
      static uint8_t js_state = 0;
      js_state = get_joystick();

      switch (js_state)
      {
      case 1:
        line1_pos--;
        break;
      case 2:
        result_type--;
        break;
      case 3:
        line1_pos++;
        break;
      case 4:
        result_type++;
        break;
      default:
        break;
      }

      if(line1_pos < 0)
        line1_pos = 1;
      else if(line1_pos > 1)
        line1_pos = 0;
      else if(result_type < 0)
        result_type = 1;
      else if(result_type > 1)
        result_type = 0;

      switch (line1_pos)
      {
      case 0:
        lcd_gotoxy(3,0);
        break;
      case 1:
        lcd_gotoxy(3,1);
        break;
      default:
        break;
      }

      if(last_js_state != js_state)
        update_result();

      last_js_state = js_state;
      _delay_ms(200);
    }    
        return 0;
}

/* Interrupt service routines ----------------------------------------*/

ISR(INT0_vect)
{
  static int8_t step = 0;
  
  if(GPIO_read(&PINB, DT) == 0)
    step = 1;
  else
    step = -1;

  switch (line1_pos)
  {
  case 1:
    b1 += step;
    update_input(3, 1, &b1);
    break;

  default:
    a1 += step;
    update_input(3, 0, &a1);
    break;
  }
  update_result();

}

/**********************************************************************
 * Function: ADC complete interrupt
 * Purpose:  Display converted value on LCD screen.
 **********************************************************************/
ISR(ADC_vect)
{   
  if(js_axis_select == 0)
    x_axis = ADC;
  else 
    y_axis = ADC;
}