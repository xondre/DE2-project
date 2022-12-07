/* Defines -----------------------------------------------------------*/
#define CLK PD2   //main output from rotary encoder                   
#define DT PB4    //reference output from rotary encoder


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
uint16_t x_axis = 512;  //stores x axis position from ADC
uint16_t y_axis = 512;  //stores y axis position from ADC
uint8_t js_axis_select = 0;    //tells to ADC which axis to read
int8_t input_pos = 0;   //tracks current selected input variable
int8_t result_type = 0;   //tracks current result type (rectangle or circle)
int8_t last_result_type = 0;  //tracks previous result type
int8_t a1 = 0;    //stores the first input variable
int8_t b1 = 0;    //stores the second output variable

//array which stores custom icons for
//inputs and outputs for each type of result
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
    //exists only to simplify the code, which automatically
    //loads 4 custom characters, but circle needs only 3 icons
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
 * Function: add_decimal
 * Purpose:  Take the decimal part of result(1st 2 digits), round it 
 *           to the 1st digit and add it to the integral part
 * Arguments: Pointer to the initial integral part, pointer to
 *            the initial decimal part
 * Returns:  none
 **********************************************************************/
//https://github.com/xondre/DE2-project/blob/main/geometric-calculator/add_decimal.png
void add_decimal(uint16_t *result, uint16_t *decimal)
{
  //rounding the 1st 2 decimal digits
  if((*decimal % 10) < 5)
    *decimal = *decimal / 10;
  else
    *decimal = *decimal / 10 + 1;
  //if the decimal part contains any integer add it to the integral part
  if(*decimal > 9)
    *result += *decimal / 10;
  *decimal = *decimal % 10; //decimal part is left with just the 1st decimal digit
}

/************************************************************************
 * Function: update_input
 * Purpose: updates the selected number <0;9> as digit on the display
 * Arguments: digit coordinates as two integers, pointer to integer carrying
 *            the displayed value
 * Returns: none
 ************************************************************************/
//https://github.com/xondre/DE2-project/blob/main/geometric-calculator/update_input.png
void update_input(uint8_t col, uint8_t row, int8_t *value)
{
  if(*value > 9)  //if the value was changed to be out of <0;9>, move it back
    *value = 0;
  if(*value < 0)
    *value = 9;
  lcd_gotoxy(col, row);   //move cursor to the digit's coordinates
  lcd_putc(0x30 + *value);    //diplay the new value
}

/************************************************************************
 * Function: update_result
 * Purpose: Based on the selected result type, display the icons, calculate
 *          the results and put them on the display
 * Arguments: none
 * Returns: none
 ************************************************************************/
//https://github.com/xondre/DE2-project/blob/main/geometric-calculator/update_result.png
void update_result(void)
{
  uint16_t P = 0;   //perimeter
  uint16_t S = 0;   //surface
  char result[3];   //string for converting the integral part of result into text
  uint16_t decimal_P = 0;   //integers representing first decimal digit of each result
  uint16_t decimal_S = 0;

  lcd_command(1<<LCD_CGRAM);    //configure addressing to CGRAM 
  for(uint8_t i = 0; i < 32; i++)   //upload 4 custom characters(icons) depending on the selected result type
    lcd_data(custom_char[i + result_type*32]);
  for(uint8_t i = 0; i < 32; i++)   //upload 4 empty characters to fill the whole CGRAM
    lcd_data(0x00);
  lcd_command(1<<LCD_DDRAM);    //switch back to DDRAM, so characters can be displayed

  switch (result_type)
  {
  case 0:     //if result is set to rectangle
    P = 2*a1 + 2*b1;  //calculate perimeter
    S = a1 * b1;    //calculate surface (area)
    if((a1 == 0) | (b1 == 0))   //if any side is 0, perimeter is 0
      P = 0;
    break;
  
  case 1:     //if result is set to circle 
    //calculate perimeter with 2pi = 6.28
    P = 6*a1;   //initial value of result is calculated only with the integer, i.e. 6
    decimal_P = 28 * a1;   //initial value of first decimal digit is calculated with first two decimal digits, i.e. 28  
    add_decimal(&P, &decimal_P);  //rounds up the decimal digits to the first and adds it to the result
    
    //calculate surface area of the circle with pi = 3.14
    S = 3*a1*a1;    //3.14*R^2 -->3*R^2; 14*R^2
    decimal_S = 14*a1*a1;    
    add_decimal(&S, &decimal_S);
    break;
  default:
    break;
  }
    //display the results
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
  
  //move the cursor back to the current selected input
  switch (input_pos)
  {
  case 1:
    lcd_gotoxy(3,1);
    break;
  default:
    lcd_gotoxy(3,0);
    break;
  }
}

/************************************************************************
 * Function: get_joystick
 * Purpose: Performs A to D conversion on both outputs of the joystick
 *          and returns its current orientation
 *          
 * Arguments: none                                                    1
 * Returns: number from 0 to 4 representing the joystick position   4 0 2
 *                                                                    3
 ************************************************************************/
//https://github.com/xondre/DE2-project/blob/main/geometric-calculator/get_joystick.png
uint8_t get_joystick(void)
{ 
  uint8_t position = 0;   //internal return value 
  for(uint8_t i = 0; i < 2; i++)    //run cycle two times
  {
    if(i == 0)    //on first run, perform ADC for pin A0(x axis of joystick)
    {
    // set ADC output variable to x axis
      js_axis_select = 0;
    // Select input channel ADC0 for x axis
      ADMUX &= ~((1<<MUX0) | (1<<MUX1) | (1<<MUX2) | (1<<MUX3));
    // Start ADC conversion
      ADCSRA |= (1<<ADSC);
      _delay_us(125);
    }
    else    //on second run, perform ADC for pin A1(y axis of joystick)
    {
    // set ADC output variable to y axis
      js_axis_select = 1;
    // Select input channel ADC1 for y axis
      ADMUX |= (1<<MUX0);  ADMUX &= ~((1<<MUX1) | (1<<MUX2) | (1<<MUX3));
    // Start ADC conversion
      ADCSRA |= (1<<ADSC);
    }
}
      //if the joystick points far enough to the edge
      //                                    1
      //return the coresponding position  4 0 2
      //                                    3
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

/************************************************************************
 * Function: main
 * Purpose: Initializing during the start, periodically updating the program
 *          based on the joystick input
 * Arguments: none
 * Returns: 0 if it ends successfully
 ************************************************************************/
//https://github.com/xondre/DE2-project/blob/main/geometric-calculator/main.png
int main(void)
{  
  
    GPIO_mode_input_nopull(&DDRB, DT);    //pin with the reference output from encoder
                                          //is set to input
    
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

    //initialize display
    lcd_init(LCD_DISP_ON_BLINK);
    lcd_gotoxy(1,0);
    lcd_puts("A=0");
    lcd_gotoxy(1,1);
    lcd_puts("B=0");
    update_result();


    // Infinite loop
    while (1)
    {      
      //get the current position of the joystick
      //if joystick points left or right, change input variable selection
      //if joystick points up or down, change the result type 
      switch (get_joystick())
      {
      case 1:         
        input_pos--;
        break;
      case 2:
        result_type--;    
        break;
      case 3:
        input_pos++;
        break;
      case 4:
        result_type++;
        break;
      default:
        break;
      }

      //if any selection variable gets out of bounds,
      //put it back from the other side of the interval
      if(input_pos < 0)
        input_pos = 1;
      else if(input_pos > 1)
        input_pos = 0;
      else if(result_type < 0)
        result_type = 1;
      else if(result_type > 1)
        result_type = 0;

      //change the cursor position according to the current selection
      switch (input_pos)
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

      //result is updated only when there was change
      if(last_result_type != result_type)
        update_result();
      last_result_type = result_type;

      _delay_ms(200);   //wait for 200 ms
    }    

        return 0;   //this point is never reached
}

/* Interrupt service routines ----------------------------------------*/
/*********************************************************************
 * Function: External interrupt on pin PD2
 * Purpose: Sensing the movement of the encoder and changing the value
 *          of the selected input variable
***********************************************************************/
//https://github.com/xondre/DE2-project/blob/main/geometric-calculator/INT0.png
ISR(INT0_vect)
{
  static int8_t step = 0;   //
  
  //on every leading edge of the main output from the encoder,
  //the reference output is read and direction of movement is determined 
  if(GPIO_read(&PINB, DT) == 0)
    step = 1;
  else
    step = -1;

  //current input variable is changed based on the encoder movement
  //and its value is updated also on the display
  switch (input_pos)
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
  update_result();    //since the input variable was changed, the results are updated

}

/**********************************************************************
 * Function: ADC complete interrupt
 * Purpose:  Read current value of one of the axes in joystick
 **********************************************************************/
//https://github.com/xondre/DE2-project/blob/main/geometric-calculator/ADC.png
ISR(ADC_vect)
{ 
  //depending on the axis which was selected prior
  //save the converted value of the axis  
  if(js_axis_select == 0)
    x_axis = ADC;
  else 
    y_axis = ADC;
}
