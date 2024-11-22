#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "usart.h"
#include "i2cmaster.h"
#include "lcd.h"
#include "lm75.h"
#include "adcpwm.h"



int main(void)
{


  i2c_init(); // should be first
  LCD_init();
  
  uart_init();
              //io_redirect(); //          wtf it blokcs the lcd 
  pwm3_init(); // initialize PWM signal at pin PB0, PB1, PB2 with frequency of 150 Hz
  adc_init();  // initialize the ADC module

  DDRD = 0xFF;
  PORTD = 0x00;

  DDRC = 0xF0;
  PORTC = 0x3F;

  while (1)
  {

    //if (PINC == 0b00111110)
    //{  
    

      float ledbright0;
      float ledbright1;
      float ledbright2;

      
        for (int i = 1; i < 100; i++)
        {
          pwm3_set_duty(i, i, i); // duty cycles in percent at PB0, PB1, PB2, from 0 - 100
          // pwm1_set_duty(i); //duty cycle in percent at PB1, from 0 - 100
          _delay_ms(10);

          //adc_value = adc_read(0); // Value 0-1023 representing analog voltage on pin PC0

          ledbright0 = adc_read(0);
          ledbright1 = adc_read(1);
          ledbright2 = adc_read(2);

          LCD_set_cursor(0, 0);
          printf("RED  BLUE  GREEN");

          LCD_set_cursor(0, 1);
          printf("%.0f\n", ledbright0/1023*100);
          LCD_set_cursor(3, 1);
          printf("p");

          LCD_set_cursor(5, 1);
          printf("%.0f\n", ledbright1/1023*100);
          LCD_set_cursor(8, 1);
          printf("p");

          LCD_set_cursor(11, 1);
          printf("%.0f\n", ledbright2/1023*100);
          LCD_set_cursor(14, 1);
          printf("p");

          PORTD = 0b00000000;

          if (PINC == 0b00111101)
          {
            i = 100;
          }
 //     }
    }

    if (PINC == 0b00111101)
    {

      LCD_set_cursor(0, 0);
      printf("PARTY MODE IS ON");
      LCD_set_cursor(0, 1);
      printf("                  ");
      PORTD = 0b10100000;
      _delay_ms(200);
      PORTD = 0b1100000;
      _delay_ms(200);
      PORTD = 0b01100000;
      _delay_ms(200);
      PORTD = 0b00100000;
    }
  }
}


/*
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "usart.h"
#include "i2cmaster.h"
#include "lcd.h"
#include "lm75.h"
#include "adcpwm.h"
 
int main(void) 
{ 
 
  uart_init(); 
  //io_redirect(); 
  // pwm1_init(); // initialize PWM signal at pin PB1, frequency of 4 kHz 
  pwm3_init(); // initialize PWM signal at pin PB0, PB1, PB2 with frequency of 150 Hz 
  adc_init();  // initialize the ADC module 
   i2c_init(); // should be first
  LCD_init();

  DDRC = 0b11110000;  // configure pins PC0 to PC3 as inputs 
  PORTC = 0b00110000; // configure pins PC0 to PC3 to not use pullups for the ADC 
 
  float adc_value; 
 
  while (1) 
  { 
    for (int i = 1; i < 100; i++) 
    { 
      pwm3_set_duty(i, i, i); // duty cycles in percent at PB0, PB1, PB2, from 0 - 100 
      // pwm1_set_duty(i); //duty cycle in percent at PB1, from 0 - 100 
      _delay_ms(1); 
    } 
    adc_value = adc_read(0); // Value 0-1023 representing analog voltage on pin PC0 
    LCD_set_cursor(0, 0);
    printf("Result of the ADC conversion : %.0f\n", adc_value / 1023 * 100); 
  } 
}

*/