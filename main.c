#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "usart.h"
#include "i2cmaster.h"
#include "lcd.h"
#include "adcpwm.h"

void pwm_1_init(void);
long map(long x, long in_min, long in_max, long out_min, long out_max);

int switchMask = 0b00010000; // PB4
int ledMask = 0b11101111;    // PD4

uint8_t pirMask = 0b00001000; // PD3
unsigned int door_adc;
int door_pot = 6;
int mask_door = 0b00100000; // PB4

int main(void)
{
  // PORT B
  DDRB = 0b00110111;  // PB0 - PB3 outputs
  PORTB = 0b00110111; // Set the outputs to 1 so the LEDs are not lit up

  // PORT C
  DDRC = 0b10110000;  // configure pins PC0 to PC3 and PC6 as inputs
  PORTC = 0b00110000; // configure pins PC0 to PC3 and PC6 to not use pullups for the ADC

  // PORT D
  DDRD = 0b00010000;  // PD7 - PD5 as inputs and PD4 as output
  PORTD = 0b11110000; // PD7 - PD5 enable pull-ups and PD4 so the LED is not lit up

   i2c_init();
   LCD_init();
   
  adc_init(); // initialize the ADC module
  // io_redirect();
  uart_init();
  pwm_1_init(); // Initialize PWM
  pwm3_init();

  // printf("adc_read %0.f\n", ledbright5);
  // printf("input %d\n", led5);
  while (1)
  {

    float ledbright0 = adc_read(0) * 0.097;
    float ledbright1 = adc_read(1) * 0.097;
    float ledbright2 = adc_read(2) * 0.097; 
    // int ledbright3 = adc_read(3) * 0.097; // room 1;

    unsigned char led0 = (unsigned char)ledbright0; // red
    unsigned char led1 = (unsigned char)ledbright1; // green
    unsigned char led2 = (unsigned char)ledbright2; // blue
    // unsigned char led3 = (unsigned char)ledbright3;

    // room 1
    float ledbright5 = adc_read(3) * 0.097; // Read from PC3
    unsigned char led5 = (unsigned char)ledbright5;

    if (PIND & (1 << PD2))
    {
      // Switch is pressed, stop PWM
    
      LCD_set_cursor(0, 2);
      printf("r-1: %dp   ", 100 - led5);

      OCR0B = led5 * 2.55;
      
    }

    else
    {
     
      LCD_set_cursor(0, 2);
      printf("r-1: is off ");
      _delay_ms(50);     
      // Switch is not pressed, set PWM duty cycle

      OCR0B = 255;
    }

    _delay_ms(200);

    // room 2

    if (PINB & (1 << PB3))
    {
    
      LCD_set_cursor(12, 2);
      printf("r-2: on   ");
      
      PORTD &= ledMask;
    
    }

    else
    {
     
      LCD_set_cursor(12, 2);
      printf("r-2: off   ");
     
      PORTD |= ~ledMask;

    }

    // room 3

    if (PIND & (1 << PD7))
    {
    
            LCD_set_cursor(0, 0);
            printf("BLUE  GREEN  RED");

           
            LCD_set_cursor(0, 1);
            printf("%d   ", 100 - led0);
   

            LCD_set_cursor(6, 1);
            printf("%d   ", 100 - led1);
         

            LCD_set_cursor(13, 1);
            printf("%d   ", 100 - led2);
         
         
      

      if (led0 >= 95)
      {
        led0 = 100;
      }
      if (led1 >= 90)
      {
        led1 = 100;
      }
      if (led2 >= 85)
      {
        led2 = 100;
      }
    }

    if (PIND & (1 << PD6))
    {

    
      LCD_set_cursor(0, 0);
      printf("PARTY MODE IS ON");
      LCD_set_cursor(0, 1);
      printf("                  ");



      led0 = 0;
      led1 = 0;
      led2 = 100;
      _delay_ms(100);

      pwm3_set_duty(led0, led1, led2);
      led0 = 100;
      led1 = 0;
      led2 = 0;
      _delay_ms(100);

      pwm3_set_duty(led0, led1, led2);
      led0 = 0;
      led1 = 100;
      led2 = 0;
      _delay_ms(100);

      pwm3_set_duty(led0, led1, led2);
    }

    if (!(PIND & ((1 << PD6) | (1 << PD7))))
    {
  
      LCD_set_cursor(0, 0);
      printf("room 3 is off    ");
      LCD_set_cursor(0, 1);
      printf("                  ");

      led0 = 100; 
      led1 = 100; 
      led2 = 100; 
    }
    pwm3_set_duty(led0, led1, led2);


    // Code for door


    
  
door_adc = map(adc_read(door_pot), 0, 1024, 0, 100); 
     
    // Code for door 
    if ((PIND & pirMask) == pirMask) 
    { 
       if ((door_adc < 20)){
      PORTB |= mask_door; // PB4 is high 
      LCD_set_cursor(0,3);
      printf("out on  is move   ");
       }
   else
    { 
      PORTB &= ~mask_door; 
          LCD_set_cursor(0,3);
      printf("out off is move   ");
    }
    }
  else{
  if ((door_adc < 20 )){
      PORTB &= ~mask_door; 
      LCD_set_cursor(0,3);
      printf("out on  no move   ");
       }
   else
    { 
      PORTB &= ~mask_door; 
          LCD_set_cursor(0,3);
      printf("out off no move   ");
    }
    }


  }
}





void pwm_1_init(void)
{
  // Set Fast PWM mode, non-inverting mode on OC0A and OC0B
  TCCR0A |= (1 << WGM00) | (1 << COM0A1) | (1 << WGM01) | (1 << COM0B1);
  // Set prescaler to 64
  TCCR0B |= (1 << CS01) | (1 << CS00);
  // Set output pin
  DDRD |= (1 << PD5);
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

    
  
