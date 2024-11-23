#include <stdio.h>
#include <util/delay.h>
#include <avr/io.h>
#include "usart.h"
#include "adcpwm.h"
#include "lcd.h"
#include "i2cmaster.h"

void printBinary(uint8_t value);

uint8_t pirMaskD = 0b10000000;
uint8_t ledMask = 0b10111111;


int main(void)
{ 
  i2c_init();
  LCD_init();
  //uart_init(); 
  //io_redirect();
  //pwm1_init(); // initialize PWM signal at pin PB1, frequency of 4 kHz
  //pwm3_init(); // initialize PWM signal at pin PB0, PB1, PB2 with frequency of 150 Hz
  adc_init(); // initialize the ADC module
  LCD_set_cursor(0,0);
  unsigned int adc_value;
  DDRC = 0b11110000; // configure pins PC0 to PC3 as inputs
  PORTC = 0b00110000; // configure pins PC0 to PC3 to not use pullups for the ADC
  //DDRB = 0b11111110;
  //PORTB = 0b00000010;
  DDRD = 0b01111111;
  PORTD = 0b01000000;

  while(1) {
      adc_value = adc_read(0);
      //printf("PIND: ");
      printBinary(PIND); // Print the binary value of PIND
      _delay_ms(1000);   // Delay for readability
      if((PIND & pirMaskD) == pirMaskD || adc_value <= 45)
      {
        printf("Detected");
        PORTD &= ledMask;
      }else{
        PORTD |= ~ledMask;
      } 
      //printf("ADC value: %u\n", adc_value);
    }
  return 0;
    
}

void printBinary(uint8_t value) {
    for (int i = 7; i >= 0; i--) { // Iterate from MSB to LSB
        if (value & (1 << i)) {
            printf("1");
        } else {
            printf("0");
        }
    }
    //printf("\n"); // Move to the next line after printing
}