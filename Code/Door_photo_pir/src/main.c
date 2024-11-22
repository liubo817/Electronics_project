#include <stdio.h>
#include <util/delay.h>
#include <avr/io.h>
#include "usart.h"
#include "adcpwm.h"

void printBinary(uint8_t value);
int pirMask = 0b00000001;
int ledMask = 0b11111101;

int main(void)
{ 
  uart_init(); 
  io_redirect();
  //pwm1_init(); // initialize PWM signal at pin PB1, frequency of 4 kHz
  //pwm3_init(); // initialize PWM signal at pin PB0, PB1, PB2 with frequency of 150 Hz
  adc_init(); // initialize the ADC module

  unsigned int adc_value;
  DDRC = 0b11110000; // configure pins PC0 to PC3 as inputs
  PORTC = 0b00110000; // configure pins PC0 to PC3 to not use pullups for the ADC
  DDRB = 0b11111110;
  PORTB = 0b00000010;
  while(1) {
      adc_value = adc_read(0);
      printf("PINB: ");
      printBinary(PINB); // Print the binary value of PIND
      _delay_ms(1000);   // Delay for readability
      if((PINB & pirMask == pirMask) || (adc_value <= 50))
      {
        printf("Detected.\n");
        PORTB &= ledMask;
      }else{
        PORTB |= ~ledMask;
      }
      
      printf("ADC value: %u\n", adc_value);
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
    printf("\n"); // Move to the next line after printing
}