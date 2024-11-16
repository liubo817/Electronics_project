#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "usart.h"

void printBinary(uint8_t value);

int main(void) {  

  uart_init(); // open the communication to the microcontroller
  io_redirect(); // redirect input and output to the communication
  int pirMask = 0b00000001;
  int ledMask = 0b11111101;
  DDRB = 0b11111110;
  PORTB = 0b00000010;
  while(1) {
    printf("PINB: ");
    printBinary(PINB); // Print the binary value of PIND
    _delay_ms(1000);   // Delay for readability
    if(PINB & pirMask == pirMask)
    {
      printf("Detected.\n");
      PORTB &= ledMask;
    }else{
      PORTB |= ~ledMask;
    } 
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