//UART Example for inclass coding
//Roger Traylor 12.4.12
//Connect two mega128 boards via rs232 and they will send to each
//other a message and a sequence number.


#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include "uart_functions.h"
#include "hd44780.h"
#include <avr/interrupt.h>
#include <util/delay.h>

uint8_t           i;
volatile uint8_t  rcv_rdy;
char              rx_char; 
uint8_t           send_seq=0;         //transmit sequence number

int main(){
  DDRF |= 0x08; //lcd strobe bit
  uart_init();  

  sei();
  while(1){

//**************  start tx portion ***************
    uart_puts("Hi! Dilbert: ");
    itoa(send_seq,lcd_string,10);
    uart_puts(lcd_string);
    uart_putc('\0');
    for(i=0;i<=9;i++){_delay_ms(100);}
    send_seq++;
    send_seq=(send_seq%20);
//**************  end tx portion ***************
  }//while
}//main

