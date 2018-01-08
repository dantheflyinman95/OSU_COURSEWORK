// mega48_temp.c 
// D. Eisenbach
// 12.03.16

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include "twi_master.h"
#include "uart_functions_m48.h"
#include "lm73_functions_skel.h"

//interrupt indicating data has been received from the mega128
ISR(USART_RX_vect){
  rx_char = uart_getc();  //get data byte from mega128
  uart_puts(temp_str); uart_putc('\0');  // Send lm73 temp data byte to mega128
}

//*************************************************************************/
int main() {
  uart_init();
  init_twi();  //initialize TWI, used for radio and LM73

  //set LM73 mode for reading temperature by loading pointer register
  //this is done outside of the normal interrupt mode of operation 
  lm73_wr_buf[0] = LM73_PTR_TEMP;   //load lm73_wr_buf[0] with temperature pointer address
  twi_start_wr(LM73_ADDRESS, lm73_wr_buf, 1);   //start the TWI write process (twi_start_wr())

  sei();  //enable interrupts

  while(1) {
    _delay_ms(2);
    if (rx_char == 'C') {lm73_temp_convert(1);}  //display temp in C
    else if (rx_char == 'F') {lm73_temp_convert(0);}  //display temp in F
  }
}

  



