// lm73_functions.c       
// Daniel Eisenbach
// 12.03.2016

#include <util/twi.h>
#include "lm73_functions_skel.h"
#include <util/delay.h>
#include <string.h>

//********************************************************************************

//******************************************************************************
//                       lm_73_temp_convert
//Given a temperature reading from an LM73 and a format (deg F or C) it 
//formats the temperature into ascii in the temp_str buffer.
void lm73_temp_convert(uint8_t c_not_f){
  //decimal point precision variable
  uint16_t dec_temp;  //.25C change equals .45F change
  char dec_str[16];
  
  while( twi_busy() ){}; //spin while previous TWI transaction finshes
  twi_start_rd(LM73_ADDRESS, lm73_rd_buf, 2); //read temperature data from LM73 (2 bytes)
  _delay_ms(2);    //wait for it to finish

  //now assemble the two bytes read back into one 16-bit value
  lm73_temp = lm73_rd_buf[0]; //save high temperature byte into lm73_temp
  lm73_temp = (lm73_temp << 8);  //shift it into upper byte 
  lm73_temp |= lm73_rd_buf[1]; //"OR" in the low temp byte to lm73_temp
  lm73_temp = (lm73_temp << 1);  //remove sign bit
  dec_temp = (lm73_temp << 8);  //isolate decimal place bits (0.25C)
  dec_temp = (dec_temp >> 14);  //make right justified
  lm73_temp = (lm73_temp >> 6) / 4;  //make right justified, adjust such that 1 bit = 0.25C
  
  //display temp in fahrenheit
  if (!c_not_f) {
    lm73_temp = ((lm73_temp * 9) / 5) + 32;
    if (dec_temp < 3) {dec_temp = dec_temp * 45;}
    else {lm73_temp += 1; dec_temp = 35;}

    itoa(lm73_temp, temp_str, 10); //convert to string in array with itoa() from avr-libc
    itoa(dec_temp, dec_str, 10); //convert dec to string in array with itoa() from avr-libc
    strcat(temp_str, ".");
    strcat(temp_str, dec_str);
    strcat(temp_str, "F");
    if (temp_str[4] == 'F') {temp_str[4] = '0'; temp_str[5] = 'F';}
  }
  else {  //display in celsius
    dec_temp = dec_temp * 25;
    itoa(lm73_temp, temp_str, 10); //convert to string in array with itoa() from avr-libc
    itoa(dec_temp, dec_str, 10); //convert dec to string in array with itoa() from avr-libc
    strcat(temp_str, ".");
    strcat(temp_str, dec_str);
    strcat(temp_str, "C");
    if (temp_str[4] == 'C') {temp_str[4] = '0'; temp_str[5] = 'C';}
  }
                        
}//lm73_temp_convert
//******************************************************************************
