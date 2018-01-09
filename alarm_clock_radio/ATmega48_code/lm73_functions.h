// lm73_functions.h 
// Daniel Eisenbach
// 12.03.2016
//special defines and functions for the lm73 temperature sensor

#include "twi_master.h"  //my TWCR_START, STOP, NACK, RNACK, SEND
//use the status codes in: usr/local/AVRMacPack/avr-3/include/util/twi.h

#define LM73_ADDRESS 0x90                    //LM73-0, address pin floating
#define LM73_WRITE (LM73_ADDRESS | TW_WRITE) //LSB is a zero to write
#define LM73_READ  (LM73_ADDRESS | TW_READ)  //LSB is a one to read
#define LM73_PTR_TEMP          0x00          //LM73 temperature address
#define LM73_PTR_CONFIG        0x01          //LM73 configuration address
#define LM73_PTR_CTRL_STATUS   0x04          //LM73 control and status register
#define LM73_CONFIG_VALUE0     0x60          //no pwr dwn, disbl alert, no one shot: config reg
#define LM73_CONFIG_VALUE1     0xE0          //no timeout, max resolution: for ctl/status reg

//special functions for lm73 temperature sensor
void lm73_temp_convert(uint8_t f_not_c);

//lm73 variables
uint8_t lm73_wr_buf[2];
uint8_t lm73_rd_buf[2];
uint16_t lm73_temp;  //a place to assemble the temperature from the lm73
char temp_str[16];  //stores string for temp

