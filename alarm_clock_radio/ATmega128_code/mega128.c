// final_project.c 
// D. Eisenbach
// 12.05.16

//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
//  PORTB bit 7 goes to the PWM transistor base.

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include "hd44780.h"
#include "si4734.h"
#include "twi_master.h"
#include "uart_functions.h"
#include "lm73_functions_skel.h"
//#define DEBUG

//holds data to be sent to the segments of the five 7seg num_digits
uint8_t seg_data[5]; 

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[11] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x98}; 

//decimal to 7-segment LED display encodingsi with dp
uint8_t dec_to_7seg_dp[11] = {0x40, 0x79, 0x24, 0x30, 0x19, 0x12, 0x02, 0x78, 0x00, 0x18}; 

//number of 7seg digits to be displayed at any given time
uint8_t num_digits;

//keep track of alarm clock mode, init everything to off
uint8_t mode = 0;

//map for showing volume on bargraph
uint8_t volume;  

//counter in main ISR for incrementing time, 488 = 1s
//used to time events in code, such as when to get temp data
uint16_t tcount = 0;

//track if alarm should be sounding and the digits flashing
uint8_t alarm_engaged = 0;
uint16_t alarm = 43200;  //initial value for alarm (12:00)
char alarm_str[16];  //stores "ALARM 12:00"

//LCD variables
char lcd_str_array[33];
uint8_t lcd_str_len = 0;
uint8_t lcd_refresh = 1;

//radio variables
volatile uint8_t STC_interrupt;
volatile enum radio_band current_radio_band = FM;
uint16_t current_fm_freq = 10370; //94.5Mhz, 200khz steps
uint16_t fm_freq_temp = 10370;
uint16_t eeprom_fm_freq;
uint16_t eeprom_am_freq;
uint16_t eeprom_sw_freq;
uint8_t eeprom_volume;
uint16_t current_am_freq;
uint16_t current_sw_freq;
uint8_t current_volume;
uint8_t show_fm_freq_timer;
uint8_t freq_dp;
uint8_t rssi;  //received signal strength indicator
char rssi_str[16];  //stores "SS 23", SS = signal strength
uint8_t radio_on = 0;  //flag for radio on condition
uint8_t power_up = 0;  //flag for turning on radio
uint8_t power_down = 0;  //flag for turning off radio
//end radio variables

char ext_temp_str[16];  //stores external temp data from mega48

/***********************************************************************/
//                            spi_init                               
// Initalizes the SPI port on the mega128, ADC and other registers.
// Sets up SPI to be:                        
// master mode, clock=clk/2, cycle half phase, low polarity, MSB first
// interrupts disabled, poll SPIF bit in SPSR to check xmit completion
//***********************************************************************/
void spi_init(void){
  //init spi
  DDRB = 0xF7; //output mode for SS, MOSI, SCLK, PB4-PB7, input for MISO
  SPCR |= (1 << MSTR) | (1 << SPE); //master mode, clk low on idle, leading edge sample
  SPSR |= (1 << SPI2X); //choose double speed operation

  //init ADC for PF0 (ADC0), used to read photoresistor voltage divider input
  DDRF &= ~(1 << PF0);  //set PF0 to input
  PORTF &= ~(1 << PF0);  //turn pullup off for PF0
  ADCSRA |= (1 << ADEN);  //enable ADC
  ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);  //division factor 128
  ADCSRA &= ~(1 << ADFR);  //single shot mode (free running disabled)
  ADMUX |= (1 << REFS0) | (1 << ADLAR);  //Vref = AVCC (~5V), left-adjust result
  ADMUX &= ~((1 << REFS1)); 
  ADMUX &= ~((1 << MUX0) | (1 << MUX1) | (1 << MUX2) | (1 << MUX3) | (1 << MUX4));
  ADCSRA |= (1 << ADSC);  //start ADC conversion

  //enable INT7 for radio GP02 on PE7
  EICRB |= (1 << ISC71) | (1 << ISC70);  //rising edge of PE7 generates interrupt
  EIMSK |= (1 << INT7);  //enable INT7
  
  DDRC |= (1 << PC1);  //output for alarm	
  DDRD |= (1 << PD2) | (1 << PD1) | (1 << PD0);  //output for strobing HC595 output data reg, TWI
  DDRE |= (1 << PE2) | (1 << PE3) | (1 << PE7);  //output for volume and radio reset 
}//spi_init

//***********************************************************************/
//                              tcntn_init                             
// Initalizes TCNT0, TCNT1, TCNT2, and TCNT3, which run off the 
// i/o clock.
//***********************************************************************/
void tcntn_init(void){
  //init TCNT0, used for main ISR
  TIMSK |= (1 << TOIE0);  //enable interrupts
  TCCR0 &= ~((1 << WGM01) | (1 << WGM00));  //normal mode
  TCCR0 |= (1 << CS02) | (1 << CS00);  //prescale by 128

  //init TCNT1, used for alarm sound output
  TIMSK |= (1 << OCIE1A);  //output compare match interrupt enable
  TCCR1A |= (1 << COM1A0);  //toggle OC1A on compare match
  TCCR1A &= (1 << COM1A1);  //toggle OC1A on compare match
  TCCR1A |= (1 << WGM10);  //phase correct pwm
  TCCR1A &= ~(1 << WGM11);
  TCCR1B &= ~((1 << WGM13) | (1 << WGM12));
  TCCR1B |= (1 << CS11) | (1 << CS10);  //prescale by 64
  TCCR1B &= ~(1 << CS12);
  OCR1A = 150;  //foc1a = 500Hz

  //init TCNT2, used for 7seg display PWM input
  TCCR2 |= (1 << WGM20);  //PWM mode, phase correct
  TCCR2 &= ~((1 << WGM21) | (1 << FOC2));
  TCCR2 &= ~(1 << COM20);  //toggle OC2 on compare match
  TCCR2 |= (1 << COM21);
  TCCR2 |= (1 << CS20);  //no prescale
  TCCR2 &= ~((1 << CS21) | (1 << CS22));
  OCR2 = 0x00;  //initialize OCR2 to give maximum 7seg brightness

  //init TCNT3, used for audio amp volume control
  TCCR3A |= (1 << COM3A1) | (1 << COM3A0);  //set OC3A on compare match
  TCCR3A |= (1 << WGM30);  //fast PWM, 8-bit
  TCCR3A &= ~(1 << WGM31);
  TCCR3B |= (1 << WGM32);
  TCCR3B &= ~(1 << WGM33);
  TCCR3B |= (1 << CS31);  //prescale by 8 
  TCCR3B &= ~((1 << CS32) | (1 << CS30));
  OCR3A = 200;  //124 = 2.5V, 255 = 0V, 1 = 0.02V
}//tcntn_init

//*************************************************************************/
//                            chk_buttons                                      
// Checks the state of the button number passed to it. It shifts in ones 
// till the button is pushed. Function returns a 1 only once per debounced
// button push so a debounce and toggle function can be implemented at the
// same time. Adapted to check all buttons from Ganssel's
// "Guide to Debouncing". Expects active low pushbuttons on PINA port.
// Debounce time is determined by external loop delay times 12. 
//*************************************************************************/
uint8_t chk_buttons(uint8_t button) {
  static uint16_t state[8] = {0,0,0,0,0,0,0,0}; //holds present state for all buttons
  state[button] = (state[button] << 1) | (! bit_is_clear(PINA, button)) | 0xE000;
  if (state[button] == 0xF000) return 1;
  return 0;
}//chk_buttons

//*************************************************************************/
//                             encoder_stm                                    
// Takes the previous state and current state of encoder as input 
// in addition to variables ccw, and cw that keep track of the encoder                      
// position in the 4-state machine. Returns direction encoder is
//  being rotated.
//*************************************************************************/
int encoder_stm(uint8_t cur_state, uint8_t prev_state) {  
  int8_t dir = 0;

  //counts states changed in state machine in ccw or cw directions
  static uint8_t ccw = 0;
  static uint8_t cw = 0;
  
  if (prev_state != cur_state) {
    if (prev_state == 0x03) {
      if (cur_state == 0x02) {ccw++;} 
        else if (cur_state == 0x01) {cw++;}
    }
    else if (prev_state == 0x02) {
      if (cur_state == 0x00) {ccw++;}
      else if (cur_state == 0x03) {cw++;}
    }
    else if (prev_state == 0x00) {
      if (cur_state == 0x01) {ccw++;}
      else if (cur_state == 0x02) {cw++;}
    }
    else if (prev_state == 0x01) {
      if (cur_state == 0x03) {ccw++;} 
      else if (cur_state == 0x00) {cw++;}
    }
  }
  if (ccw == 4) {dir = 1; ccw = 0;}  //four consecutive ccw turns detected set dir to 1
  else if (cw == 4) {dir = -1; cw = 0;}  //four consecutive cw turns detected set dir to -1
  else {dir = 0;}

  return dir;  //direction of the encoder rotation
}//encoder_stm

//*************************************************************************/
//                            fm_freq_segsum                                      
// Breaks up the fm frequency output into data displayable on 7seg digits,
// and flags which digit should receive a decimal point.
// Ex. 8870 --> 88.70
//*************************************************************************/
void fm_freq_segsum(uint16_t fm_freq) {
  //check where dp should be, and reduce freq > 10000 to 1000
  if (fm_freq >= 10000) {fm_freq = (fm_freq / 10); freq_dp = 1;}
  else {freq_dp = 2;}

  seg_data[0] = fm_freq % 10;  //ones place
  seg_data[1] = ((fm_freq % 100) - (fm_freq % 10)) / 10;  //tens place
  seg_data[2] = ((fm_freq % 1000) - (fm_freq % 100)) / 100;  //hundreds place
  seg_data[3] = (fm_freq - (fm_freq % 1000)) / 1000;  //thousands place 
}//fm_freq_segsum  

//*************************************************************************/
//                            clock_segsum                                      
// Breaks up the time input (s) into data displayable on 7seg digits
// in a 12 hour clock format.
//*************************************************************************/
void clock_segsum(uint16_t seconds) {
  uint8_t hours = (seconds - (seconds % 3600)) / 3600;
  uint8_t minutes = ((seconds - hours * 3600) - ((seconds - hours * 3600) % 60)) / 60;
  num_digits = 5;
  
  seg_data[0] = minutes % 10;
  seg_data[1] = (minutes - (minutes % 10)) / 10;
  seg_data[2] = hours % 10;
  seg_data[3] = (hours - (hours % 10)) / 10;
  seg_data[4] = 0x02;
}//clock_segsum  

//*************************************************************************/
//                           digit_sel                                    
// Takes in a number that indicates which digit needs to be displayed
// on the 7seg, selects the correct PORTB output to turn that digit on,
// and outputs the corresponding digit data from PORTA to the 7seg.
//*************************************************************************/
void digit_sel(uint8_t digit, uint8_t dp) {
  if (digit < num_digits) {  //only turn on digits that have a value to display
    switch(digit) {
      case 0: PORTB &= ~((1 << 4) | (1 << 5) | (1 << 6));
        break; 
      case 1:
        PORTB &= ~((1 << 5) | (1 << 6));
        PORTB |= (1 << 4);
        break;
      case 2:
        PORTB &= ~(1 << 6);
        PORTB |= (1 << 4) | (1 << 5);
        break;
      case 3:
        PORTB &= ~((1 << 4) | (1 << 5));
        PORTB |= (1 << 6);
        break;
      case 4:
        PORTB &= ~((1 << 4) | (1 << 6));
        PORTB |= (1 << 5);
        break;
      default :
        PORTB &= ~(1 << 4);
        PORTB |= (1 << 5) | (1 << 6);
    }

    //send 7seg encoding to LED segments
    if (dp) {PORTA = dec_to_7seg_dp[seg_data[digit]];}
    else {PORTA = dec_to_7seg[seg_data[digit]];}
  }
}//digit_sel

//*************************************************************************/
//                        rssi2str
// Converts the received signal strength indicator (rssi) value to a
// string, and concatenates it with "SS".
// Ex. "SS48"
//*************************************************************************/
void rssi2str() {
  char temp[16];
  rssi = si4734_tune_status_buf[4];
  itoa(rssi, temp, 10);  //convert int freq to string
  strcpy(rssi_str, "SS");
  strcat(rssi_str, temp);  //cat hour digit 1
}//rssi2str

//*************************************************************************/
//                        alarm2str
// Converts the alarm time to a string, and concatenates it with "ALARM ".
// Ex. "ALARM 12:00"
//*************************************************************************/
void alarm2str() {
  char temp[16];
  clock_segsum(alarm);
  strcpy(alarm_str, "ALARM ");
  itoa(seg_data[3], temp, 10);  //convert time to string
  strcat(alarm_str, temp);  //cat hour digit 1
  itoa(seg_data[2], temp, 10);
  strcat(alarm_str, temp);  //cat hour digit 2
  strcat(alarm_str, ":");  //cat colon
  itoa(seg_data[1], temp, 10);
  strcat(alarm_str, temp);  //cat minute digit 1
  itoa(seg_data[0], temp, 10);
  strcat(alarm_str, temp);  //cat minute digit 2
}//alarm2str

//*************************************************************************/
//                        lcd_format
// Concatenate strings to be displayed on the 32 char LCD and format
// LCD screen. Alarm, signal strength, inside temperature, and
// external temperature will be displayed on the LCD.
//*************************************************************************/
void lcd_format() {
  //intialize data strings
  alarm2str();
  rssi2str();

  if (bit_is_set(mode, 1) && radio_on) {  //display alarm, signal strength, and temp
    strcpy(lcd_str_array, alarm_str);
    strcat(lcd_str_array, " "); 
    strcat(lcd_str_array, rssi_str); 
  }
  else if (!bit_is_set(mode, 1) && radio_on) {
    strcpy(lcd_str_array, "ALARM OFF   ");
    strcat(lcd_str_array, rssi_str);
  }
  else if (bit_is_set(mode, 1) && !radio_on) {
    strcpy(lcd_str_array, alarm_str);
    strcat(lcd_str_array, "     "); 
  }
  else if (!bit_is_set(mode, 1) && !radio_on) {
    strcpy(lcd_str_array, "ALARM OFF       ");
  }
  else {strcpy(lcd_str_array, "                ");}

  //cat indoor and external temps
  strcat(lcd_str_array, temp_str); 
  strcat(lcd_str_array, "    "); 
  strcat(lcd_str_array, ext_temp_str); 

  lcd_str_len = strlen(lcd_str_array);

  uint8_t i;
  for (i = lcd_str_len; i < 32; i++) {lcd_str_array[i] = ' ';}
}//lcd_format


//*************************************************************************/
//                        map_vol2bargraph
// Takes the OCR3A volume volume control output and maps to to a byte LED
// range to be displayed on the bargraph. Each LED is a 16 unit step in
// volume, or 0.32V change.
// Volume max: OCR3A = 128, all 8 LEDs on
// Volume off: OCR3A = 255, all LEDs off
//*************************************************************************/
void map_vol2bargraph() {
    if (OCR3A >= 128 && OCR3A < 144) {volume = 0xFF;}  //128 = max volume, 16 unit steps between vol bars
    else if (OCR3A >= 144 && OCR3A < 160) {volume = 0xFE;}
    else if (OCR3A >= 160 && OCR3A < 176) {volume = 0xFC;}
    else if (OCR3A >= 176 && OCR3A < 192) {volume = 0xF8;}
    else if (OCR3A >= 192 && OCR3A < 208) {volume = 0xF0;}
    else if (OCR3A >= 208 && OCR3A < 224) {volume = 0xE0;}
    else if (OCR3A >= 224 && OCR3A < 240) {volume = 0xC0;}
    else if (OCR3A >= 240 && OCR3A < 255) {volume = 0x80;}
    else {volume = 0x00;}  //255 = volume off
}

//*************************************************************************/
//                            INT7 ISR
// External interrupt 7 is on Port E bit 7. The interrupt is triggered 
// on the rising edge of Port E bit 7.  The i/o clock must be running
// to detect the edge (not asynchronouslly triggered).
//*************************************************************************/
ISR(INT7_vect){STC_interrupt = TRUE;}
//*************************************************************************/

//*************************************************************************/
//                           USART0_RX ISR                          
// When USART0 finishes receivign data, this interrupt triggers.
//*************************************************************************/
ISR(USART0_RX_vect){
  static  uint8_t  i;
  rx_char = UDR0;              //get character
  rx_str[i++] = rx_char;  //store in array 
 //if entire string has arrived, set flag, reset index
  if(rx_char == '\0'){
    rcv_rdy = 1;
    i = 0;  
  }
}

//*************************************************************************/
//                           timer/counter1 ISR                          
// When the TCNT1 OC1A compare match interrupt occurs, the alarm sounds
// if the alarm is engaged.
//*************************************************************************/
ISR(TIMER1_COMPA_vect){
  //if the alarm is engaged, turn off radio,
  //and twiddle PC1 to create the alarm sound output
  if(alarm_engaged) {
    if (radio_on) {power_down = 1;}  //flag that radio needs to be turned off
    if (!power_down) {PORTC ^= (1 << PC1);}
  }
    
  //display data on LCD
  if (lcd_refresh) {
    //format the data to be displayed on the LCD
    lcd_format();
    _delay_us(50);
    refresh_lcd(lcd_str_array);
  }
}

//*************************************************************************/
//                           timer/counter0 ISR                          
// When the TCNT0 overflow interrupt occurs, the buttons and encoders are
// scanned for changes, and the time iterator tcount is incremented.
// The clock and alarm can be set using the encoders and buttons.
// The time is output to the 7seg display.
//*************************************************************************/
ISR(TIMER0_OVF_vect){
  //encoder state machine variables
  static uint8_t prev_state1 = 0x03;
  static uint8_t cur_state1;
  static uint8_t prev_state2 = 0x03;
  static uint8_t cur_state2;

  //keep track of encoder rotation direction
  uint8_t encoder_data;  //combined encoder data
  int16_t enc1_dir;  //encoder 1 direction 
  int8_t enc2_dir;  //encoder 2 direction
  
  //clock and alarm variables
  static uint16_t time = 43200;  //initial value for time (12:00)
  static uint32_t t_error = 0;  //error correction for time advancing
  static uint8_t colon = 1;  //track if colon should be displayed
  static uint8_t flash_digits = 0;  //track if set mode should be indicated
  static uint16_t flash_count = 0;  //track when digits should be flashed for set mode

  //ADC variable for controlling 7seg brightness using photoresistor
  static uint8_t adc_result = 0;
  if (bit_is_set(ADCSRA, ADIF)) {
    adc_result = ADCH;
    ADCSRA |= (1 << ADSC) | (1 << ADIF);  //clear completion flag, start next conversion

    //adjust PWM output using ADC converted input from photoresistor
    if (255 - adc_result * 1.5 < 0) {OCR2 = 0;}
    else if (255 - adc_result * 1.5 > 252) {OCR2 = 252;}
    else {OCR2 = 255 - adc_result * 1.5;}
  }

  //make PORTA an input port with pullups
  DDRA = 0x00;
  PORTA = 0xFF;

  //enable tristate buffer for pushbutton switches
  PORTB |= (1 << 4) | (1 << 5) | (1 << 6);
  
  //check buttons and toggle mode
  //mode 1: enable alarm, mode 2: set alarm, mode 3: snooze
  //mode 4: enable radio, mode 5: seek tune, mode 7: set time
  uint8_t i;  //counter to keep track of buttons
  for (i = 0; i < 8; i++) {
    if (chk_buttons(i) == 1) {
      if (i != 0) {
        mode ^= (1 << i);  //only toggle button pressed

        //prevent alarm and clock from being set simulataneously
        if (bit_is_set(mode, 7) && bit_is_set(mode, 2)) {
          mode &= ~((1 << 7) | (1 << 2));  
        }
      }
    }
  }    	 

  //disable tristate buffer for pushbutton switches (DEC5 - NC)
  PORTB |= (1 << 4) | (1 << 6);
  PORTB &= ~(1 << 5);

  //mode 1: alarm
  //if alarm set, display "ALARM 12:00" on LCD
  //if the alarm is engaged, flash digits and sound alarm
  if (alarm == time && bit_is_set(mode, 1) && !bit_is_set(mode, 2) && !bit_is_set(mode, 7)) {alarm_engaged = 1;}

  if (alarm_engaged && bit_is_set(mode, 1)) {
    flash_digits = 1;
  }
  else {flash_digits = 0; alarm_engaged = 0;}

  //mode 3: snooze 10 seconds 
  if (bit_is_set(mode, 3)) {alarm = alarm + 60; mode &= ~(1 << 3); alarm_engaged = 0;}

  //mode 4: turn on radio if mode selected and alarm not engaged
  if (bit_is_set(mode, 4) && !radio_on && !alarm_engaged) {power_up = 1;}    
  else if (radio_on && !bit_is_set(mode, 4)) {power_down = 1;}

  if (bit_is_set(mode, 7) || bit_is_set(mode, 2)) {lcd_refresh = 0;}
  else if (tcount % 10 == 0) {lcd_refresh = 1;}  //don't update LCD if encoder's being used 
  else {lcd_refresh = 0;}
 
 //scan encoders and output mode to bargraph
  SPDR = volume;  //write volume level to HC595 to start SPI and to display on bargraph
  while (bit_is_clear(SPSR, SPIF)){}  //wait till data is sent out
  PORTD |= (1 << PD2);  //strobe HC595 output data reg - rising edge
  PORTD &= ~(1 << PD2);  //falling edge
  PORTB &= ~(1 << PB0);  //strobe HC165 SH/!(LD) reg - falling edge
  encoder_data = SPDR;  //save encoder data
  PORTB |= (1 << PB0);  //rising edge
  
  cur_state1 = encoder_data & 0x03;  //separate encoder1 data
  cur_state2 = (encoder_data & 0x0C) >> 2;  //separate encoder2 data
  
  enc1_dir = encoder_stm(cur_state1, prev_state1);  //run state machine for encoder 1
  enc2_dir = encoder_stm(cur_state2, prev_state2);  //run state machine for encoder 2

  prev_state1 = cur_state1;  //set new prev_state value for encoder 1
  prev_state2 = cur_state2;  //set new prev_state value for encoder 2

  //enc1_dir = hours, enc2_dir = seconds
  if (bit_is_set(mode, 7) && !alarm_engaged) {
    time = time - (time % 60);  //clear seconds place of time 
    time = time + enc1_dir * 3600 + enc2_dir * 60;  //inc time by encoder directions
    flash_digits = 1;
    show_fm_freq_timer = 0;  //stop displaying fm freq
  }
  else if (bit_is_set(mode, 2) && !alarm_engaged) {
    alarm = alarm + enc1_dir * 3600 + enc2_dir * 60;  //inc alarm by encoder directions
    flash_digits = 1;  //make digits flash
    show_fm_freq_timer = 0;  //stop displaying fm freq
  }
  else if (!alarm_engaged && bit_is_set(mode, 4)) {  //adjust volume or  FM frequency if radio on
    //adjust volume
    if (enc1_dir < 0) {  //turn down volume
      if (OCR3A - 10 * enc1_dir > 255) {OCR3A = 255;}
      else {OCR3A = OCR3A - 10 * enc1_dir;}
      map_vol2bargraph();  //convert OCR3A volume output to bargraph encoding
    }
    if (enc1_dir > 0) {  //turn up volume
      if (OCR3A - 10 * enc1_dir < 128) {OCR3A = 128;}
      else {OCR3A = OCR3A - 10 * enc1_dir;}
      map_vol2bargraph();  //convert OCR3A volume output to bargraph encoding
    }

    //adjust fm freq
    if (enc2_dir > 0) {
      if (current_fm_freq + 20 > 10790) {current_fm_freq = 8810;}
      else {current_fm_freq += 20;}  //increase fm freq tune
    }
    else if (enc2_dir < 0) {
      if (current_fm_freq - 20 < 8810) {current_fm_freq = 10790;}
      else {current_fm_freq -= 20;}  //increase fm freq tune
    }
  }
  else {  //adjust volume
    if (enc1_dir < 0) {  //turn down volume
      if (OCR3A - 10 * enc1_dir > 255) {OCR3A = 255;}
      else {OCR3A = OCR3A - 10 * enc1_dir;}
      map_vol2bargraph();  //convert OCR3A volume output to bargraph encoding
    }
    if (enc1_dir > 0) {  //turn up volume
      if (OCR3A - 10 * enc1_dir < 128) {OCR3A = 128;}
      else {OCR3A = OCR3A - 10 * enc1_dir;}
      map_vol2bargraph();  //convert OCR3A volume output to bargraph encoding
    }
  }

  //488 ~= 1 sec, inc if not set time mode, toggle colon on/off every second
  if (bit_is_clear(mode, 7)) {tcount++; t_error++;}
  else {tcount = 0; t_error = 0;}
  flash_count++;

  //iterate time and subtract one sec every 29 minutes to reduce error
  if (tcount >= 488) {
    time++; tcount = 0; 
    if (flash_digits != 1) {colon ^= (1 << 0);}  //toggle colon if not in set mode
    if (show_fm_freq_timer > 0) {show_fm_freq_timer--;}
  }
  if (time >= 46800) {time = time - 46800 + 3600;}  //rollover time from 12:59 to 1:00
  else if (time < 3600) {time = time + 46800 - 3600;}  //rollover time from 1 to 12:59
  if (t_error >= 846734) {time = time - 1; t_error = 0;}  //subtract one sec every 29min
  if (alarm >= 46800) {alarm = alarm - 46800 + 3600;}  //rollover alarm from 12:59 to 1:00
  else if (alarm < 3600) {alarm = alarm + 46800 - 3600;}  //rollover alarm from 1 to 12:59
  if (flash_count >= 488) {flash_count = 0;}  //reset flash count every second
  
  //if in set mode, blink digits rapidly
  if (flash_digits == 1) {
    if ((flash_count == 120) | (flash_count == 242) | (flash_count == 364) | (flash_count == 486)) {
      colon ^= (1 << 0);
    }
  }
  
  //break up the time (s) to 4, BCD digits in the seg_data rray
  //if the alarm set mode isn't selected, display time
  if (bit_is_set(mode, 2)) {clock_segsum(alarm);}
  else {clock_segsum(time);}
  
  DDRA = 0xFF;  //make PORTA an output

  //show fm freq if fm mode recently selected, or freq recently changed
  if (show_fm_freq_timer && !alarm_engaged) {
    //keep track of digit to display for each interrupt
    static uint8_t digit = 0;
    digit++;
    if (digit > 3) {digit = 0;} 
    
    fm_freq_segsum(current_fm_freq);
 
    if (freq_dp == 1 && digit == 1) {digit_sel(digit, 1);}
    else if (freq_dp == 2 && digit == 2) {digit_sel(digit, 2);}
    else {digit_sel(digit, 0);}
  }

  //show time or alarm
  else {
    //keep track of digit to display for each interrupt
    static uint8_t digit = 0;
    digit++;

    if (digit > 4) {digit = 0;} 
    if (flash_digits == 1) {
      if (colon == 1 && digit != 4) {digit_sel(digit, 0);}  //display digits only when colon is toggled on
      if (digit == 4) {digit_sel(digit, 0);}  //always display colon segments
    }
    else {
      if (digit != 4) {digit_sel(digit, 0);}  //always display digits
      if (digit == 4 && colon == 1) {digit_sel(digit, 0);}  //display colon for one sec every other sec
    }
  }
}

//*************************************************************************/
int main() {
  tcntn_init();  //initialize timers
  uart_init();
  spi_init();  //initialize SPI, ADC, and i/0
  init_twi();  //initialize TWI, used for radio and LM73
  lcd_init();  //intialize LCD (lcd_functions.h)
  clear_display();
  map_vol2bargraph();  //convert OCR3A volume output to bargraph encoding

  //hard reset radio
  DDRE  |= 0x04; //Port E bit 2 is active high reset for radio 
  PORTE |= 0x04; //radio reset is on at powerup (active high)

  //hardware reset of Si4734
  PORTE &= ~(1<<PE7); //int2 initially low to sense TWI mode
  DDRE  |= 0x80;      //turn on Port E bit 7 to drive it low
  PORTE |=  (1<<PE2); //hardware reset Si4734 
  _delay_us(200);     //hold for 200us, 100us by spec         
  PORTE &= ~(1<<PE2); //release reset 
  _delay_us(30);      //5us required because of my slow I2C translators I suspect
                        //Si code in "low" has 30us delay...no explaination
  DDRE  &= ~(0x80);   //now Port E bit 7 becomes input from the radio interrupt

  //set LM73 mode for reading temperature by loading pointer register
  //this is done outside of the normal interrupt mode of operation 
  lm73_wr_buf[0] = LM73_PTR_TEMP;   //load lm73_wr_buf[0] with temperature pointer address
  twi_start_wr(LM73_ADDRESS, lm73_wr_buf, 1);   //start the TWI write process (twi_start_wr())

  sei();  //enable interrupts

  while(1){
    //send POWER_UP to si4734 if flag set
    if (power_up) {
      //powerup radio, added 50ms to 120ms delay in fm_pwr_up() to avoid flakiness/static
      while( twi_busy() ){}; //spin while previous TWI transaction finshes
      fm_pwr_up(); radio_on = 1;

      //set FM radio tune freq. and get signal status info
      while( twi_busy() ){}; //spin while previous TWI transaction finshes
      show_fm_freq_timer = 5;  //show fm freq for 5 seconds
      fm_tune_freq();
      fm_freq_temp = current_fm_freq;
      power_up = 0; radio_on = 1;
    }

    //send POWER_DOWN to si4734 if flag set
    if (power_down) {
      while( twi_busy() ){}; //TODO: verify IF NEEDED
      radio_pwr_dwn();
      power_down = 0; radio_on = 0;
    }

    if (radio_on && (fm_freq_temp != current_fm_freq)) {
      show_fm_freq_timer = 5;  //show fm freq for 5 seconds
      _delay_ms(1000);

      while( twi_busy() ){}; //spin while previous TWI transaction finshes
      fm_tune_freq();
      fm_freq_temp = current_fm_freq;
    }

    //if the radio is on, get the signal status
    if (radio_on) {
      while( twi_busy() ){}; //spin while previous TWI transaction finshes
      fm_rsq_status();
    }

    //mode 5: FM_SEEK_START
    if (bit_is_set(mode, 5) && radio_on) {
      mode &= ~(1 << 5);  //turn off seek mode
      show_fm_freq_timer = 5;  //show fm freq for 5 seconds

      //send FM_SEEK_START command
      while( twi_busy() ){}; //spin while previous TWI transaction finshes
      si4734_wr_buf[0] = 0x21;  //command
      si4734_wr_buf[1] = 0x0C;  //seek up and wrap
      twi_start_wr(SI4734_ADDRESS, si4734_wr_buf, 2);
      _delay_ms(1000);  //wait for command to finish

      //send FM_TUNE_STATUS to read new fm freq
      while( twi_busy() ){}; //spin while previous TWI transaction finshes
      si4734_wr_buf[0] = 0x22;  //command
      si4734_wr_buf[1] = 0x00;
      twi_start_wr(SI4734_ADDRESS, si4734_wr_buf, 2);
      _delay_us(300);  //wait for command to finish
      while( twi_busy() ){}; //spin while previous TWI transaction finshes
      twi_start_rd(SI4734_ADDRESS, si4734_rd_buf, 8);
      _delay_us(300);  //wait for command to finsih
      current_fm_freq = (uint16_t)(si4734_rd_buf[2] << 8) + si4734_rd_buf[3];
      fm_freq_temp = (uint16_t)(si4734_rd_buf[2] << 8) + si4734_rd_buf[3];
    }

    //get temp data every 1s, display in fahrenheit
    if (tcount % 488 == 0) {
      //internal temp
      if (bit_is_set(mode, 6)) {lm73_temp_convert(1);}  //display temp in C
      else {lm73_temp_convert(0);}  //display temp in F
  
      //get external temp data from mega48
      if (bit_is_set(mode, 6)) {uart_putc('C');}  //display ext temp in C
      else {uart_putc('F');}  //display ext temp in F
    }

    if (rcv_rdy) {strcpy(ext_temp_str, rx_str); rcv_rdy = 0;}
  }
}//main
