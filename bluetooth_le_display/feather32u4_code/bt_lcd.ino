#include <Arduino.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_SPI.h>
#include <LiquidCrystal.h>

#define BLUEFRUIT_SPI_CS      8
#define BLUEFRUIT_SPI_IRQ     7
#define BLUEFRUIT_SPI_RST     4
#define DEBUG                 0
#define NUM_SIGNS             17

char vbat_str[16];  //string for storing battery voltage info string, Ex. "  3.68V"

//  initialize BT module object
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

//  initialize LCD control ports/pins  
LiquidCrystal lcd(12, 11, 10, 3, 6, 5);  //rs, en, d4, d5, d6, d7

// array of valid speed signs
char *speed_signs[NUM_SIGNS] = {"5", "10", "15", "20", "25", "30", "35", "40", "45", "50", "55", "60", "65", "70", "75", "80", "85"};

/*
 * adc_init()
 *
 * Input(s): N/A
 *
 * Post-Condition(s): 
 * ADC configured to use ADC12 to read the lipo
 * battery voltage connected to PB5.
 *
 * Overview: 
 * Initializes and configures the ADC to read the lipo
 * battery voltage. Uses AVcc for the reference voltage (AREF = 3.3V).
 */
void adc_init() {
  ADCSRA |= (1 << ADEN);  //enable ADC
  ADCSRB |= (1 << MUX5);  //choose ADMUX inputs to be ADC8-ADC15
  ADMUX |= (1 << REFS0) | (1 << MUX2);  //select ADC12 as single-ended input (MUX5:0 = 0b100100)
  ADMUX &= ~((1 << REFS1) | (1 << MUX0) | (1 << MUX1) | (1 << MUX3) | (1 << MUX4));
}// adc_init

/*
 * update_vbat()
 *
 * Input(s): N/A
 *
 * Post-Condition(s): 
 * vbat_str is set to "LOW BAT" if the lipo battery
 * voltage falls below 3.5V. If in debug mode vbat_str will store
 * the actual battery voltage. Otherwise, vbat_str is empty,
 * and used simply for LCD formatting.
 *
 * Overview: 
 * Reads the voltage of the lipo battery using ADC12. The voltage is
 * then built into a string. If the battery voltage falls below 3.5V
 * the string will contain a "LOW BAT" warning.
 */
void update_vbat() {
  uint8_t adc_low, adc_high;
  uint16_t adc_val;    

  // start reading VBAT
  ADCSRA |= (1 << ADSC);  //start ADC conversion
  
  // ADSC is cleared when the conversion finishes
  while (bit_is_set(ADCSRA, ADSC));

  // read ADC data
  // read ADCL first to lock ADCL and ADCH until ADCH is read
  adc_low = ADCL;
  adc_high = ADCH;

  adc_val = (adc_high << 8) | adc_low;  //combine ADC low and high bytes
  
  if (adc_val < 559) {strcpy(vbat_str, "   LOW BAT");}  // display low battery voltage warning below ~3.6V, ADC val = 559 = ((vbat / 2) / 3.3) * 1024
  else if (adc_val > 636) {strcpy(vbat_str, "  FULL BAT");}  // display full battery indicator above ~4.1V, ADC val = 636 = ((vbat / 2) / 3.3) * 1024
  else if (!DEBUG) {strcpy(vbat_str, "          ");}
  else{
    adc_val *= 2;  //vbat measured from voltage divider
    adc_val *= 33;  //multiply times reference voltage * 10 (AREF = 3.3V)


    // build battery voltage string
    strcpy(vbat_str, "     ");  // add buffer at beginning of string 
    vbat_str[5] = ((adc_val - (adc_val % 10240)) / 10240) + '0';  // store ones place of vbat by converting to char
    vbat_str[6] = '.';  // add decimal point to string
    vbat_str[7] = (((adc_val - (adc_val % 1024)) / 1024) % 10) + '0';  // store tenths place of vbat by converting to string
    vbat_str[8] = ((((adc_val * 10) - ((adc_val * 10) % 1024)) / 1024) % 10) + '0';  // store hundreths place of vbat by converting to string
    vbat_str[9] = 'V';
    vbat_str[10] = NULL;
  }
} //update_vbat

/*
 * main()
 *
 * Input(s): N/A
 *
 * Post-Condition(s): N/A
 *
 * Overview: 
 * Configures the Adafruit Feather 32U4 BLE
 * to receive speed sign data in the form of a string from
 * a connected BLE device. The received speed sign data is then
 * matched to a known valid speed sign string and output onto
 * a 2X16 LCD. If no BLE device is connected, then 
 * "NO CONNECTION" is output onto the LCD. Otherwise, speed sign
 * data is output in the following format:
 * "DETECTED SIGN: 55 MPH".
 */
int main()
{
  const char info_str[32];
  char temp_str[16];
  char lcd_str[33];
  char ble_data[4] = {0, 0, 0, 0};  // holds raw data received over BLE
  uint8_t startup_flag = 1;
  uint16_t vbat_update_timer;
  uint8_t ble_temp = 0;
  uint8_t count = 0;  //number of char bytes for bluetooth data string
  uint8_t connected_flag = 0;
  uint8_t new_data_flag = 0;
  uint8_t *data[5] = {0, 0, 0, 0, 0}; 

  init();  // initialize arduino setup
  adc_init();  // initialize ADC12 for reading the battery voltage
  sei();  //enable interrupts

  strcpy(info_str, "DETECTED SIGN:  ");
  strcpy(temp_str, "      ");  // initialize temp ble data string w/seven characters including the null appended
  strcpy(vbat_str, "          ");  // initialize battery data string w/eleven characters including the null appended

  lcd.begin(16,2);  // initalize dimensions of lcd display (columns, rows)
  lcd.clear(); 

  ble.begin();  // initialize communication between MCU and BLE module

  // reset BLE module to factory settings
  ble.factoryReset();

  // Change BLE module name to OCR Display
  ble.println("AT+GAPDEVNAME=OCR\ Display");
  ble.waitForOK();  // Check response status

  // Set module to DATA mode
  ble.setMode(BLUEFRUIT_MODE_DATA);

  while (1) {
    lcd.setCursor(0, 0);  // initialize LCD
    uint8_t row_1 = 0;
    
    vbat_update_timer++;

    if (ble.available()) {
      for (int i = 0; i < 4; i++) {ble_data[i] = 0;}  // clear old BLE data string
      count = 0; new_data_flag = 1;
    }
    while (ble.available() > 0) {   
      ble_temp = ble.read();
      data[count] = ble_temp;

      if (ble_temp == 10) {  // check for LF (newline) character that signals end of BLE transmission
        ble_data[count] = NULL;  // append NULL character to indicate end of Bluetooth data string
        break;  // break loop if '\n' is detected, indicates end of BT transmition
      }
      else if (count < 4) {ble_data[count] = (char)ble_temp; count++;}
    }

    // trash excess bluetooth data that doesn't fit on the screen
    while (ble.available() > 0) {
      ble_temp = ble.read();
    }

    // update battery voltage approx. every 30 seconds
    if (vbat_update_timer > 500 || startup_flag == 1) {
      update_vbat();
      startup_flag = 0;
      vbat_update_timer = 0;
    }

    // check ble device connection
    if (! ble.isConnected()) {
      strcpy(lcd_str, "NO CONNECTION         ");
      strcpy(temp_str, "      ");  // clear ble data string
      connected_flag = 0;
    }
    else if (!connected_flag) {connected_flag = 1;}

    if(connected_flag) {
      uint8_t valid_sign = 0;

      if (new_data_flag) {
        // compare rx bluetooth data to array of valid speed signs
        for (int i = 0; i < NUM_SIGNS; i++) {
          if (strcmp(ble_data, speed_signs[i]) == 0) {valid_sign = 1;}
        }
        if (valid_sign && (strcmp(ble_data, "5") == 0)) {
          strcpy(temp_str, ble_data);
          strcat(temp_str, " MPH ");  // append extra space after ble string to fill 32 char LCD buffer, bc "5" is only one char
        }
        else if (valid_sign) {
          strcpy(temp_str, ble_data);
          strcat(temp_str, " MPH");  // append " MPH" to received Bluetooth speed sign information
        }
        else {strcpy(temp_str, "SCAN  ");}

        new_data_flag = 0;  // indicate bluetooth string has been updated with new data
      }
 
      strcpy(lcd_str, info_str);
      strcat(lcd_str, temp_str);  // append received sign info to lcd string
    }
    strcat(lcd_str, vbat_str);

    if (DEBUG) {
      data[4] = (int)ble_data[2];
      itoa(data[4], lcd_str, 10);
      for (int i = strlen(lcd_str); i < 32; i++) {lcd_str[i] = ' ';}
    }

    for (int i = 0; i < 32; i++) {
      if ((i > 15)  && (row_1 == 0)) {lcd.setCursor(0, 1); row_1 = 1; delay(1);}  //start writing to 2nd lcd row if more than 15 chars
      lcd.write(lcd_str[i]);
    }

    delay(2);
  }
}
