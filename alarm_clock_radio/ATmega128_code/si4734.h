//Si4734 Addresses on the I2C bus
#define SI4734_ADDRESS                0x22   //fixed by Silicon Labs 

//property definitions
#define GPO_IEN                       0x0001
#define GPO_IEN_STCIEN                0x0001 
#define GPO_IEN_CTSIEN                0x0080 
#define AM_SOFT_MUTE_MAX_ATTENUATION  0x3302
#define AM_PWR_LINE_NOISE_REJT_FILTER 0x0100
#define AM_CHANNEL_FILTER             0x3102
#define AM_CHFILT_6KHZ                0x0000
#define AM_CHFILT_4KHZ                0x0001
#define AM_CHFILT_3KHZ                0x0002
#define AM_CHFILT_2KHZ                0x0003
#define AM_CHFILT_1KHZ                0x0004

//command definitions
#define FM_TUNE_FREQ    0x20
#define FM_PWR_UP       0x01
#define AM_PWR_UP       0x01
#define AM_TUNE_FREQ    0x40
#define PWR_DOWN        0x11
#define SET_PROPERTY    0x12
#define GET_INT_STATUS  0x14
#define FM_TUNE_STATUS_IN_INTACK 0x01
#define FM_TUNE_STATUS  0x22
#define FM_RSQ_STATUS_IN_INTACK 0x01
#define FM_RSQ_STATUS   0x23
#define AM_TUNE_STATUS_IN_INTACK 0x01
#define AM_TUNE_STATUS  0x42
#define AM_RSQ_STATUS   0x43
#define AM_RSQ_STATUS_IN_INTACK 0x01
#define GET_REV         0x10 

#define FALSE           0x00
#define TRUE            0x01

//si4734.c function prototypes
uint8_t get_int_status();
void    fm_tune_freq();
void    am_tune_freq();
void    sw_tune_freq();
void    fm_tune_status();
void    fm_rsq_status();
void    am_tune_status();
void    am_rsq_status();
void    fm_pwr_up();
void    am_pwr_up();
void    sw_pwr_up();
void    radio_pwr_dwn();
void    set_property();
void    get_rev();
void    get_fm_rsq_status();

uint8_t si4734_wr_buf[9];          //buffer for holding data to send to the si4734 
uint8_t si4734_rd_buf[15];         //buffer for holding data recieved from the si4734
uint8_t si4734_tune_status_buf[8]; //buffer for holding tune_status data  
uint8_t si4734_revision_buf[16];   //buffer for holding revision  data

//external radio setup variables
extern volatile uint8_t STC_interrupt;  //flag bit to indicate tune or seek is done

enum radio_band{FM, AM, SW};
extern volatile enum radio_band current_radio_band;

extern uint16_t eeprom_fm_freq;
extern uint16_t eeprom_am_freq;
extern uint16_t eeprom_sw_freq;
extern uint8_t  eeprom_volume;

extern uint16_t current_fm_freq;
extern uint16_t current_am_freq;
extern uint16_t current_sw_freq;
extern uint8_t  current_volume;

