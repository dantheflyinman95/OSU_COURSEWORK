//UART Functions header file for Mega 48 only
//Roger Traylor 12.7.15
//For controlling the UART and sending debug data to a terminal
//as an aid in debugging.

//functions
void uart_putc(char data);
void uart_puts(char *str);
void uart_puts_p(const char *str);
void uart_init();
char uart_getc(void);

//variables
volatile uint8_t  rcv_rdy;
char rx_char;
char rx_str[16];
