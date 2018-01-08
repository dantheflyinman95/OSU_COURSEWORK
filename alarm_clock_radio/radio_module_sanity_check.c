  char status[16];
  while( twi_busy() ){}; //spin while previous TWI transaction finshes
  si4734_wr_buf[0] = 0x21;  //command
  si4734_wr_buf[1] = 0x12;  //seek up and wrap around once top frequency hit
  twi_start_wr(SI4734_ADDRESS, si4734_wr_buf, 2);
  _delay_us(300);
  while( twi_busy() ){}; //spin while previous TWI transaction finshes
  twi_start_rd(SI4734_ADDRESS, status, 1);
  while( twi_busy() ){}; //spin while previous TWI transaction finshes
  //TODO: Read status[0], and make sure it is 0x80. This indicates the radio can properly receive and process your commands.

