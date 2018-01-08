/* robot_ctrl.h 
 * D. Eisenbach
 * 5.24.17
 *
 * The sequence of control signals for 4 control wire stepper motors is as follows:
 *
 * Step C0 C1 C2 C3
 *    1  1  0  1  0
 *    2  0  1  1  0
 *    3  0  1  0  1
 *    4  1  0  0  1
 */

#define MSTEPS           200  // steps per revolution
#define MSPEED           50   // rpm
#define STEP_DELAY       ((60L * 1000L * 1000L) / MSTEPS / MSPEED)  // delay between steps, in us, ctrls rpm

#define ARM_HT_MP1       9  // PWM outputs to L298N H-Bridge motor driver module
#define ARM_HT_MP2       10

#define ARM_LEN_EN       14
#define ARM_LEN_DIR      15
#define ARM_LEN_PUL      16

#define CLAMP_PIN        11
#define HINGE_PIN        6

#define BB_SENSOR_PIN    12

#define ARM_EXT_PIN      7
#define ARM_RETRACT_PIN  13
#define ARM_LOWERED_PIN  5

#define ENCOD_PINA       2  //quadrature encoder A pin (blue)
#define ENCOD_PINB       8  //quadrature encoder B pin (gray)

#define TOWER_INIT_PIN   4  //input that signals orientation of Jenga tower; 0 = top row (16th) is first sideblock

#define START_PIN        19  //start turn input from game arbitrator

//state machine variables
enum {WAIT = 0x00, START = 0x01, EXT_ARM = 0x02, CHECK_BLOCK = 0x03, CLEAR_TOWER = 0x05, LOWER_ARM = 0x06, RETURN_HOME = 0x07}; 
int cntrl_ps, cntrl_ns; 

//flags
volatile int isSideBlock = 0;  //set by break beam sensor input
volatile int counter = 0;
volatile int arm_raised = 0;
volatile int raise_arm = 0;
volatile int lower_arm = 0;
int arm_extended = 0;
int arm_retracted = 0;
int grab_block = 0;
int tower_cleared = 0;
int playable_blocks [15] = {0};  //flags playable side blocks of the Jenga tower
int done = 0;
int end_game = 0;  //flag if robot needs to knock over tower

int topmost_block;  //next Jenga block to be removed

volatile double encoderPos;
uint8_t step_dir;  //motor step direction
uint16_t step_number = 0;

//motor movement distances
int steps_clear_hinge = 100;  //fine tuning for arm length extension after block detected
int steps_clear_tower = 100;
double rot_per_row = 8;
int clamp_close = 26;
int clamp_open = 90;
int hinge_lower = 50;
int hinge_raise = 230;

//aggressive and conservative PID tuning Parameters
double aggKp=5000, aggKi=10000, aggKd=0.05;
double consKp=500, consKi=5000, consKd=0.01;
double input = 0, output = 0, setpoint = 0, gap = 0;

PID arm_ht_PID(&input, &output, &setpoint, consKp, consKi, consKd, DIRECT);
Adafruit_SoftServo clamp;
Adafruit_SoftServo hinge;

// pulse and direction, direct port reading to save cycles
void encoder()  {
  if (digitalRead(ENCOD_PINB) == HIGH) encoderPos = encoderPos + 0.01;
  else  encoderPos = encoderPos - 0.01;
}

// break beam sensor object detected interrupt
void bb_sensor_ISR() {
  if (digitalRead(ARM_EXT_PIN) == HIGH)  //indicates arm not extended around middle block
    isSideBlock = 1;  // flag that break beam sensor detected object
 
  // turn Arduino LED on:
  digitalWrite(LED_BUILTIN, HIGH);
}

/*
 * Moves the motor steps_to_move steps.  If the number is negative,
 * the motor moves in the reverse direction.
 */
void step(int steps_to_move) {
  int steps_left = abs(steps_to_move);  //how many steps to take

  //determine direction based on whether steps_to_mode is + or -:
  if (steps_to_move > 0) {step_dir = 1;}
  if (steps_to_move < 0) {step_dir = 0;}

  //decrement the number of steps, moving one step each time:
  while (steps_left > 0) {
    //increment or decrement the step number, depending on direction:
    if (step_dir == 1) {
      step_number++;
      if (step_number == MSTEPS) {step_number = 0;}
    }
    else {
      if (step_number == 0) {  //count down from num steps to move
        step_number = MSTEPS;
      }
    
      step_number--;
    }
  
    //decrement the steps left:
    steps_left--;
  
    //step the motor to step phase number
    if (step_dir == 1) {  //step fwd
      digitalWrite(ARM_LEN_DIR, LOW);
      digitalWrite(ARM_LEN_EN, HIGH);
      digitalWrite(ARM_LEN_PUL, HIGH);
      delayMicroseconds(50);
      digitalWrite(ARM_LEN_PUL, LOW);
    }
    else {  //step bck
      digitalWrite(ARM_LEN_DIR, HIGH);
      digitalWrite(ARM_LEN_EN, HIGH);
      digitalWrite(ARM_LEN_PUL, HIGH);
      delayMicroseconds(50);
      digitalWrite(ARM_LEN_PUL, LOW); 
    } 

    delayMicroseconds(STEP_DELAY);
  }
}

void hbridge_pwmOut(int out) {  //to H-Bridge board
  if (out > 0) {
    analogWrite(ARM_HT_MP1, out);  //drive motor CW
    analogWrite(ARM_HT_MP2, 0);
  }
  else {
    analogWrite(ARM_HT_MP1, 0);
    analogWrite(ARM_HT_MP2, abs(out));  //drive motor CCW
  }
}

void init_playable_blocks(void) {
  if (digitalRead(TOWER_INIT_PIN) == LOW) {  //three rows below top is first playable row
    playable_blocks[0] = 0;
    playable_blocks[1] = 1; playable_blocks[2] = 0;  //bottom playable row
    playable_blocks[3] = 1; playable_blocks[4] = 0;
    playable_blocks[5] = 1; playable_blocks[6] = 0;
    playable_blocks[7] = 1; playable_blocks[8] = 0;
    playable_blocks[9] = 0; //top playable row
  }
  else {  //4 rows below top is first playable side block
    playable_blocks[0] = 0;  //flag to check if there are no playable rows left
    playable_blocks[1] = 0; playable_blocks[2] = 1;  //bottom playable row
    playable_blocks[3] = 0; playable_blocks[4] = 1;
    playable_blocks[5] = 0; playable_blocks[6] = 1;
    playable_blocks[7] = 0; playable_blocks[8] = 0;
    playable_blocks[9] = 0; //top playable row
  }
}

void arm_ht_check(){
  if (end_game) lower_arm = 1;
  if (!lower_arm) {
    if (arm_extended && raise_arm) {
	    topmost_block = 0;
      for (int i = 9; i >= 0; i--) {
	      if (playable_blocks[i] == 1) {
          topmost_block = i;
		      break;
	      }
	    }
   
      if (topmost_block == 0) end_game = 1;
      else {
        switch(topmost_block) {
          case 1: setpoint = 6; break;
          case 2: setpoint = 12; break;
          case 3: setpoint = 22; break;
          case 4: setpoint = 32; break;
          case 5: setpoint = 40; break;
          case 6: setpoint = 50; break;
          case 7: setpoint = 56; break;
          case 8: setpoint = 65; break;
          case 9: setpoint = 71; break;
        }
      }
    }
  }
  else {
    if (digitalRead(ARM_LOWERED_PIN) == LOW) {
      lower_arm = 0;
      encoderPos = 0;
    }
    else setpoint = -100;
  }
  
  input = encoderPos ;  //data from encoder
  gap = abs(setpoint - input);
  //Serial.println(setpoint);
  //Serial.println(encoderPos);  //monitor motor position
  //if (isSideBlock) Serial.println("isSideBlock");

  if ((gap < 0.2) && raise_arm && (setpoint != -100)) arm_raised = 1;
  else arm_raised = 0;
  
  if (gap < 20) arm_ht_PID.SetTunings(consKp, consKi, consKd);  //close to setpoint
  else arm_ht_PID.SetTunings(aggKp, aggKi, aggKd);  //far from setpoint

  arm_ht_PID.Compute();  //calculate new output
  hbridge_pwmOut(output);  //drive L298N H-Bridge module  TODO: uncomment this line
}

// Interrupt is called once a millisecond, 
SIGNAL(TIMER0_COMPA_vect) {
  if (digitalRead(BB_SENSOR_PIN) == LOW) isSideBlock = 1;  //indicates found side block

  counter += 2;
  // every 20 milliseconds, refresh the servos!
  if (counter >= 20) {
    counter = 0;
    clamp.refresh();
    hinge.refresh();
  }
  
  arm_ht_check();  //PID control
}
