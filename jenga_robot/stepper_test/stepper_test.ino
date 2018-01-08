#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>
#include <PID_v1.h>
#include <Servo.h>

#define MSTEPS           200  // steps per revolution
#define MSPEED           50  // rpm
#define STEP_DELAY       ((60L * 1000L * 1000L) / MSTEPS / MSPEED)  // delay between steps, in us, ctrls rpm

#define ARM_LEN_EN       14
#define ARM_LEN_DIR      15
#define ARM_LEN_PUL      16
#define ARM_HT_MP1       9  // PWM outputs to L298N H-Bridge motor driver module
#define ARM_HT_MP2       10

#define encod_PINA       2  //quadrature encoder A pin (gray)
#define encod_PINB       7  //quadrature encoder B pin (blue)

#define CLAMP_MP1        13

int stepped = 0;
uint8_t step_dir;  //motor step direction
uint16_t step_number = 0;

volatile double encoderPos = 0;
int arm_raised = 0;

int clamp_close = 40;
int clamp_open = 80;
int clamped = 0;

//aggressive and conservative PID tuning Parameters
double aggKp=500, aggKi=1000, aggKd=0.05;
double consKp=50, consKi=1000, consKd=0.01;
double input = 0, output = 0, setpoint = 0;

PID arm_ht_PID(&input, &output, &setpoint, consKp, consKi, consKd, DIRECT);
Servo clamp;

void setup() {
  pinMode(ARM_HT_MP1, OUTPUT);
  pinMode(ARM_HT_MP2, OUTPUT);
  pinMode(ARM_LEN_EN, OUTPUT);
  pinMode(ARM_LEN_DIR, OUTPUT);
  pinMode(ARM_LEN_PUL, OUTPUT);
  pinMode(encod_PINA, INPUT_PULLUP);  //quadrature encoder input A
  pinMode(encod_PINB, INPUT_PULLUP);  //quadrature encoder input B

  clamp.attach(CLAMP_MP1);

  //TCCR1B = TCCR1B & 0b11111000 | 1;  //set 31KHz PWM to prevent motor noise

  // Timer0 is already used for millis() - we'll just interrupt somewhere
  // in the middle and call the "Compare A" function below
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);

  attachInterrupt(0, encoder, FALLING);  //update encoder position

  arm_ht_PID.SetMode(AUTOMATIC);
  arm_ht_PID.SetSampleTime(1);
  arm_ht_PID.SetOutputLimits(-255, 255);
  
  Serial.begin (115200);  //for debugging
}

void loop() {
  setpoint = -6;
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

void arm_ht_check(){
  input = encoderPos ;  //data from encoder
  double gap = abs(setpoint - input);
  //Serial.println(encoderPos);  //monitor motor position
  Serial.println(gap);

  if (gap < 0.5) arm_raised = 1;
  else arm_raised = 0;
  
  if (gap < 20) arm_ht_PID.SetTunings(consKp, consKi, consKd);  //close to setpoint
  else arm_ht_PID.SetTunings(aggKp, aggKi, aggKd);  //far from setpoint

  arm_ht_PID.Compute();  //calculate new output
  hbridge_pwmOut(output);  //drive L298N H-Bridge module
}

// Interrupt is called once a millisecond, 
SIGNAL(TIMER0_COMPA_vect) {
  arm_ht_check();  //PID control
}

// pulse and direction, direct port reading to save cycles
void encoder()  {
  if (digitalRead(encod_PINB) == HIGH) encoderPos = encoderPos + 0.01;
  else  encoderPos = encoderPos - 0.01;
}

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
