/* Team TOM (B)
 * Motor Lab v1
 * 
 * Pinout:
 * Hall Effect (Analog)
 * A2: OUT
 * 
 * FSR (ADC)
 * A0: left/fixed side
 * A1: right/variable side
 * Tested with wheastone bridge, 2.2k ohms, placed on left pin and right pin as B and D
 * 
 * Optical (Digital)
 * D2: S0
 * D3: S1
 * D4: S2
 * D5: S3
 * D8: OUT
 * 
 * Thermocouple (ADC)
 * A5: OUT
 * 
 * Ultrasonic (Digital)
 * D12: Trigger
 * D13: Echo
 * 
 * Potentiometer (ADC)
 *  A4: tap
 *  
 * 
 * Version history:
 * 2/1/16: Nishant Pol: Initial release, combinaion of ldc1000, fsr, and matlab interface arduino code
 */
#include <FreqMeasure.h>
//#include <PID_v1.h>
#include <Servo.h>
#include <Stepper.h>

/* Hall Sensor Macros and Globals */

const uint8_t hall_pin = A2;

/* END Hall Sensor Macros and Globals */

/* FSR Macros and Globals */

const int ptD = A0; // leftmost side
const int ptB = A1; // right most sides
int sensorValue = 0; 

int sensorB, sensorD = 0;

/* END FSR Macros and Globals */

/* Optical Macros and Globals */

uint8_t optical_S0_pin = 2;
uint8_t optical_S1_pin = 3;
uint8_t optical_S2_pin = 4;
uint8_t optical_S3_pin = 5;

/* END Optical Macros and Globals */

/* Thermocouple Macros and Globals */

const int TEMP_IN = A5; 
int sensor_value;
float temp;
float k = 0.8227; 
float b = 76.971;

/* END Thermocouple Macros and Globals */

/* Ultrasonic Macros and Globals */

const int TRIG_PIN = 12;
const int ECHO_PIN = 13;

/* END Ultrasonic Macros and Globals */

/* DC Motor Macros and Globals */ 
 const int encoder0PinA = 9;
 const int encoder0PinB = 10;
 const int dcmotor_I1 = 6;  //Speed
 const int dcmotor_I2 = 7;  //Direction
 int encoder0Pos = 0;
 int encoder0PinALast = LOW;
 int n = LOW;

 //PID
 //PID library code from http://playground.arduino.cc/Code/PIDLibrary
 //enum pid_mode_type {PIDMODE_OFF, PIDMODE_SPEED, PIDMODE_POSITION};
 //pid_mode_type pid_mode = PIDMODE_POSITION;
 double Input, Output;
 double Setpoint = 3;
 //const double Kp = 10;
 //const double Ki = 5;
 //const double Kd = 0;
 //Specify the links and initial tuning parameters
// PID myPID(&Input, &Output, &Setpoint, 100,1,1,DIRECT);
 int encoder_prev = 0;

const uint8_t I_sum_size = 50;
double I_sum[I_sum_size];
uint8_t I_sum_head = 0;
 
 int16_t encoder_old = 0;
 int16_t encoder_original = 0;
/* END DC Motor Macros and Globals */

/*Servo Macros and Globals*/
Servo myservo; 
int pos = 0;
const int SERVO_PIN = 11;
/*END Servo Macros and Globals */

/*Stepper Macros and Globals */
const int STEPPER_PIN1 = 2; 
const int STEPPER_PIN2 = 3; 
uint16_t TOTAL_STEPS = 200;
uint8_t current_step = 0; 
uint8_t stepper_setpoint = 0;
Stepper myStepper(TOTAL_STEPS, STEPPER_PIN1, STEPPER_PIN2);


/*END Stepper Macros and Globals */
/* Mode Macros and Globals */

enum mode_type {MODE_DC_POT_SPEED, MODE_DC_POT_POS, MODE_STEP_HALL, MODE_SERVO_FSR, MODE_DC_GUI_SPEED, MODE_DC_GUI_POS, MODE_STEP_GUI, MODE_SERVO_GUI};
mode_type op_mode = MODE_DC_POT_SPEED;
uint8_t button_prev = HIGH;
/* END Mode Macros and Globals */

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); //Open serial port for computer
  //No hall setup
  fsr_setup();
  optical_setup();
  thermocouple_setup();
  ultrasonic_setup();
  dc_motor_setup();
  servo_setup();
}

void loop() {
  // put your main code here, to run repeatedly:
  uint16_t force_reading = 0;
  uint16_t hall_reading = 0;
  uint16_t optical_reading_red = 0;
  uint16_t optical_reading_blue = 0;
  uint16_t optical_reading_clear = 0;
  uint16_t optical_reading_green = 0;
  uint16_t temp_reading = 0;
  uint16_t ultrasonic_reading = 0;
  int16_t encoder_val = 0;
  int16_t motor_vel = 0;
  /*
  dc_encoder_loop();
  force_reading = fsr_loop(); 
  dc_encoder_loop(); 
  hall_reading = hall_loop();
  dc_encoder_loop();
  optical_reading_red = optical_loop_red();
  dc_encoder_loop();
  optical_reading_blue = optical_loop_blue();
  dc_encoder_loop();
  optical_reading_clear = optical_loop_clear();
  dc_encoder_loop();
  optical_reading_green = optical_loop_green();
  dc_encoder_loop();
  temp_reading = thermocouple_loop();
  //dc_encoder_loop();
  //ultrasonic_reading = ultrasonic_loop();
  */
  check_mode();
  update_setpoint();
  encoder_val = dc_encoder_loop();
  motor_vel = pid_loop(encoder_val);
  dc_encoder_loop();
  dc_motor_loop(motor_vel);
  dc_encoder_loop();
  servo_loop();
  dc_encoder_loop();
  update_stepper_setpoint();

  Serial.print(op_mode);
  Serial.write('\t');
  Serial.print(force_reading);
  Serial.write('\t');
  dc_encoder_loop();
  Serial.print(hall_reading);
  Serial.write('\t');
  dc_encoder_loop();
  Serial.print(optical_reading_red);
  Serial.write('\t');
  dc_encoder_loop();
  Serial.print(optical_reading_blue);
  Serial.write('\t');
  dc_encoder_loop();
  Serial.print(optical_reading_clear);
  Serial.write('\t');
  dc_encoder_loop();
  Serial.print(optical_reading_green);
  Serial.write('\t');
  dc_encoder_loop();
  Serial.print(temp_reading);
  Serial.write('\t');
  dc_encoder_loop();
  Serial.print(ultrasonic_reading);
  Serial.write('\t');
  dc_encoder_loop();
  Serial.print(encoder_val);
  Serial.write('\t');
  dc_encoder_loop();
  Serial.print(encoder_val - encoder_old);
  encoder_old = encoder_val;
  Serial.write('\t');
  dc_encoder_loop();
  Serial.print(motor_vel);
  Serial.write('\t');
  Serial.print((int)Setpoint);
  Serial.write('\n');
  for(int i = 0; i < 2000; i++){
    dc_encoder_loop();
  }
  /* Force Inductance Optical (4) Temp Ultrasonic Encoder Motor*/
}

/* Hall effect functions */

uint16_t hall_loop(){
  uint16_t raw_val = analogRead(hall_pin);
  raw_val += analogRead(hall_pin);
  raw_val += analogRead(hall_pin);
  raw_val += analogRead(hall_pin);
  raw_val += analogRead(hall_pin);
  raw_val += analogRead(hall_pin);
  raw_val += analogRead(hall_pin);
  raw_val += analogRead(hall_pin);
  float raw = ((float)raw_val)/8;
  float micrometers = 3.3238*(raw*raw)-1777.5*raw+244606;
  return (uint16_t)micrometers;
}

/* END Hall effect functions */

/* FSR functions */

void fsr_setup(){
  pinMode(sensorB, INPUT);
  pinMode(sensorD, INPUT);
}

uint16_t fsr_loop(){
  sensorB = analogRead(ptB);
  sensorD = analogRead(ptD);
  int16_t raw = sensorD - sensorB; 
  return (uint16_t)(2*raw+1174);
}

/* END FSR functions */

/* Optical functions */
void optical_setup(){
  pinMode(optical_S0_pin,OUTPUT);
  pinMode(optical_S1_pin,OUTPUT);
  pinMode(optical_S2_pin,OUTPUT);
  pinMode(optical_S3_pin,OUTPUT);
  digitalWrite(optical_S0_pin,HIGH);  //100% frequency scaling
  digitalWrite(optical_S1_pin,HIGH);
  FreqMeasure.begin();
}

uint16_t optical_loop_red(){
  digitalWrite(optical_S2_pin,LOW);
  digitalWrite(optical_S3_pin,LOW);
  return optical_get_reading();
}

uint16_t optical_loop_blue(){
  digitalWrite(optical_S2_pin,LOW);
  digitalWrite(optical_S3_pin,HIGH);
  return optical_get_reading();
}

uint16_t optical_loop_clear(){
  digitalWrite(optical_S2_pin,HIGH);
  digitalWrite(optical_S3_pin,LOW);
  return optical_get_reading();
}

uint16_t optical_loop_green(){
  digitalWrite(optical_S2_pin,HIGH);
  digitalWrite(optical_S3_pin,HIGH);
  return optical_get_reading();
}

uint16_t optical_get_reading(){
  while(!FreqMeasure.available());
  //average readings
  double sum = 0;
  for(int i = 0; i < 32; i++){
    sum = sum + FreqMeasure.read();
  }
  float frequency = FreqMeasure.countToFrequency(sum/32);
  return (uint16_t)frequency;
}

/* END Optical functions */

/* Thermocouple functions */

void thermocouple_setup(){
  pinMode(TEMP_IN, INPUT);
}

uint16_t thermocouple_loop(){
  sensor_value = (float)analogRead(TEMP_IN);
  sensor_value += (float)analogRead(TEMP_IN);
  sensor_value += (float)analogRead(TEMP_IN);
  temp = (sensor_value/3)*k + b;
  return (uint16_t)temp;
}

/* END Thermocouple functions */

/* Ultrasonic functions */

void ultrasonic_setup(){
  pinMode(TRIG_PIN,OUTPUT);
  pinMode(ECHO_PIN,INPUT);
}

uint16_t ultrasonic_loop(){
   long duration, distanceCm, distanceIn;
 
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN,HIGH);
 
  // convert the time into a distance
  distanceCm = duration / 29.1 / 2 ;
  distanceIn = duration / 74 / 2;
  return (uint16_t)distanceCm;
}

/* END Ultrasonic functions */

/* DC Motor functions */

void dc_motor_setup(){
   pinMode(encoder0PinA,INPUT);
   pinMode(encoder0PinB,INPUT);
   pinMode(dcmotor_I1,OUTPUT);
   pinMode(dcmotor_I2,OUTPUT);

  //PID setup
  Input = 0;  //Encoder count/speed is 0
  //myPID.SetMode(MANUAL);
  //myPID.SetTunings(100,1,1);//Kp, Ki, Kd
}

void servo_setup(){
  myservo.attach(SERVO_PIN);
  
}

// Code from http://playground.arduino.cc/Main/RotaryEncoders#Example1
int dc_encoder_loop() { 
   n = digitalRead(encoder0PinA);
   if ((encoder0PinALast == LOW) && (n == HIGH)) {
     if (digitalRead(encoder0PinB) == LOW) {
       encoder0Pos--;
     } else {
       encoder0Pos++;
     }
   } 
   encoder0PinALast = n;
   return encoder0Pos;
 } 

void dc_motor_loop(int16_t motor_velocity){
  if(motor_velocity > 0){
    analogWrite(dcmotor_I1,map(motor_velocity,0,255,66,255));
    digitalWrite(dcmotor_I2,LOW);
  } else if(motor_velocity < 0){
    analogWrite(dcmotor_I1,map(255+motor_velocity,0,255,66,255));
    digitalWrite(dcmotor_I2,HIGH); 
  }  else {
    analogWrite(dcmotor_I1,0);
    digitalWrite(dcmotor_I2,LOW);
  }
}

int pid_loop(int encoder_count){
  double error;
  int16_t motor_out = 0;
  double Kp = 0;
  double Ki = 0;
  if((op_mode == MODE_DC_POT_POS) || (op_mode == MODE_DC_GUI_POS)){
    error = Setpoint - (encoder_count - encoder_original);
    Kp = 10;
    Ki = 0;
  } else if((op_mode == MODE_DC_POT_SPEED) || (op_mode == MODE_DC_GUI_SPEED)){
    error = Setpoint - (encoder_count-encoder_prev);
    Kp = 10;
    Ki = 5;
  } else {
    motor_out = 0;
    return motor_out;
  }
  //double error = Setpoint-Input;
  Input = encoder_count - encoder_prev;
  encoder_prev = encoder_count;
  //myPID.Compute();
  I_sum[I_sum_head] = error;
  I_sum_head++;
  I_sum_head = I_sum_head % I_sum_size;
  double I_sum_total = 0;
  for(uint8_t i = 0; i < I_sum_size; i++){
    I_sum_total += I_sum[i];
  }
  Output = Kp*(error) + Ki*I_sum_total;  
  if(Output > 127){
    motor_out = 127;
  } else if(Output < -127){
    motor_out = -127;
  } else {
    motor_out = (int)Output;
  }
  return motor_out;
}

void update_setpoint(void){
  uint16_t pot_read = analogRead(A4);
  if(op_mode == MODE_DC_GUI_SPEED){
    Setpoint = 0; //TODO get from gui
  } else if(op_mode == MODE_DC_GUI_POS){
    Setpoint = 0; //TODO get from gui
  } else if(op_mode == MODE_DC_POT_SPEED){
    Setpoint = map(pot_read,0,1024,-5,5);
  } else if(op_mode == MODE_DC_POT_POS){
    Setpoint = map(pot_read,0,1024,-30,30);
  } else {
    Setpoint = 0;
  }
}

void clear_Isum(void){
  for(int i = 0; i < I_sum_size; i++){
    I_sum[i] = 0;
  }
}

/* END DC Motor functions */

/* Mode set functions */

void check_mode(void){
  uint8_t button_state = digitalRead(13);
  if((button_state == LOW) && (button_prev == HIGH)){  //Falling edge
    switch(op_mode){
      case MODE_DC_POT_SPEED:
        op_mode = MODE_DC_POT_POS;
        clear_Isum();
        break;
      case MODE_DC_POT_POS:
        op_mode = MODE_STEP_HALL;
        break;
      case MODE_STEP_HALL:
        op_mode = MODE_SERVO_FSR;
        break;
      case MODE_SERVO_FSR:
        op_mode = MODE_DC_GUI_SPEED;
        break;
      case MODE_DC_GUI_SPEED:
        op_mode = MODE_DC_GUI_POS;
        clear_Isum();
        break;
      case MODE_DC_GUI_POS:
        op_mode = MODE_STEP_GUI;
        clear_Isum();
        break;
      case MODE_STEP_GUI:
        op_mode = MODE_SERVO_GUI;
        break;
      case MODE_SERVO_GUI:
        op_mode = MODE_DC_POT_SPEED;
        clear_Isum();
        break;
      default:
        op_mode = MODE_DC_POT_SPEED;
        clear_Isum();
        break;
    }
  }
  button_prev = button_state;
  return;
}

/* END Mode set functions */

/* Servo functions */
void servo_loop(){
  uint16_t force_reading = 0;
  uint16_t force_threshold = 1;
  force_reading = fsr_loop();

  if(op_mode == MODE_SERVO_GUI || op_mode == MODE_SERVO_FSR){
    if(force_reading > force_threshold){
      myservo.write(180); 
    }
    else{
      myservo.write(0);
    }
  }
  else{
    myservo.write(0);
    return;
  }
}
/*END servo function */

/* Stepper functions */
void update_stepper_setpoint(void){
  if(op_mode == MODE_STEP_GUI || op_mode == MODE_STEP_HALL){
    stepper_setpoint = hall_loop();
  }
  else{
    stepper_setpoint = 0;
  }
}

void stepper_loop(){
  
  if(op_mode == MODE_STEP_GUI || op_mode == MODE_STEP_HALL){
      if(current_step < stepper_setpoint){ // increment one step
        myStepper.step(1); 
        current_step++;
      }
      else if(current_step > stepper_setpoint){ // decrement one step
        myStepper.step(-1);
        current_step--;
      }
  }
  return;
}

