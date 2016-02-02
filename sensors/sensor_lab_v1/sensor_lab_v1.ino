/* Team TOM (B)
 * Sensor Lab v1
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
 * Version history:
 * 2/1/16: Nishant Pol: Initial release, combinaion of ldc1000, fsr, and matlab interface arduino code
 */

/* Hall Sensor Macros and Globals */

const uint8_t hall_pin = A2;

/* END Hall Sensor Macros and Globals */

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //Open serial port for computer
  
}

void loop() {
  // put your main code here, to run repeatedly:
  uint16_t force_reading;
  uint16_t hall_reading;
  uint16_t optical_reading;
  uint16_t temp_reading;
  uint16_t ultrasonic_reading;
  force_reading = 0;  //Todo
  hall_reading = hall_loop();
  optical_reading = 0;
  temp_reading = 0;
  ultrasonic_reading = 0;
  Serial.print(force_reading);
  Serial.write('\t');
  Serial.print(hall_reading);
  Serial.write('\t');
  Serial.print(optical_reading);
  Serial.write('\t');
  Serial.print(temp_reading);
  Serial.write('\t');
  Serial.print(ultrasonic_reading);
  Serial.write('\n');
  delay(100);
  /* Force Inductance Optical Temp Ultrasonic */
}

/* Hall effect functions */

uint16_t hall_loop(){
  return analogRead(hall_pin);
}

/* END Hall effect functions */

