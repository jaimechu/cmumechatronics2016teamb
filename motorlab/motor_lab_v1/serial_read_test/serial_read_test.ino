//int incomingByte = 0;   // for incoming serial data
//char inData[24]; 
void setup() {
    Serial.begin(115200);     // opens serial port, sets data rate to 9600 bps
}

void loop() {
 /* int16_t read_val = 0;
  int val = 0;
  int count = 0; 
  while(Serial.available() > 0) {
    val = Serial.read(); 
    inData[count] = val;
    if(val == 0 && count>=2) { // if third byte is a new line 
      read_val = inData[count-1] << 8; 
      read_val |= inData[count-2];

      Serial.print(read_val);
    }
    count++;
  }*/
  uint8_t test, test2; 
  test = 0xFE;
  test2 = 0xFF;
  Serial.print((uint16_t)test<<8);
  Serial.print('\t');
  Serial.println((int16_t)(test)<<8|test2);
}
