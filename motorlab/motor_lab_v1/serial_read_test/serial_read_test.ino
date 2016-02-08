int incomingByte = 0;   // for incoming serial data
char inData[24]; 
void setup() {
    Serial.begin(115200);     // opens serial port, sets data rate to 9600 bps
}

void loop() {
  int read_val = 0;
    // send data only when you receive data:
    if (Serial.available() > 0) {
           // read the incoming byte:
           Serial.readBytes(inData, 3);
           if(inData[2] == 0){
             Serial.print("I received: ");
             read_val = inData[0] << 8; 
             read_val += inData[1]; 
             Serial.println(read_val, DEC);
           }
           else 
            Serial.println("NO");
  
    }
}
