/*Tested with wheatstone bridge, 2.2k ohms, placed on 
left most pin and right most pin as B and D */ 


const int ptD = A0; // leftmost side
const int ptB = A1; // right most sides
int sensorValue = 0; 

int sensorB, sensorD = 10;

void setup() {
  Serial.begin(9600);
  pinMode(sensorB, INPUT);
  pinMode(sensorD, INPUT);
}

void loop(){
  sensorB = analogRead(ptB);
  sensorD = analogRead(ptD);
  sensorValue = sensorD - sensorB; 
  Serial.print("sensor = ");
  Serial.println(sensorValue);
  Serial.print(sensorB);
  Serial.print(" \t");
  Serial.println(sensorD);
  delay(300);
}

