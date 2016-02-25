const int TEMP_IN = A5; 
int sensor_value;
float temp;
float k = 0.8227; 
float b = 76.971;

void setup() {
  Serial.begin(9600);
  pinMode(TEMP_IN, INPUT);

}

void loop() {
  sensor_value = (float)analogRead(TEMP_IN);
  temp = sensor_value*k + b;
  Serial.print(sensor_value);
  Serial.print(" \t");
  Serial.println(temp);
  delay(300);
}
