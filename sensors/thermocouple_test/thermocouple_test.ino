const int TEMP_IN = A2; 
int sensor_value, temp;
int k = 1; 
int b = 1;

void setup() {
  Serial.begin(9600);
  pinMode(TEMP_IN, INPUT);

}

void loop() {
  sensor_value = analogRead(TEMP_IN);
  temp = sensor_value*k + b;
  Serial.print(sensor_value);
  Serial.print(" \t");
  Serial.print(temp);
  delay(300);
}
