void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t num1 = 65;
  uint8_t num2 = 66;
  uint8_t num3 = 67;
  Serial.print(num1);
  Serial.write('\t');
  Serial.print(num2);
  Serial.write('\t');
  Serial.print(num3);
  Serial.write('\n');
  delay(100);
}
