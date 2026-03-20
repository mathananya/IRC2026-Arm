void setup() {
  pinMode(27,INPUT);
  pinMode(12,INPUT);
  pinMode(32,INPUT);
  pinMode(25,INPUT);

  Serial.begin(9600);
}

void loop() {
  Serial.print("FL: ");
  Serial.print(analogRead(32));
  Serial.print("| FR: ");
  Serial.print(analogRead(25));
  Serial.print("| BR: ");
  Serial.print(analogRead(27));
  Serial.print("| BL: ");
  Serial.println(analogRead(12));
}