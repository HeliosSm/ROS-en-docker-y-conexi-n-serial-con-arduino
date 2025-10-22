// Arduino: Lee potenciómetro en A0 y envía por Serial

const int sensorPin = 32;
int sensorValue = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  sensorValue = analogRead(sensorPin);
  Serial.println(sensorValue);
  delay(1000); // cada segundo
}