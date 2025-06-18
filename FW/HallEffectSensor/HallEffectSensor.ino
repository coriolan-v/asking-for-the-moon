
const int sensorPin = A6;   // analog input
bool homePos = false;

unsigned long lastReadTime = 0;
const unsigned long readInterval = 50; // ms

void setup() {
  pinMode(sensorPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastReadTime >= readInterval) {
    lastReadTime = currentTime;

    int sensorValue = analogRead(sensorPin);

    Serial.println(sensorValue);

    homePos = (sensorValue >= 700);

    if (homePos) {
      Serial.println("HOME");
    } else {
      Serial.println(sensorValue);
    }
  }
}
