#define PIN_LED 7

void setup() {
  pinMode (PIN_LED, OUTPUT);
}

void loop() {
  digitalWrite(PIN_LED, 0);
  delay(1000);
  for (int i=1; i<6; i++)
  {
    digitalWrite(PIN_LED, 1);
    delay(100);
    digitalWrite(PIN_LED, 0);
    delay(100);
  }
  digitalWrite(PIN_LED, 1);
  while(1){}
}
