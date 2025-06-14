#include <Arduino.h>

#define ENC1 14
#define ENC2 12
#define CONTROL_PIN 10

void setup() {
  Serial.begin(115200);
  
  pinMode(CONTROL_PIN, OUTPUT);
  digitalWrite(CONTROL_PIN, HIGH);

  pinMode(ENC1, OUTPUT);
  pinMode(ENC2, OUTPUT);

  digitalWrite(ENC1, HIGH);
  digitalWrite(ENC2, HIGH);

  // Serial.println("Type 'o' to toggle LIGHT");
}

bool toggled = false;

void loop() {
  if (Serial.available() > 0) {
    char input = Serial.read();

    if (input == 'o') {
      digitalWrite(CONTROL_PIN, LOW);
      delay(200);
      digitalWrite(CONTROL_PIN, HIGH);
    
    } else if (input == 't') {
      toggled = !toggled;
      digitalWrite(CONTROL_PIN, toggled);

    } else if (input == 'd') {
      digitalWrite(ENC1, HIGH);
      digitalWrite(ENC2, LOW);
      delay(10);
      digitalWrite(ENC1, LOW);
    
    } else if (input == 'u') {
      digitalWrite(ENC1, LOW);
      digitalWrite(ENC2, HIGH);
      delay(10);
      digitalWrite(ENC2, LOW);
    
    } else if (input == 'r') {
      digitalWrite(ENC1, HIGH);
      digitalWrite(ENC2, HIGH);
    }
  }
}
