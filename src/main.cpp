#include <Arduino.h>

#define UP_KEY D1
#define DOWN_KEY D2


void setup() {
    Serial.begin(115200);
    
    pinMode(UP_KEY, OUTPUT);
    pinMode(DOWN_KEY, OUTPUT);

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(LED_BUILTIN_AUX, OUTPUT);
}

void loop() {
    if (!Serial.available()) {
        return;
    }

    String rawInput = Serial.readStringUntil('\n');
    
    if (rawInput == "u") {
        digitalWrite(UP_KEY, HIGH);
        digitalWrite(LED_BUILTIN, LOW);
        delay(200);
    } else if (rawInput == "d") {
        digitalWrite(DOWN_KEY, HIGH);
        digitalWrite(LED_BUILTIN_AUX, LOW);
        delay(200);
    }

    digitalWrite(UP_KEY, LOW);
    digitalWrite(DOWN_KEY, LOW);

    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_BUILTIN_AUX, HIGH);

    delay(10);
}