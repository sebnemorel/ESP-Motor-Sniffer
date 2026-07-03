#include <Arduino.h>
#include "CanSniffer.h"

CanSniffer sniffer;

void setup() {
    Serial.begin(115200);
    delay(1000);
    sniffer.begin();
}

void loop() {
    sniffer.handle();
}
