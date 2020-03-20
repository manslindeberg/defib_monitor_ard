
#include "WiFiNINA.h"
//TEST2



WiFiServer server(80);

void setup() {
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

}

void loop() {
  // put your main code here, to run repeatedly:

}
