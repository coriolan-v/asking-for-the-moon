#include <bluefruit.h>

BLEUart bleuart;

void setup() {
  Serial.begin(115200);
  delay(500);

  // Start BLE + advertise UART as "DFU mode"
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("MyBoard-DFU");
  bleuart.begin();

  // Add service + start advertising
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.Advertising.start(0);  // 0 = no timeout

  Serial.println("BLE DFU window open (10s)...");
  delay(10000); // DFU wait window

  // Stop advertising & BLE
  Bluefruit.Advertising.stop();
  Bluefruit.disconnect();
  Bluefruit.stop();

  Serial.println("Continuing to main app");
}

void loop() {
  // Your main code starts here
  Serial.println("Running main app logic...");
  delay(1000);
}
