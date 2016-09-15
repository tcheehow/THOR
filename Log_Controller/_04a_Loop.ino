void loop()
{

//  Serial.print(F("mag: ")); Serial.print(F("\t")); Serial.print(mx); Serial.print(F("\t")); Serial.print(my); Serial.print(F("\t")); Serial.println(mz);
//  Serial.print(F("acc: ")); Serial.print(F("\t")); Serial.print(ax); Serial.print(F("\t")); Serial.print(ay); Serial.print(F("\t")); Serial.println(az);
//  Serial.print(F("gyr: ")); Serial.print(F("\t")); Serial.print(gx); Serial.print(F("\t")); Serial.print(gy); Serial.print(F("\t")); Serial.println(gz);

    // Read any Serial data.
//  do {
//    delay(10);
//  } while (Serial.available() && Serial.read() >= 0);
//  Serial.println();
//  Serial.println(F("type:"));
//  Serial.println(F("c - convert file to csv"));
//  Serial.println(F("d - dump data to Serial"));
//  Serial.println(F("e - overrun error details"));
//  Serial.println(F("r - record data"));

//  while (!Serial.available()) {
//    SysCall::yield();
//  }
//  char c = tolower(Serial.read());
//
//  // Discard extra Serial data.
//  do {
//    delay(10);
//  } while (Serial.available() && Serial.read() >= 0);

  if (ERROR_LED_PIN >= 0) {
    digitalWrite(ERROR_LED_PIN, LOW);
  }

  logData();
  binaryToCsv();

  while(digitalRead(23)!=HIGH);
  
//  if (c == 'c') {
//    binaryToCsv();
//  } else if (c == 'd') {
//    dumpData();
//  } else if (c == 'e') {
//    checkOverrun();
//  } else if (c == 'r') {
//    logData();
//  } else {
//    Serial.println(F("Invalid entry"));
//  }
  
  // End of Flight Controller Section ============================
}
