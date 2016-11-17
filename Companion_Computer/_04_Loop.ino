void loop() {
  // Time for next record.
  logTime += 1000UL * SAMPLE_INTERVAL_MS;

  // Wait for log time.
  int32_t diff;
  do {
    diff = micros() - logTime;
  } while (diff < 0);

  // Check for data rate too high.
  if (diff > 10) {
    error("Missed data record");
  }

  // Grab that data ===========================
  int trigger = digitalRead(2);
  
  if (trigger == HIGH) {
    comm_supervisor();
    sorter();

    memset(inData, 0, sizeof(inData));
  }
  else {
    // Serial.println("kappa");
  }
  
  logData();

  // Force data to SD and update the directory entry to avoid data loss.
  if (!file.sync() || file.getWriteError()) {
    error("write error");
  }

  if (Serial.available()) {
    // Close file and stop.
    file.close();
    Serial.println(F("Done"));
    SysCall::halt();
  }
}
