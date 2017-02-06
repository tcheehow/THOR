void loop() {

  if (digitalRead(TRIGGER_PIN) == HIGH) {
    comm_manager();
  }

  /* Check for interval (will change to array size check instead of time. Might be able to simplify
   * some if statements in the functions tab. */
  if (record == true) {
    if (millis() - logTime > REC_INTERVAL_MS) {

      logData();

      if (!file.sync() || file.getWriteError()) {
        error("write error");
      }

      logTime = millis();
      charon = 0;
    }
  }

  memset(inData, 0, sizeof(inData));
}
