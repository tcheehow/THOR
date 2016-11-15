//===================================================================================================================
//====== SD Logger Functions
//===================================================================================================================

// Write data header.
void writeHeader() {
  file.print(F("micros"));
  for (uint8_t i = 0; i < DATA_COUNT; i++) {
    file.print(F(",adc"));
    file.print(i, DEC);
  }
  file.println();
}

// Log a data record.
void logData() {
  uint16_t data[DATA_COUNT];

  // Read all channels to avoid SD write latency between readings.
  for (uint8_t i = 0; i < DATA_COUNT; i++) {
    data[i] = analogRead(i);
  }
  // Write data to file.  Start with log time in micros.
  file.print(logTime);

  // Write ADC data to CSV record.
  for (uint8_t i = 0; i < DATA_COUNT; i++) {
    file.write(',');
    file.print(data[i]);
  }
  file.println();
}
