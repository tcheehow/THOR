//===================================================================================================================
//====== SD Card Breakout Functions
//===================================================================================================================

void binaryToCsv() {
  uint8_t lastPct = 0;
  block_t block;
  uint32_t t0 = millis();
  uint32_t syncCluster = 0;
  SdFile csvFile;
  char csvName[13];

  if (!binFile.isOpen()) {
    debugln();
    debugln(F("No current binary file"));
    return;
  }
  binFile.rewind();
  // Create a new csvFile.
  strcpy(csvName, binName);
  strcpy(&csvName[BASE_NAME_SIZE + 3], "csv");

  if (!csvFile.open(csvName, O_WRITE | O_CREAT | O_TRUNC)) {
    error("open csvFile failed");
  }
  debugln();
  debug(F("Writing: "));
  debug(csvName);
  debugln(F(" - type any character to stop"));
  printHeader(&csvFile);
  uint32_t tPct = millis();
  while (!Serial.available() && binFile.read(&block, 512) == 512) {
    uint16_t i;
    if (block.count == 0) {
      break;
    }
    if (block.overrun) {
      csvFile.print(F("OVERRUN,"));
      csvFile.println(block.overrun);
    }
    for (i = 0; i < block.count; i++) {
      printData(&csvFile, &block.data[i]);
    }
    if (csvFile.curCluster() != syncCluster) {
      csvFile.sync();
      syncCluster = csvFile.curCluster();
    }
    if ((millis() - tPct) > 1000) {
      uint8_t pct = binFile.curPosition() / (binFile.fileSize() / 100);
      if (pct != lastPct) {
        tPct = millis();
        lastPct = pct;
        debug(pct);
        debugln('%');
      }
    }
    if (Serial.available()) {
      break;
    }
  }
  csvFile.close();
  debug(F("Done: "));
  debug(0.001 * (millis() - t0));
  debugln(F(" Seconds"));
}
//------------------------------------------------------------------------------
// read data file and check for overruns
void checkOverrun() {
  bool headerPrinted = false;
  block_t block;
  uint32_t bgnBlock, endBlock;
  uint32_t bn = 0;

  if (!binFile.isOpen()) {
    debugln();
    debugln(F("No current binary file"));
    return;
  }
  if (!binFile.contiguousRange(&bgnBlock, &endBlock)) {
    error("contiguousRange failed");
  }
  binFile.rewind();
  debugln();
  debugln(F("Checking overrun errors - type any character to stop"));
  while (binFile.read(&block, 512) == 512) {
    if (block.count == 0) {
      break;
    }
    if (block.overrun) {
      if (!headerPrinted) {
        debugln();
        debugln(F("Overruns:"));
        debugln(F("fileBlockNumber,sdBlockNumber,overrunCount"));
        headerPrinted = true;
      }
      debug(bn);
      debug(',');
      debug(bgnBlock + bn);
      debug(',');
      debugln(block.overrun);
    }
    bn++;
  }
  if (!headerPrinted) {
    debugln(F("No errors found"));
  } else {
    debugln(F("Done"));
  }
}
//------------------------------------------------------------------------------
// dump data file to Serial
void dumpData() {
  block_t block;
  if (!binFile.isOpen()) {
    debugln();
    debugln(F("No current binary file"));
    return;
  }
  binFile.rewind();
  debugln();
  debugln(F("Type any character to stop"));
  delay(1000);
  printHeader(&Serial);
  while (!Serial.available() && binFile.read(&block , 512) == 512) {
    if (block.count == 0) {
      break;
    }
    if (block.overrun) {
      debug(F("OVERRUN,"));
      debugln(block.overrun);
    }
    for (uint16_t i = 0; i < block.count; i++) {
      printData(&Serial, &block.data[i]);
    }
  }
  debugln(F("Done"));
}
//------------------------------------------------------------------------------
// log data
// max number of blocks to erase per erase call
uint32_t const ERASE_SIZE = 262144L;
