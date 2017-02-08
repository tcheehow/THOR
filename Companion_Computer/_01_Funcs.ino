/* Clears previous recording settings and sets up .csv file, giving it a file name and marking
   the top row with a header. Takes a 4 byte data packet where the first bytes informs the CC
   of file setup request. Remaining bytes configure a maximum of 24 data items that can be
   recorded. See header list for variables. */

void file_setup(byte info_packet[]) {
  header = "";
  data_tally = 0;

  uint8_t address;
  const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
  char fileName[13] = FILE_BASE_NAME "00.csv";

  if (!sd.begin()) {
    sd.initErrorHalt();
  }

  if (BASE_NAME_SIZE > 6) {
    error("FILE_BASE_NAME too long");
  }

  while (sd.exists(fileName)) {
    if (fileName[BASE_NAME_SIZE + 1] != '9') {
      fileName[BASE_NAME_SIZE + 1]++;
    }
    else if (fileName[BASE_NAME_SIZE] != '9') {
      fileName[BASE_NAME_SIZE + 1] = '0';
      fileName[BASE_NAME_SIZE]++;
    }
    else {
      error("Can't create file name");
    }
  }

  if (!file.open(fileName, O_CREAT | O_WRITE | O_EXCL)) {
    error("file.open");
  }

  for (uint8_t i = 1; i < 4; i++) {
    for (int8_t j = 7; j >= 0; j--) {
      if (bitRead(info_packet[i], j) == 1) { // remember bitRead reads right to left
        address = ((i - 1) * 8) + (7 - j);
        header += data_pool[address];
        header += ",";
        data_tally += 1;
      }
    }
  }

  file.print((header));
  file.println();

  debug("Data setup complete. Outputting: ");
  debug(header);
}

/* comm_manager handles all FC to CC communications and decides what to do with said data. It will save a data log
   whenever the FC powered down and will call for a new log when FC is powered up. While recording, it
   dumps the data in the logging buffer.

   In the future, a way to save logs at intervals will be implemented in the event of a CC failure/log
   overflow mid experiment. Also, the comm_manager will have the power to determine and log certain errors.The
   comm_manager will also be linked to a WiFi chip for additional duties (like calling for a change in tuning parameter).
*/
void comm_manager() {

  uint8_t inIndex = 0;

  if ((!Serial1.available()) && (record == true)) {           // Signal lost. Save and close current file.
    debugln("FC signal lost. Saving logs");
    record = false;
    file.close();
  }
  
  while (Serial1.available()) {                               // Receiving something from FC.
    inData[inIndex] = Serial1.read();
    inIndex += 1;
  }

  if ((inData[0] == B11111111) && (record == false)) {        // Reset key detected. Call the file setup guy.
    file_setup(inData);
    record = true;
  }
  else if ((inData[0] != B11111111) && (record == true)) {    // If it isn't a reset key, then it has to be a data log!
    digitalWrite(LED_PIN, HIGH);
    uint8_t count;
    binaryFloat letterbox;

    for (uint8_t i = 0; i < data_tally; i ++) {                 // Decipher the data and dump it into buffer. Tell charon.
      for (uint8_t j = 0; j < 4; j ++) {
        count = (i * 4) + j;
        letterbox.binary[j] = inData[count];
      }
      buff[charon] = letterbox.floatingPoint;
      charon += 1;
    }
  }

  else if ((inData[0] == B11111111) && (record == true)) {      // Receiving header but I was recording prior.
    debugln("Missed an FC reset (was still recording). Logging error and continuing record.");
    file.write("Missed an FC reset (was still recording). Logging error and continuing record.");
    file.println();
    digitalWrite(LED_PIN, LOW);
  }
  else {                                                        // Receiving item data, but I have no header (file already closed).
    digitalWrite(LED_PIN, LOW);
  }
}

/* Takes the buffer data at intervals and writes it onto the SD card. */
void logData() {
  uint8_t checker;

  for (uint16_t i = 0; i < charon; i++) {
    file.print(buff[i]);
    checker = (i + 1) % data_tally;

    if (checker == 0) {
      file.println();
    }
    else {
      file.write(",");
    }
  }
}

