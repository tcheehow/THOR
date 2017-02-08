void loop() {

  if (digitalRead(TRIGGER_PIN) == LOW) {
    comm_manager();
  }

  /* Once charon nears the buff limit, write to SD card. The reason for the (-data_tally) is because
     charon can be seen to add in increments of the data_tally. To avoid a buffer overflow, check if
     charon is less than data_tally away from BUFF_size
  */
  if (record == true) {
    if (charon > (BUFF_SIZE - data_tally)) {
      logData();
      if (!file.sync() || file.getWriteError()) {
        error("write error");
      }
      charon = 0;
    }
  }

}
