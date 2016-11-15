// ====================================================================================================
// FC Transmission ====================================================================================

void sorter (const byte c) {
  char kappa = c;
  if (kappa == 'A') {
    binaryFloat lol;
    for (int j = 0; j < i + 1; j ++) {
      lol.binary[j] = buff[j];
    }
    Serial.println(lol.floatingPoint);
    i = 0;
    Serial1.flush();
    //delay(1000);
  }
  else {
    buff[i] = c;
    i += 1;
  //  Serial.println(i);
    //   Serial.println(buff[i],BIN);

  }
}


