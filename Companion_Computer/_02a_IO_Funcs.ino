// ====================================================================================================
// FC Transmission ====================================================================================

void sorter() {
  for (uint8_t i = 0; i < items; i += 1) {
    header = header_list[i];
    binaryFloat header;
    for (uint8_t j = 0; j < 4; j += 1) {
      uint8_t count = (i * 4) + j;
      header.binary[j] = inData[count];
      datar[i] = header.floatingPoint;
    }
  }
}

void comm_supervisor() {
  inIndex = 0;
  while (Serial1.available ()) {
    inData[inIndex] = Serial1.read();
    inIndex += 1;
  }
}


