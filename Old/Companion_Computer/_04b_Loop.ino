RecordingSolid();

while (1) {
  //==========================================
  bool dataline = false;
  
  // Time for next data record.
  logTime += LOG_INTERVAL_USEC;

  if (Serial.available()) {
    
  }
  
  pool[0] = 111;
  pool[1] = 222;
  pool[2] = 333;

  //==========================================

  if (closeFile) {
    if (curBlock != 0) {
      // Put buffer in full queue.
      fullQueue[fullHead] = curBlock;
      fullHead = queueNext(fullHead);
      curBlock = 0;
    }
  }
  else {
    if (curBlock == 0 && emptyTail != emptyHead) {
      curBlock = emptyQueue[emptyTail];
      emptyTail = queueNext(emptyTail);
      curBlock->count = 0;
      curBlock->overrun = overrun;
      overrun = 0;
    }
    if ((int32_t)(logTime - micros()) < 0) {
      error("Rate too fast");
    }
    int32_t delta;
    do {
      delta = micros() - logTime;
    }
    while (delta < 0);

    if (curBlock == 0) {
      overrun++;
    }
    else {
      if (useSharedSpi) {
        sd.card()->chipSelectHigh();
      }
      acquireData(&curBlock->data[curBlock->count++], pool);
      if (useSharedSpi) {
        sd.card()->chipSelectLow();
      }
      if (curBlock->count == DATA_DIM) {
        fullQueue[fullHead] = curBlock;
        fullHead = queueNext(fullHead);
        curBlock = 0;
      }
      if ((uint32_t)delta > maxDelta) maxDelta = delta;
      if ((uint32_t)delta < minDelta) minDelta = delta;
    }
  }

  if (fullHead == fullTail) {
    // Exit loop if done.
    if (closeFile) {
      break;
    }
  } else if (!sd.card()->isBusy()) {
    // Get address of block to write.
    block_t* pBlock = fullQueue[fullTail];
    fullTail = queueNext(fullTail);
    // Write block to SD.
    uint32_t usec = micros();
    if (!sd.card()->writeData((uint8_t*)pBlock)) {
      error("write data failed");
    }
    usec = micros() - usec;
    t1 = millis();
    if (usec > maxLatency) {
      maxLatency = usec;
    }
    count += pBlock->count;

    // Add overruns and possibly light LED.
    if (pBlock->overrun) {
      overrunTotal += pBlock->overrun;
      if (ERROR_LED_PIN >= 0) {
        digitalWrite(ERROR_LED_PIN, HIGH);
      }
    }
    // Move block to empty queue.
    emptyQueue[emptyHead] = pBlock;
    emptyHead = queueNext(emptyHead);
    bn++;
    if (bn == FILE_BLOCK_COUNT) {
      // File full so stop
      break;
    }
  }
}

