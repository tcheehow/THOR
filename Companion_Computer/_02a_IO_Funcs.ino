void passBlink() {
  if (ledState == LOW) {
    ledState = HIGH;
  } else {
    ledState = LOW;
  }
  digitalWrite(SD_PIN_I, ledState);
}

void calSolid() {
  digitalWrite(SD_PIN_I, HIGH);
  delay(3000);
  digitalWrite(SD_PIN_I, LOW);
}

void FailBlink() {
  while (true) {
    digitalWrite(SD_PIN_II, LOW);
    digitalWrite(SD_PIN_I, HIGH);
    delay(200);
    digitalWrite(SD_PIN_I, LOW);
    digitalWrite(SD_PIN_II, HIGH);
    delay(200);
  }
}

void RecordingSolid() {
  digitalWrite(SD_PIN_II, HIGH);
}

void WritingBlink() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(SD_PIN_II, HIGH);
    delay(200);
    digitalWrite(SD_PIN_II, LOW);
    delay(200);
  }
  delay(1000);
}


  //==============================================================================
  // Error messages stored in flash.
#define error(msg) errorFlash(F(msg))
  //------------------------------------------------------------------------------
  void errorFlash(const __FlashStringHelper * msg) {
    sd.errorPrint(msg);
    FailBlink();
  }
  //------------------------------------------------------------------------------
  //

