void setup() {

  // Initialize Serial and Timer1 interrupt (only runs if the debug is active)

  debugbegin(38400);
  delay(500);
  Timer1.initialize(500000);
  delay(500);

  Timer1.attachInterrupt(passBlink);
  // ===============================
  // Initialize Notifiers
  debugln("Initializing Notifiers");
  debugln(" ");

  pinMode(SD_PIN_I, OUTPUT);
  pinMode(SD_PIN_II, OUTPUT);
  pinMode(IR_PIN, OUTPUT);

  delay(500);

  // ===============================
  // Initialize SD Card Breakout

  debugln("Initializing SD Card Breakout");
  debugln(" ");

  if (sizeof(block_t) != 512) {
    error("Invalid block size");
  }

  if (!sd.begin(SD_CS_PIN, SPI_FULL_SPEED)) {
    debugln("Cannot contact SD Card Breakout");
    FailBlink();
  }

  delay(500);

  // ===============================
  // Let's do a little check on how the hardware is doing

  debug(F("Available SRAM (bytes):"));
  debugln(FreeStack());

  digitalWrite(SD_PIN_I, LOW);
  Timer1.detachInterrupt();
  debugln("All Systems Good!!!");
  delay(1000);
}
