void setup() {
  // Initialize Serial and Timer1 interrupt (only runs if the debug is active)

  Serial.begin(9600);
  debug("$ \n");
  delay(500);

  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(23, INPUT);
  digitalWrite(0,HIGH);
  digitalWrite(1,LOW);

  // ===============================
  // Initialize Notifiers
  debugln("Initializing Notifiers");
  debugln(" ");

  pinMode(STATUS_LED, OUTPUT);

  SetupBlink(1);
  delay(500);

  // ===============================
  // Initialize Sensors

  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);

  // Read the WHO_AM_I registers, this is a good test of communication
  debugln("ITG3701+LSM303D 9-axis motion sensor...");
  byte c = readByte(ITG3701_ADDRESS, ITG3701_WHO_AM_I);  // Read WHO_AM_I register for ITG3701 gyro
  debugln("ITG3701 gyro"); debug("I AM "); debugt(c, HEX); debug(" I should be "); debugtln(0x68, HEX);
  byte d = readByte(LSM303D_ADDRESS, LSM303D_WHO_AM_I_XM);  // Read WHO_AM_I register for LSM303D accel/magnetometer
  debugln("LSM303D accel/magnetometer"); debug("I AM "); debugt(d, HEX); debug(" I should be "); debugtln(0x49, HEX);

  if (c == 0x68 && d == 0x49) // WHO_AM_I should always be 0xD4 for the gyro and 0x49 for the accel/mag
  {
    debugln("ITG3701+LSM303D is online...");

    initITG3701();
    debugln("ITG3701 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

    initLSM303D();
    debugln("LSM303D initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

    getAres();
    getGres();
    getMres();
    debug("accel sensitivity is "); debug(1. / (1000.*aRes)); debugln(" LSB/mg");
    debug("gyro sensitivity is "); debug(1. / (gRes)); debugln(" LSB/dps");
    debug("mag sensitivity is "); debug(1. / (1000.*mRes)); debugln(" LSB/mGauss");

    // Calibrate gyro and accelerometers, load biases in bias registers
    if (sens_calib == false) {
      gyroBias[0] = 0.96;
      gyroBias[1] = -1.66;
      gyroBias[2] = 1.43;

      accelBias[0] = -47.85;
      accelBias[1] = 10.74;
      accelBias[2] = 73.30;

      magBias[0] = 0.12;
      magBias[1] = -0.18;
      magBias[2] = 0.12;
    }
    else if (sens_calib == true) {
      gyrocalITG3701(gyroBias);
      accelcalLSM303D(accelBias);
      debugln("accel biases (mg)"); debugln(1000.*accelBias[0]); debugln(1000.*accelBias[1]); debugln(1000.*accelBias[2]);
      debugln("gyro biases (dps)"); debugln(gyroBias[0]); debugln(gyroBias[1]); debugln(gyroBias[2]);

      magcalLSM303D(magBias);
      debugln("mag biases (mG)"); debugln(1000.*magBias[0]); debugln(1000.*magBias[1]); debugln(1000.*magBias[2]);
    }

    // Reset the MS5637 pressure sensor
    MS5637Reset();
    delay(100);
    debugln("MS5637 pressure sensor reset...");
    // Read PROM data from MS5637 pressure sensor
    MS5637PromRead(Pcal);
    debugln("PROM data read:");
    debug("C0 = "); debugln(Pcal[0]);
    unsigned char refCRC = Pcal[0]  >> 12;
    debug("C1 = "); debugln(Pcal[1]);
    debug("C2 = "); debugln(Pcal[2]);
    debug("C3 = "); debugln(Pcal[3]);
    debug("C4 = "); debugln(Pcal[4]);
    debug("C5 = "); debugln(Pcal[5]);
    debug("C6 = "); debugln(Pcal[6]);
    debug("C7 = "); debugln(Pcal[7]);

    nCRC = MS5637checkCRC(Pcal);  //calculate checksum to ensure integrity of MS5637 calibration data
    debug("Checksum = "); debug(nCRC); debug(" , should be "); debugln(refCRC);

  }
  else
  {
    debug("Could not connect to ITG3701+LSM303D: 0x");
    debugtln(c, HEX);
    while (1) ; // Loop forever if communication doesn't happen
  }

  SetupBlink(4);
  delay(500);
  
  // ===============================

  Serial.print(F("FreeStack: "));
  Serial.println(FreeStack());
  Serial.print(F("Records/block: "));
  Serial.println(DATA_DIM);
  if (sizeof(block_t) != 512) {
    error("Invalid block size");
  }
  // initialize file system.
  if (!sd.begin(SD_CS_PIN, SPI_FULL_SPEED)) {
    sd.initErrorPrint();
    fatalBlink();
  }

  // ===============================

  debugln("All Systems Good!!!");
  debug("% \n");
  delay(1000);
}
