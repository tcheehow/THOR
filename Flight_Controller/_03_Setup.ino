
void setup() {

  //delay(100);     // Wait just a bit so that the CC is ready first.

  // Initialize Debugger
  debugbegin(38400);
  debugln("Debugger Initialized");
  delay(100);

  // Initialize RC
  receiver.begin(RC_PIN);
  debugln("RC Initialized");
  delay(100);

  // Initialize Notifiers
  pinMode(SYNC_LED_PIN, OUTPUT);
  pinMode(2, OUTPUT);
  debugln("LED Initialized");
  delay(100);

  // Initialize Serial1 Bridge to CC
  Serial1.begin(115200);
  debugln("CC Bridge Initialized");
  delay(100);

  // Inform the CC who you are and what you're sending.
  digitalWrite(2, LOW);
  Serial1.write(fc_info, 4);
  digitalWrite(2, HIGH);
  debugln("CC Packet Setup Sent");

  // Setup for Master mode, pins 18/19, external pullups, 400kHz for Teensy 3.1
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
  delay(5000);

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, LOW);

  I2Cscan(); // should detect SENtral at 0x28

  // Read SENtral device information
  uint16_t ROM1 = readByte(EM7180_ADDRESS, EM7180_ROMVersion1);
  uint16_t ROM2 = readByte(EM7180_ADDRESS, EM7180_ROMVersion2);
  Serial.print("EM7180 ROM Version: 0x"); Serial.print(ROM1, HEX); Serial.println(ROM2, HEX); Serial.println("Should be: 0xE609");
  uint16_t RAM1 = readByte(EM7180_ADDRESS, EM7180_RAMVersion1);
  uint16_t RAM2 = readByte(EM7180_ADDRESS, EM7180_RAMVersion2);
  Serial.print("EM7180 RAM Version: 0x"); Serial.print(RAM1); Serial.println(RAM2);
  uint8_t PID = readByte(EM7180_ADDRESS, EM7180_ProductID);
  Serial.print("EM7180 ProductID: 0x"); Serial.print(PID, HEX); Serial.println(" Should be: 0x80");
  uint8_t RID = readByte(EM7180_ADDRESS, EM7180_RevisionID);
  Serial.print("EM7180 RevisionID: 0x"); Serial.print(RID, HEX); Serial.println(" Should be: 0x02");

  delay(1000); // give some time to read the screen

  // Check which sensors can be detected by the EM7180
  uint8_t featureflag = readByte(EM7180_ADDRESS, EM7180_FeatureFlags);
  if (featureflag & 0x01)  Serial.println("A barometer is installed");
  if (featureflag & 0x02)  Serial.println("A humidity sensor is installed");
  if (featureflag & 0x04)  Serial.println("A temperature sensor is installed");
  if (featureflag & 0x08)  Serial.println("A custom sensor is installed");
  if (featureflag & 0x10)  Serial.println("A second custom sensor is installed");
  if (featureflag & 0x20)  Serial.println("A third custom sensor is installed");

  delay(1000); // give some time to read the screen

  // Check SENtral status, make sure EEPROM upload of firmware was accomplished
  byte STAT = (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01);
  if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01)  Serial.println("EEPROM detected on the sensor bus!");
  if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x02)  Serial.println("EEPROM uploaded config file!");
  if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04)  Serial.println("EEPROM CRC incorrect!");
  if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x08)  Serial.println("EM7180 in initialized state!");
  if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x10)  Serial.println("No EEPROM detected!");
  int count = 0;
  while (!STAT) {
    writeByte(EM7180_ADDRESS, EM7180_ResetRequest, 0x01);
    delay(500);
    count++;
    STAT = (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01);
    if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x01)  Serial.println("EEPROM detected on the sensor bus!");
    if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x02)  Serial.println("EEPROM uploaded config file!");
    if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04)  Serial.println("EEPROM CRC incorrect!");
    if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x08)  Serial.println("EM7180 in initialized state!");
    if (readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x10)  Serial.println("No EEPROM detected!");
    if (count > 10) break;
  }

  if (!(readByte(EM7180_ADDRESS, EM7180_SentralStatus) & 0x04))  Serial.println("EEPROM upload successful!");
  delay(1000); // give some time to read the screen

  // Set up the SENtral as sensor bus in normal operating mode
  if (!passThru) {
    // Enter EM7180 initialized state
    writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x00); // set SENtral in initialized state to configure registers
    writeByte(EM7180_ADDRESS, EM7180_PassThruControl, 0x00); // make sure pass through mode is off
    writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x01); // Force initialize
    writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x00); // set SENtral in initialized state to configure registers

    //Setup LPF bandwidth (BEFORE setting ODR's)
    writeByte(EM7180_ADDRESS, EM7180_ACC_LPF_BW, 0x03); // 41Hz
    writeByte(EM7180_ADDRESS, EM7180_GYRO_LPF_BW, 0x03); // 41Hz
    // Set accel/gyro/mage desired ODR rates
    writeByte(EM7180_ADDRESS, EM7180_QRateDivisor, 0x02); // 100 Hz
    writeByte(EM7180_ADDRESS, EM7180_MagRate, 0x64); // 100 Hz
    writeByte(EM7180_ADDRESS, EM7180_AccelRate, 0x14); // 200/10 Hz
    writeByte(EM7180_ADDRESS, EM7180_GyroRate, 0x14); // 200/10 Hz
    writeByte(EM7180_ADDRESS, EM7180_BaroRate, 0x80 | 0x32);  // set enable bit and set Baro rate to 25 Hz
    // writeByte(EM7180_ADDRESS, EM7180_TempRate, 0x19);  // set enable bit and set rate to 25 Hz

    // Configure operating mode
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // read scale sensor data
    // Enable interrupt to host upon certain events
    // choose host interrupts when any sensor updated (0x40), new gyro data (0x20), new accel data (0x10),
    // new mag data (0x08), quaternions updated (0x04), an error occurs (0x02), or the SENtral needs to be reset(0x01)
    writeByte(EM7180_ADDRESS, EM7180_EnableEvents, 0x07);
    // Enable EM7180 run mode
    writeByte(EM7180_ADDRESS, EM7180_HostControl, 0x01); // set SENtral in normal run mode
    delay(100);

    // EM7180 parameter adjustments
    Serial.println("Beginning Parameter Adjustments");

    // Read sensor default FS values from parameter space
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4A); // Request to read parameter 74
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); // Request parameter transfer process
    byte param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    while (!(param_xfer == 0x4A)) {
      param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    param[0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
    param[1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
    param[2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
    param[3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
    EM7180_mag_fs = ((int16_t)(param[1] << 8) | param[0]);
    EM7180_acc_fs = ((int16_t)(param[3] << 8) | param[2]);
    Serial.print("Magnetometer Default Full Scale Range: +/-"); Serial.print(EM7180_mag_fs); Serial.println("uT");
    Serial.print("Accelerometer Default Full Scale Range: +/-"); Serial.print(EM7180_acc_fs); Serial.println("g");
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4B); // Request to read  parameter 75
    param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    while (!(param_xfer == 0x4B)) {
      param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    param[0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
    param[1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
    param[2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
    param[3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
    EM7180_gyro_fs = ((int16_t)(param[1] << 8) | param[0]);
    Serial.print("Gyroscope Default Full Scale Range: +/-"); Serial.print(EM7180_gyro_fs); Serial.println("dps");
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //End parameter transfer
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // re-enable algorithm

    //Disable stillness mode
    EM7180_set_integer_param (0x49, 0x00);

    //Write desired sensor full scale ranges to the EM7180
    EM7180_set_mag_acc_FS (0x3E8, 0x08); // 1000 uT, 8 g
    EM7180_set_gyro_FS (0x7D0); // 2000 dps

    // Read sensor new FS values from parameter space
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4A); // Request to read  parameter 74
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x80); // Request parameter transfer process
    param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    while (!(param_xfer == 0x4A)) {
      param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    param[0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
    param[1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
    param[2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
    param[3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
    EM7180_mag_fs = ((int16_t)(param[1] << 8) | param[0]);
    EM7180_acc_fs = ((int16_t)(param[3] << 8) | param[2]);
    Serial.print("Magnetometer New Full Scale Range: +/-"); Serial.print(EM7180_mag_fs); Serial.println("uT");
    Serial.print("Accelerometer New Full Scale Range: +/-"); Serial.print(EM7180_acc_fs); Serial.println("g");
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x4B); // Request to read  parameter 75
    param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    while (!(param_xfer == 0x4B)) {
      param_xfer = readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
    }
    param[0] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte0);
    param[1] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte1);
    param[2] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte2);
    param[3] = readByte(EM7180_ADDRESS, EM7180_SavedParamByte3);
    EM7180_gyro_fs = ((int16_t)(param[1] << 8) | param[0]);
    Serial.print("Gyroscope New Full Scale Range: +/-"); Serial.print(EM7180_gyro_fs); Serial.println("dps");
    writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //End parameter transfer
    writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // re-enable algorithm


    // Read EM7180 status
    uint8_t runStatus = readByte(EM7180_ADDRESS, EM7180_RunStatus);
    if (runStatus & 0x01) Serial.println(" EM7180 run status = normal mode");
    uint8_t algoStatus = readByte(EM7180_ADDRESS, EM7180_AlgorithmStatus);
    if (algoStatus & 0x01) Serial.println(" EM7180 standby status");
    if (algoStatus & 0x02) Serial.println(" EM7180 algorithm slow");
    if (algoStatus & 0x04) Serial.println(" EM7180 in stillness mode");
    if (algoStatus & 0x08) Serial.println(" EM7180 mag calibration completed");
    if (algoStatus & 0x10) Serial.println(" EM7180 magnetic anomaly detected");
    if (algoStatus & 0x20) Serial.println(" EM7180 unreliable sensor data");
    uint8_t passthruStatus = readByte(EM7180_ADDRESS, EM7180_PassThruStatus);
    if (passthruStatus & 0x01) Serial.print(" EM7180 in passthru mode!");
    uint8_t eventStatus = readByte(EM7180_ADDRESS, EM7180_EventStatus);
    if (eventStatus & 0x01) Serial.println(" EM7180 CPU reset");
    if (eventStatus & 0x02) Serial.println(" EM7180 Error");
    if (eventStatus & 0x04) Serial.println(" EM7180 new quaternion result");
    if (eventStatus & 0x08) Serial.println(" EM7180 new mag result");
    if (eventStatus & 0x10) Serial.println(" EM7180 new accel result");
    if (eventStatus & 0x20) Serial.println(" EM7180 new gyro result");

    delay(1000); // give some time to read the screen

    // Check sensor status
    uint8_t sensorStatus = readByte(EM7180_ADDRESS, EM7180_SensorStatus);
    Serial.print(" EM7180 sensor status = "); Serial.println(sensorStatus);
    if (sensorStatus & 0x01) Serial.print("Magnetometer not acknowledging!");
    if (sensorStatus & 0x02) Serial.print("Accelerometer not acknowledging!");
    if (sensorStatus & 0x04) Serial.print("Gyro not acknowledging!");
    if (sensorStatus & 0x10) Serial.print("Magnetometer ID not recognized!");
    if (sensorStatus & 0x20) Serial.print("Accelerometer ID not recognized!");
    if (sensorStatus & 0x40) Serial.print("Gyro ID not recognized!");

    Serial.print("Actual MagRate = "); Serial.print(readByte(EM7180_ADDRESS, EM7180_ActualMagRate)); Serial.println(" Hz");
    Serial.print("Actual AccelRate = "); Serial.print(10 * readByte(EM7180_ADDRESS, EM7180_ActualAccelRate)); Serial.println(" Hz");
    Serial.print("Actual GyroRate = "); Serial.print(10 * readByte(EM7180_ADDRESS, EM7180_ActualGyroRate)); Serial.println(" Hz");
    Serial.print("Actual BaroRate = "); Serial.print(readByte(EM7180_ADDRESS, EM7180_ActualBaroRate)); Serial.println(" Hz");
    //  Serial.print("Actual TempRate = "); Serial.print(readByte(EM7180_ADDRESS, EM7180_ActualTempRate)); Serial.println(" Hz");

    delay(1000); // give some time to read the screen

  }

  // If pass through mode desired, set it up here
  if (passThru) {
    // Put EM7180 SENtral into pass-through mode
    SENtralPassThroughMode();
    delay(1000);

    I2Cscan(); // should see all the devices on the I2C bus including two from the EEPROM (ID page and data pages)

    // Read first page of EEPROM
    uint8_t data[128];
    M24512DFMreadBytes(M24512DFM_DATA_ADDRESS, 0x00, 0x00, 128, data);
    Serial.println("EEPROM Signature Byte");
    Serial.print(data[0], HEX); Serial.println("  Should be 0x2A");
    Serial.print(data[1], HEX); Serial.println("  Should be 0x65");
    for (int i = 0; i < 128; i++) {
      Serial.print(data[i], HEX); Serial.print(" ");
    }

    // Set up the interrupt pin, its set as active high, push-pull
    pinMode(myLed, OUTPUT);
    digitalWrite(myLed, HIGH);

    // Read the WHO_AM_I register, this is a good test of communication
    Serial.println("MPU9250 9-axis motion sensor...");
    byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
    Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);
    if (c == 0x71) // WHO_AM_I should always be 0x71
    {
      Serial.println("MPU9250 is online...");

      MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
      Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0], 1); Serial.println("% of factory value");
      Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1], 1); Serial.println("% of factory value");
      Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2], 1); Serial.println("% of factory value");
      Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3], 1); Serial.println("% of factory value");
      Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4], 1); Serial.println("% of factory value");
      Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5], 1); Serial.println("% of factory value");
      delay(1000);

      // get sensor resolutions, only need to do this once
      getAres();
      getGres();
      getMres();

      Serial.println(" Calibrate gyro and accel");
      accelgyrocalMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
      Serial.println("accel biases (mg)"); Serial.println(1000.*accelBias[0]); Serial.println(1000.*accelBias[1]); Serial.println(1000.*accelBias[2]);
      Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);

      delay(1000);

      initMPU9250();
      Serial.println("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
      writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
      I2Cscan(); // should see all the devices on the I2C bus including two from the EEPROM (ID page and data pages)

      // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
      byte d = readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);  // Read WHO_AM_I register for AK8963
      Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);

      delay(1000);

      // Get magnetometer calibration from AK8963 ROM
      initAK8963(magCalibration); Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer

      magcalMPU9250(magBias, magScale);
      Serial.println("AK8963 mag biases (mG)"); Serial.println(magBias[0]); Serial.println(magBias[1]); Serial.println(magBias[2]);
      Serial.println("AK8963 mag scale (mG)"); Serial.println(magScale[0]); Serial.println(magScale[1]); Serial.println(magScale[2]);
      delay(2000); // add delay to see results before serial spew of data

      if (SerialDebug) {
        //  Serial.println("Calibration values: ");
        Serial.print("X-Axis sensitivity adjustment value "); Serial.println(magCalibration[0], 2);
        Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(magCalibration[1], 2);
        Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(magCalibration[2], 2);
      }

      delay(1000);

      // Read the WHO_AM_I register of the BMP280 this is a good test of communication
      byte f = readByte(BMP280_ADDRESS, BMP280_ID);  // Read WHO_AM_I register for BMP280
      Serial.print("BMP280 ");
      Serial.print("I AM ");
      Serial.print(f, HEX);
      Serial.print(" I should be ");
      Serial.println(0x58, HEX);
      Serial.println(" ");

      delay(1000);

      writeByte(BMP280_ADDRESS, BMP280_RESET, 0xB6); // reset BMP280 before initilization
      delay(100);

      BMP280Init(); // Initialize BMP280 altimeter
      Serial.println("Calibration coeficients:");
      Serial.print("dig_T1 =");
      Serial.println(dig_T1);
      Serial.print("dig_T2 =");
      Serial.println(dig_T2);
      Serial.print("dig_T3 =");
      Serial.println(dig_T3);
      Serial.print("dig_P1 =");
      Serial.println(dig_P1);
      Serial.print("dig_P2 =");
      Serial.println(dig_P2);
      Serial.print("dig_P3 =");
      Serial.println(dig_P3);
      Serial.print("dig_P4 =");
      Serial.println(dig_P4);
      Serial.print("dig_P5 =");
      Serial.println(dig_P5);
      Serial.print("dig_P6 =");
      Serial.println(dig_P6);
      Serial.print("dig_P7 =");
      Serial.println(dig_P7);
      Serial.print("dig_P8 =");
      Serial.println(dig_P8);
      Serial.print("dig_P9 =");
      Serial.println(dig_P9);

      delay(1000);

    }
    else
    {
      Serial.print("Could not connect to MPU9250: 0x");
      Serial.println(c, HEX);
      while (1) ; // Loop forever if communication doesn't happen
    }
  }

  attachInterrupt(intPin, myinthandler, RISING);  // define interrupt for INT pin output of EM7180

  // Check event status register to clear the EM7180 interrupt before the main loop
  readByte(EM7180_ADDRESS, EM7180_EventStatus); // reading clears the register and interrupt


  // Misc
  rc_throttle    = &data[1];
  rc_roll        = &data[2];
  rc_pitch       = &data[3];
  rc_yaw         = &data[4];
  rc_mode        = &data[5];
  rc_aux1        = &data[6];
  rc_aux2        = &data[7];
  rc_orientation = &data[8];

  logTime = millis();
}

