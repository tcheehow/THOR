//===================================================================================================================
//====== Sensor Functions
//===================================================================================================================

// I2C read/write functions for the LSM303Dand AK8963 sensors

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
  //  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  //  Wire.requestFrom(address, 1);  // Read one byte from slave register address
  Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(0x80 | subAddress);     // Put slave register address in Tx buffer, include 0x80 for LSM303D multiple byte read
  Wire.endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive
  //  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  //        Wire.requestFrom(address, count);  // Read bytes from slave register address
  Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address
  while (Wire.available()) {
    dest[i++] = Wire.read();
  }         // Put read results in the Rx buffer
}


void getMres() {
  switch (Mscale)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 2 Gauss (00), 4 Gauss (01), 8 Gauss (10) and 12 Gauss (11)
    case MFS_2G:
      mRes = 2.0 / 32768.0;
      break;
    case MFS_4G:
      mRes = 4.0 / 32768.0;
      break;
    case MFS_8G:
      mRes = 8.0 / 32768.0;
      break;
    case MFS_12G:
      mRes = 12.0 / 32768.0;
      break;
  }
}

void getGres() {
  switch (Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 500 DPS (00), 1000 DPS (01), 2000 DPS (10) and 4000 DPS  (11).
    case GFS_500DPS:
      gRes = 500.0 / 32768.0;
      break;
    case GFS_1000DPS:
      gRes = 1000.0 / 32768.0;
      break;
    case GFS_2000DPS:
      gRes = 2000.0 / 32768.0;
      break;
    case GFS_4000DPS:
      gRes = 4000.0 / 32768.0;
      break;
  }
}

void getAres() {
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (000), 4 Gs (001), 6 gs (010), 8 Gs (011), and 16 gs (100).
    case AFS_2G:
      aRes = 2.0 / 32768.0;
      break;
    case AFS_4G:
      aRes = 4.0 / 32768.0;
      break;
    case AFS_6G:
      aRes = 6.0 / 32768.0;
      break;
    case AFS_8G:
      aRes = 8.0 / 32768.0;
      break;
    case AFS_16G:
      aRes = 16.0 / 32768.0;
      break;
  }
}


void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(LSM303D_ADDRESS, LSM303D_OUT_X_L_A, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}


void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(ITG3701_ADDRESS, ITG3701_GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

void readMagData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(LSM303D_ADDRESS, LSM303D_OUT_X_L_M, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

int16_t readTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(LSM303D_ADDRESS, LSM303D_OUT_TEMP_L_XM, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
  return (((int16_t)rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a 16-bit signed value
}


void initITG3701()
{
  // wake up device
  writeByte(ITG3701_ADDRESS, ITG3701_PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  delay(100); // Wait for all registers to reset

  // get stable time source
  writeByte(ITG3701_ADDRESS, ITG3701_PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200);

  // Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
  // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
  // be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the ITG3701, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(ITG3701_ADDRESS, ITG3701_CONFIG, 0x03);

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(ITG3701_ADDRESS, ITG3701_SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
  // determined inset in CONFIG above

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(ITG3701_ADDRESS, ITG3701_GYRO_CONFIG);
  //  writeRegister(GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
  writeByte(ITG3701_ADDRESS, ITG3701_GYRO_CONFIG, c & ~0x02); // Clear Fchoice bits [1:0]
  writeByte(ITG3701_ADDRESS, ITG3701_GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(ITG3701_ADDRESS, ITG3701_GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro
  // writeRegister(GYRO_CONFIG, c | 0x00); // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
  writeByte(ITG3701_ADDRESS, ITG3701_INT_PIN_CFG, 0x20);
  writeByte(ITG3701_ADDRESS, ITG3701_INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
  delay(100);
}


void initLSM303D()
{
  // configure the accelerometer-specify ODR (sample rate) selection with Aodr, enable block data update
  writeByte(LSM303D_ADDRESS, LSM303D_CTRL_REG1_XM, Aodr << 4 | 0x0F);
  // configure the accelerometer-specify bandwidth and full-scale selection with Abw, Ascale
  writeByte(LSM303D_ADDRESS, LSM303D_CTRL_REG2_XM, Abw << 6 | Ascale << 3);
  // enable temperature sensor, set magnetometer ODR (sample rate) and resolution mode
  writeByte(LSM303D_ADDRESS, LSM303D_CTRL_REG5_XM, 0x80 | Mres << 5 | Modr << 2);
  // set magnetometer full scale
  writeByte(LSM303D_ADDRESS, LSM303D_CTRL_REG6_XM, Mscale << 5 & 0x60);
  writeByte(LSM303D_ADDRESS, LSM303D_CTRL_REG7_XM, 0x00); // select continuous conversion mode
}


// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void accelcalLSM303D(float * dest1)
{
  uint8_t data[6] = {0, 0, 0, 0, 0, 0};
  int32_t accel_bias[3] = {0, 0, 0};
  uint16_t samples, ii;


  debugln("Calibrating accel...");

  // now get the accelerometer bias
  byte c = readByte(LSM303D_ADDRESS, LSM303D_CTRL_REG0_XM);
  writeByte(LSM303D_ADDRESS, LSM303D_CTRL_REG0_XM, c | 0x40);     // Enable gyro FIFO
  delay(200);                                                       // Wait for change to take effect
  writeByte(LSM303D_ADDRESS, LSM303D_FIFO_CTRL_REG, 0x20 | 0x1F);  // Enable gyro FIFO stream mode and set watermark at 32 samples
  delay(1000);  // delay 1000 milliseconds to collect FIFO samples

  samples = (readByte(LSM303D_ADDRESS, LSM303D_FIFO_SRC_REG) & 0x1F); // Read number of stored samples

  for (ii = 0; ii < samples ; ii++) {           // Read the gyro data stored in the FIFO
    int16_t accel_temp[3] = {0, 0, 0};
    readBytes(LSM303D_ADDRESS, LSM303D_OUT_X_L_A, 6, &data[0]);
    accel_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]); // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]);
    accel_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]);

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
  }

  accel_bias[0] /= samples; // average the data
  accel_bias[1] /= samples;
  accel_bias[2] /= samples;

  if (accel_bias[2] > 0L) {
    accel_bias[2] -= (int32_t) (1.0 / aRes); // Remove gravity from the z-axis accelerometer bias calculation
  }
  else {
    accel_bias[2] += (int32_t) (1.0 / aRes);
  }

  dest1[0] = (float)accel_bias[0] * aRes; // Properly scale the data to get g
  dest1[1] = (float)accel_bias[1] * aRes;
  dest1[2] = (float)accel_bias[2] * aRes;

  c = readByte(LSM303D_ADDRESS, LSM303D_CTRL_REG0_XM);
  writeByte(LSM303D_ADDRESS, LSM303D_CTRL_REG0_XM, c & ~0x40);   //Disable accel FIFO
  delay(200);
  writeByte(LSM303D_ADDRESS, LSM303D_FIFO_CTRL_REG, 0x00);  // Enable accel bypass mode
}

void magcalLSM303D(float * dest1)
{
  uint8_t data[6]; // data array to hold mag x, y, z, data
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0};
  int16_t mag_max[3] = {0, 0, 0}, mag_min[3] = {0, 0, 0};

  debugln("Mag Calibration: Wave device in a figure eight until done!");
  delay(4000);

  sample_count = 128;
  for (ii = 0; ii < sample_count; ii++) {
    int16_t mag_temp[3] = {0, 0, 0};
    readBytes(LSM303D_ADDRESS, LSM303D_OUT_X_L_M, 6, &data[0]);  // Read the six raw data registers into data array
    mag_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]) ;   // Form signed 16-bit integer for each sample in FIFO
    mag_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]) ;
    mag_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]) ;
    for (int jj = 0; jj < 3; jj++) {
      if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    delay(105);  // at 10 Hz ODR, new mag data is available every 100 ms
  }

  //    debugln("mag x min/max:"); debugln(mag_max[0]); debugln(mag_min[0]);
  //    debugln("mag y min/max:"); debugln(mag_max[1]); debugln(mag_min[1]);
  //    debugln("mag z min/max:"); debugln(mag_max[2]); debugln(mag_min[2]);

  mag_bias[0]  = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias in counts

  dest1[0] = (float) mag_bias[0] * mRes; // save mag biases in G for main program
  dest1[1] = (float) mag_bias[1] * mRes;
  dest1[2] = (float) mag_bias[2] * mRes;

  /* //write biases to accelerometermagnetometer offset registers as counts);
    writeByte(LSM303DM_ADDRESS, LSM303DM_OFFSET_X_REG_L_M, (int16_t) mag_bias[0]  & 0xFF);
    writeByte(LSM303DM_ADDRESS, LSM303DM_OFFSET_X_REG_H_M, ((int16_t)mag_bias[0] >> 8) & 0xFF);
    writeByte(LSM303DM_ADDRESS, LSM303DM_OFFSET_Y_REG_L_M, (int16_t) mag_bias[1] & 0xFF);
    writeByte(LSM303DM_ADDRESS, LSM303DM_OFFSET_Y_REG_H_M, ((int16_t)mag_bias[1] >> 8) & 0xFF);
    writeByte(LSM303DM_ADDRESS, LSM303DM_OFFSET_Z_REG_L_M, (int16_t) mag_bias[2] & 0xFF);
    writeByte(LSM303DM_ADDRESS, LSM303DM_OFFSET_Z_REG_H_M, ((int16_t)mag_bias[2] >> 8) & 0xFF);
  */
  debugln("Mag Calibration done!");
}

// Function which accumulates gyro data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into gyro bias registers.
void gyrocalITG3701(float * dest1)
{
  uint8_t data[6]; // data array to hold gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0};

  // reset device
  writeByte(ITG3701_ADDRESS, ITG3701_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);

  // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
  // else use the internal oscillator, bits 2:0 = 001
  writeByte(ITG3701_ADDRESS, ITG3701_PWR_MGMT_1, 0x01);
  writeByte(ITG3701_ADDRESS, ITG3701_PWR_MGMT_2, 0x00);
  delay(200);

  // Configure device for bias calculation
  writeByte(ITG3701_ADDRESS, ITG3701_INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(ITG3701_ADDRESS, ITG3701_FIFO_EN, 0x00);      // Disable FIFO
  writeByte(ITG3701_ADDRESS, ITG3701_PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(ITG3701_ADDRESS, ITG3701_USER_CTRL, 0x00);    // Disable FIFO
  writeByte(ITG3701_ADDRESS, ITG3701_USER_CTRL, 0x04);    // Reset FIFO
  delay(15);

  // Configure ITG3701 gyro for bias calculation
  writeByte(ITG3701_ADDRESS, ITG3701_CONFIG, 0x01);      // Set low-pass filter to 184 Hz
  writeByte(ITG3701_ADDRESS, ITG3701_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(ITG3701_ADDRESS, ITG3701_GYRO_CONFIG, 0x00); // Set gyro full-scale to 500 degrees per second, maximum sensitivity

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(ITG3701_ADDRESS, ITG3701_USER_CTRL, 0x40);   // Enable FIFO
  writeByte(ITG3701_ADDRESS, ITG3701_FIFO_EN, 0x70);     // Enable gyro sensors for FIFO  (max size 512 bytes in ITG3701)
  delay(100); // accumulate 80 samples in 80 milliseconds = 480 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  writeByte(ITG3701_ADDRESS, ITG3701_FIFO_EN, 0x00);        // Disable gyro sensors for FIFO
  readBytes(ITG3701_ADDRESS, ITG3701_FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = (uint16_t)((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/6; // How many sets of full gyro data for averaging
  //packet_count = 128; // How many sets of full gyro data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t gyro_temp[3] = {0, 0, 0};
    //    readBytes(ITG3701_ADDRESS, ITG3701_FIFO_R_W, 6, &data[0]); // read data for averaging
    readBytes(ITG3701_ADDRESS, ITG3701_GYRO_XOUT_H, 6, &data[0]); // read data for averaging
    gyro_temp[0]  = (int16_t) (((int16_t)data[0] << 8) | data[1]) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[2] << 8) | data[3]) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[4] << 8) | data[5]) ;

    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
    delay(5);
  }
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;

  // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]  >> 8) & 0xFF;
  data[1] = (-gyro_bias[0])       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1])       & 0xFF;
  data[4] = (-gyro_bias[2]  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2])       & 0xFF;

  // Push gyro biases to hardware registers
  //  writeByte(ITG3701_ADDRESS, ITG3701_XG_OFFS_USRH, data[0]);
  //  writeByte(ITG3701_ADDRESS, ITG3701_XG_OFFS_USRL, data[1]);
  //  writeByte(ITG3701_ADDRESS, ITG3701_YG_OFFS_USRH, data[2]);
  //  writeByte(ITG3701_ADDRESS, ITG3701_YG_OFFS_USRL, data[3]);
  //  writeByte(ITG3701_ADDRESS, ITG3701_ZG_OFFS_USRH, data[4]);
  //  writeByte(ITG3701_ADDRESS, ITG3701_ZG_OFFS_USRL, data[5]);

  // Output scaled gyro biases for display in the main program
  dest1[0] = (float) gyro_bias[0] * gRes;
  dest1[1] = (float) gyro_bias[1] * gRes;
  dest1[2] = (float) gyro_bias[2] * gRes;
}

/*
  // Gyroscope self test; check calibration wrt factory settings
  void ITG3701SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
  {
   uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
   uint8_t selfTest[3];
   int16_t gAvg[3], gSTAvg[3];
   float factoryTrim[3];
   uint8_t FS = 0;

  writeByte(ITG3701_ADDRESS, ITG3701_SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
  writeByte(ITG3701_ADDRESS, ITG3701_CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeByte(ITG3701_ADDRESS, ITG3701_GYRO_CONFIG, 1<<FS);  // Set full scale range for the gyro to 250 dps
  for( int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer

    readBytes(ITG3701_ADDRESS, ITG3701_GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
  gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
  gAvg[ii] /= 200;
  }

  // Configure the accelerometer for self-test
   writeByte(ITG3701_ADDRESS, ITG3701_GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   delay(25);  // Delay a while to let the device stabilize
  for( int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer

   readBytes(ITG3701_ADDRESS, ITG3701_GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
  gSTAvg[ii] /= 200;
  }

  // Configure the gyro and accelerometer for normal operation
  writeByte(ITG3701_ADDRESS, ITG3701_GYRO_CONFIG,  0x00);
   delay(25);  // Delay a while to let the device stabilize

   // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
   selfTest[0] = readByte(ITG3701_ADDRESS, ITG3701_SELF_TEST_X_GYRO);  // X-axis gyro self-test results
   selfTest[1] = readByte(ITG3701_ADDRESS, ITG3701_SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
   selfTest[2] = readByte(ITG3701_ADDRESS, ITG3701_SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results
  // Retrieve factory self-test value from self-test code reads
   factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
   factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
   factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
  // To get percent, must multiply by 100
   for (int i = 0; i < 3; i++) {
     destination[i]   = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i];   // Report percent differences
   }

  }
*/

// I2C communication with the MS5637 is a little different from that with the LSM303Dand most other sensors
// For the MS5637, we write commands, and the MS5637 sends data in response, rather than directly reading
// MS5637 registers

void MS5637Reset()
{
  Wire.beginTransmission(MS5637_ADDRESS);  // Initialize the Tx buffer
  Wire.write(MS5637_RESET);                // Put reset command in Tx buffer
  Wire.endTransmission();                  // Send the Tx buffer
}

void MS5637PromRead(uint16_t * destination)
{
  uint8_t data[2] = {0, 0};
  for (uint8_t ii = 0; ii < 8; ii++) {
    Wire.beginTransmission(MS5637_ADDRESS);  // Initialize the Tx buffer
    Wire.write(0xA0 | ii << 1);              // Put PROM address in Tx buffer
    Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    Wire.requestFrom(MS5637_ADDRESS, 2);   // Read two bytes from slave PROM address
    while (Wire.available()) {
      data[i++] = Wire.read();
    }               // Put read results in the Rx buffer
    destination[ii] = (uint16_t) (((uint16_t) data[0] << 8) | data[1]); // construct PROM data for return to main program
  }
}

uint32_t MS5637Read(uint8_t CMD, uint8_t OSR)  // temperature data read
{
  uint8_t data[3] = {0, 0, 0};
  Wire.beginTransmission(MS5637_ADDRESS);  // Initialize the Tx buffer
  Wire.write(CMD | OSR);                  // Put pressure conversion command in Tx buffer
  Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive

  switch (OSR)
  {
    case ADC_256: delay(1); break;  // delay for conversion to complete
    case ADC_512: delay(3); break;
    case ADC_1024: delay(4); break;
    case ADC_2048: delay(6); break;
    case ADC_4096: delay(10); break;
    case ADC_8192: delay(20); break;
  }

  Wire.beginTransmission(MS5637_ADDRESS);  // Initialize the Tx buffer
  Wire.write(0x00);                        // Put ADC read command in Tx buffer
  Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(MS5637_ADDRESS, 3);     // Read three bytes from slave PROM address
  while (Wire.available()) {
    data[i++] = Wire.read();
  }               // Put read results in the Rx buffer
  return (uint32_t) (((uint32_t) data[0] << 16) | (uint32_t) data[1] << 8 | data[2]); // construct PROM data for return to main program
}



unsigned char MS5637checkCRC(uint16_t * n_prom)  // calculate checksum from PROM register contents
{
  int cnt;
  unsigned int n_rem = 0;
  unsigned char n_bit;

  n_prom[0] = ((n_prom[0]) & 0x0FFF);  // replace CRC byte by 0 for checksum calculation
  n_prom[7] = 0;
  for (cnt = 0; cnt < 16; cnt++)
  {
    if (cnt % 2 == 1) n_rem ^= (unsigned short) ((n_prom[cnt >> 1]) & 0x00FF);
    else         n_rem ^= (unsigned short)  (n_prom[cnt >> 1] >> 8);
    for (n_bit = 8; n_bit > 0; n_bit--)
    {
      if (n_rem & 0x8000)    n_rem = (n_rem << 1) ^ 0x3000;
      else                  n_rem = (n_rem << 1);
    }
  }
  n_rem = ((n_rem >> 12) & 0x000F);
  return (n_rem ^ 0x00);
}

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
  float norm;
  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;

  // Auxiliary variables to avoid repeated arithmetic
  float _2q1mx;
  float _2q1my;
  float _2q1mz;
  float _2q2mx;
  float _4bx;
  float _4bz;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * q1 * mx;
  _2q1my = 2.0f * q1 * my;
  _2q1mz = 2.0f * q1 * mz;
  _2q2mx = 2.0f * q2 * mx;
  hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
  hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
  _2bx = sqrt(hx * hx + hy * hy);
  _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
  norm = 1.0f / norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;

  // Compute rate of change of quaternion
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

  // Integrate to yield quaternion
  q1 += qDot1 * deltat;
  q2 += qDot2 * deltat;
  q3 += qDot3 * deltat;
  q4 += qDot4 * deltat;
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;

}

void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;        // use reciprocal for division
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;        // use reciprocal for division
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
  hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
  bx = sqrt((hx * hx) + (hy * hy));
  bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

  // Estimated direction of gravity and magnetic field
  vx = 2.0f * (q2q4 - q1q3);
  vy = 2.0f * (q1q2 + q3q4);
  vz = q1q1 - q2q2 - q3q3 + q4q4;
  wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
  wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
  wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

  // Error is cross product between estimated direction and measured direction of gravity
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  if (Ki > 0.0f)
  {
    eInt[0] += ex;      // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
  }
  else
  {
    eInt[0] = 0.0f;     // prevent integral wind up
    eInt[1] = 0.0f;
    eInt[2] = 0.0f;
  }

  // Apply feedback terms
  gx = gx + Kp * ex + Ki * eInt[0];
  gy = gy + Kp * ey + Ki * eInt[1];
  gz = gz + Kp * ez + Ki * eInt[2];

  // Integrate rate of change of quaternion
  pa = q2;
  pb = q3;
  pc = q4;
  q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
  q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
  q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
  q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

  // Normalise quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;

}
