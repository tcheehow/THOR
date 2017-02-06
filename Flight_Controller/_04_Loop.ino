void loop() {
  data_manager();  // Gather sensor/input data

  if (!passThru) {


    //If intPin goes high, the EM7180 has new data
    if (newData == true) { // On interrupt, read data
      newData = false;  // reset newData flag


      // Check event status register, after receipt of interrupt
      uint8_t eventStatus = readByte(EM7180_ADDRESS, EM7180_EventStatus); // reading clears the register

      // Check for errors
      if (eventStatus & 0x02) { // error detected, what is it?

        uint8_t errorStatus = readByte(EM7180_ADDRESS, EM7180_ErrorRegister);
        if (errorStatus != 0x00) { // non-zero value indicates error, what is it?
          Serial.print(" EM7180 sensor status = "); Serial.println(errorStatus);
          if (errorStatus == 0x11) Serial.print("Magnetometer failure!");
          if (errorStatus == 0x12) Serial.print("Accelerometer failure!");
          if (errorStatus == 0x14) Serial.print("Gyro failure!");
          if (errorStatus == 0x21) Serial.print("Magnetometer initialization failure!");
          if (errorStatus == 0x22) Serial.print("Accelerometer initialization failure!");
          if (errorStatus == 0x24) Serial.print("Gyro initialization failure!");
          if (errorStatus == 0x30) Serial.print("Math error!");
          if (errorStatus == 0x80) Serial.print("Invalid sample rate!");
        }

        // Handle errors ToDo

      }

      // if no errors, see if new data is ready
      if (eventStatus & 0x10) { // new acceleration data available
        readSENtralAccelData(accelCount);

        // Now we'll calculate the acceleration value into actual g's
        ax = (float)accelCount[0] * 0.000488; // get actual g value
        ay = (float)accelCount[1] * 0.000488;
        az = (float)accelCount[2] * 0.000488;
      }

      if (eventStatus & 0x20) { // new gyro data available
        readSENtralGyroData(gyroCount);

        // Now we'll calculate the gyro value into actual dps's
        gx = (float)gyroCount[0] * 0.153; // get actual dps value
        gy = (float)gyroCount[1] * 0.153;
        gz = (float)gyroCount[2] * 0.153;
      }

      if (eventStatus & 0x08) { // new mag data available
        readSENtralMagData(magCount);

        // Now we'll calculate the mag value into actual G's
        mx = (float)magCount[0] * 0.305176; // get actual G value
        my = (float)magCount[1] * 0.305176;
        mz = (float)magCount[2] * 0.305176;
      }

      if (eventStatus & 0x04) { // new quaternion data available
        readSENtralQuatData(Quat);
      }

      // get MS5637 pressure
      if (eventStatus & 0x40) { // new baro data available
        rawPressure = readSENtralBaroData();
        pressure = (float)rawPressure * 0.01f + 1013.25f; // pressure in mBar

        // get MS5637 temperature
        rawTemperature = readSENtralTempData();
        temperature = (float) rawTemperature * 0.01; // temperature in degrees C
      }

      Yaw   = atan2(2.0f * (Quat[0] * Quat[1] + Quat[3] * Quat[2]), Quat[3] * Quat[3] + Quat[0] * Quat[0] - Quat[1] * Quat[1] - Quat[2] * Quat[2]);
      Pitch = -asin(2.0f * (Quat[0] * Quat[2] - Quat[3] * Quat[1]));
      Roll  = atan2(2.0f * (Quat[3] * Quat[0] + Quat[1] * Quat[2]), Quat[3] * Quat[3] - Quat[0] * Quat[0] - Quat[1] * Quat[1] + Quat[2] * Quat[2]);
      Pitch *= 180.0f / PI;
      Yaw   *= 180.0f / PI;
      Yaw   += 0.23f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
      if (Yaw < 0) Yaw   += 360.0f ; // Ensure yaw stays between 0 and 360
      Roll  *= 180.0f / PI;

      data[14] = mx;
      data[15] = my;
      data[16] = mz;

      data[21] = Roll;
      data[22] = Pitch;
      data[23] = Yaw;

    }

    // Feed through controller

    // Sync log with cameras using LED.
    if (*rc_aux2 > 2000) {
      digitalWrite(SYNC_LED_PIN, HIGH);
    }
    else {
      digitalWrite(SYNC_LED_PIN, LOW);
    }

    // Send data over if it's the interval
    if (millis() - logTime > 10) {
      courier();
      logTime = millis();
    }
  }
}
