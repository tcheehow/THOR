// Fixed Wing Mode ===================================================================================================

if (rc_mode < 1500) {

  // Set Mode Origins ============================================================================================
  l_origin = servo_min + 5;
  r_origin = servo_min + 2;

  // Prep Flight Mode ============================================================================================
  if (mode != FIXED_WING) {
    memset(e_Roll, 0, sizeof(e_Roll));
    memset(e_Pitch, 0, sizeof(e_Pitch));
    memset(e_sYaw, 0, sizeof(e_sYaw));

    mode = FIXED_WING;
  }

  // Collect Data from All Sensors ================================================================================
  if (readByte(LSM303D_ADDRESS, LSM303D_STATUS_REG_A) & 0x08) {  // check if new accel data is ready
    readAccelData(accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0] * aRes - accelBias[0]; // get actual g value, this depends on scale being set
    ay = (float)accelCount[1] * aRes - accelBias[1];
    az = (float)accelCount[2] * aRes - accelBias[2];
  }

  if (readByte(ITG3701_ADDRESS, ITG3701_INT_STATUS) & 0x01) {  // check if new gyro data is ready
    readGyroData(gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0] * gRes - gyroBias[0]; // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1] * gRes - gyroBias[1];
    gz = (float)gyroCount[2] * gRes - gyroBias[2];
  }

  if (readByte(LSM303D_ADDRESS, LSM303D_STATUS_REG_M) & 0x08) {  // check if new mag data is ready
    readMagData(magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    mx = (float)magCount[0] * mRes - magBias[0]; // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1] * mRes - magBias[1];
    mz = (float)magCount[2] * mRes - magBias[2];
  }

  // Process Data =============================================================================================

  MadgwickQuaternionUpdate(ax, ay, az, gx * PI / 180.0f, gy * PI / 180.0f, gz * PI / 180.0f,  mx,  my, -mz);
  //  MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, mx, my, -mz);

  yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
  pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
  roll  = -atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);

  // Determine Desired Positions and Check for Deadzones =========================================================
  if (rc_roll > 1450 && rc_roll < 1550) {
    des_roll = 0;
  }
  else {
    des_roll = map(rc_roll, 1000, 2000, -PI / 2, PI / 2);
  }

  if (rc_pitch > 1450 && rc_pitch < 1550) {
    des_pitch = 0;
  }
  else {
    des_pitch = map(rc_pitch, 1000, 2000, -PI / 2, PI / 2);
  }

  if (rc_yaw > 1450 && rc_yaw < 1550) {
    des_yaw = 0;
  }
  else {
    if (rc_throttle < 1000) {
      des_yaw = 0;
    }
    else {
      des_yaw = rc_yaw * PI / 1000;
    }
  }

  // Determine Outputs ========================================================================================

  // Throttle =============
  fw_throttle = rc_throttle;

  // Roll/Pitch ============
  e_Pitch[2] = (des_pitch - pitch) - e_Pitch[0];
  e_Roll[2]  = (des_roll - roll) - e_Roll[0];

  e_Pitch[0] = des_pitch - pitch;
  e_Roll[0]  = des_roll - roll;

  e_Pitch[1] += (e_Pitch[0] * deltat);
  e_Roll[1]  += (e_Roll[0] * deltat);

  pitch_out = K_Pitch[0] * e_Pitch[0] + K_Pitch[1] * e_Pitch[1] + K_Pitch[2] * e_Pitch[2];
  roll_out  = K_Roll[0] * e_Roll[0] + K_Roll[1] * e_Roll[1] + K_Roll[2] * e_Roll[2];

  // Yaw ============
  //  if (des_yaw == 0) {
  // do nothing
  // }
  // else if (des_yaw != 0 {
  //   des_heading = des_yaw + yaw;
  // }
  des_heading = 0;

  e_sYaw[2] = (yaw - des_heading) - e_sYaw[0];
  e_sYaw[0] = yaw - des_heading;
  e_sYaw[1] += (e_sYaw[0]  * deltat);

  yaw_out = K_sYaw[0] * e_sYaw[0] + K_sYaw[1] * e_sYaw[1] + K_sYaw[2] * e_sYaw[2];

  // Actual Outputs
  con_lflap = l_trim + l_origin - (pitch_out - roll_out);
  con_rflap = r_trim + r_origin + pitch_out + roll_out;

  // ===
  if (con_lflap > servo_max) {
    con_lflap = servo_max;
  }
  else if (con_lflap < servo_min) {
    con_lflap = servo_min;
  }

  if (con_rflap > servo_max) {
    con_rflap = servo_max;
  }
  else if (con_rflap < servo_min) {
    con_rflap = servo_min;
  }

  // ==

  if (yaw_out > 0) {
    con_lmotor = fw_throttle + yaw_out;
    con_rmotor = fw_throttle;
  }
  else if (yaw_out < 0) {
    con_lmotor = fw_throttle;
    con_rmotor = fw_throttle - yaw_out;
  }

  if (fw_throttle < 1000) {
    con_lmotor = con_rmotor = 1000;
  }


  // ===

  L_Flap.write(con_lflap);
  R_Flap.write(con_rflap);

  L_Motor.write(con_lmotor);
  R_Motor.write(con_rmotor);
}


