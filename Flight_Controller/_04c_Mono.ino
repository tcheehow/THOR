// Monocopter Mode ===================================================================================================
else if (rc_mode > 1500) {
  // Set Mode Origins ============================================================================================

  l_origin = servo_max;
  r_origin = servo_max;

  // Prep Flight Mode ============================================================================================
  if (mode != MONOCOPTER) {
    memset(e_Roll, 0, sizeof(e_Roll));
    memset(e_Pitch, 0, sizeof(e_Pitch));
    memset(e_Yaw, 0, sizeof(e_Yaw));
    mode = MONOCOPTER;
  }

  // Determine Desired Positions and Check for Deadzones =========================================================

  // Collect Data from Magnotometer Only ================================================================================

  if (readByte(LSM303D_ADDRESS, LSM303D_STATUS_REG_M) & 0x08) {  // check if new mag data is ready
    readMagData(magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    mx = (float)magCount[0] * mRes - magBias[0]; // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1] * mRes - magBias[1];
    mz = (float)magCount[2] * mRes - magBias[2];
  }

  // Determine Desired Positions and Check for Deadzones =========================================================
  if (rc_roll > 1450 && rc_roll < 1550) {
    des_roll = 0;
  }
  else {
    des_roll = map(rc_roll, RC_min, RC_max, -1000, 1000) / 1000.0;
  }

  if (rc_pitch > 1450 && rc_pitch < 1550) {
    des_pitch = 0;
  }
  else {
    des_pitch = map(rc_pitch, RC_min, RC_max, -1000, 1000) / 1000.0;
  }

  // Process Data =============================================================================================
  // comm_dir = atan2(rc_pitch, rc_roll) - (PI / 2) + (PI / 2) + phi_wing + phi_fwd;

  phi_curr = atan2(my, mz);
  phi_pilot = map(rc_orientation, 900, 1900, -PI, PI);
  phi_fwd = phi_curr - phi_pilot;

  comm_dir = atan2(des_pitch, des_roll) - (PI / 2) + phi_fwd;

  comm_mag = K_Mono * ((pow(des_pitch, 2) + 2 * pow(des_roll, 2)) / 2);

  servo_comm = comm_mag * cos(phi_pilot - comm_dir);


  // Determine Outputs ========================================================================================
  con_lflap = l_origin + l_trim - servo_comm;
  con_rflap = r_origin + r_trim + servo_comm;

  //con_lflap = l_origin + mono_trim;
  //con_rflap = r_origin + mono_trim;

  con_lmotor = rc_throttle;
  con_rmotor = rc_throttle;

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

  L_Flap.write(con_lflap);
  R_Flap.write(con_rflap);

  L_Motor.write(con_lmotor);
  R_Motor.write(con_rmotor);

}

// End of Flight Controller Section ============================
