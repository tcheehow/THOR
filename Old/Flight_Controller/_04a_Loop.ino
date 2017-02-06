void loop()
{
  //====================================================================================================================================
  // Deactivated =======================================================================================================================

  while (mode == DEACTIVATED) {
    if (mode != DEACTIVATED) {
      memset(e_Roll, 0, sizeof(e_Roll));
      memset(e_Pitch, 0, sizeof(e_Pitch));
      memset(e_sYaw, 0, sizeof(e_sYaw));
      mode = DEACTIVATED;
    }

    digitalWrite(STATUS_LED, LOW);

    int num = 0;

    num = myIn.available();

    if (num > 0) {
      // Collect Inputs =============================================================================================

      bool rc_valid = true;

      for (int i = 1; i < 8; i++) {
        if (myIn.read(i) < 900) {
          rc_valid = false;
        }
      }

      if (rc_valid == true) {
        rc_throttle    = myIn.read(1);
        rc_roll        = myIn.read(2);
        rc_pitch       = myIn.read(3);
        rc_yaw         = myIn.read(4);
        rc_mode        = myIn.read(5);
        rc_aux1        = myIn.read(6);
        rc_aux2        = myIn.read(7);
        rc_orientation = myIn.read(8);
      }

      // Uncomment for Quick RC Calibration
      if (rc_calib == true) {
        R_Motor.write(rc_throttle);
        L_Motor.write(rc_throttle);
      }
      else {
        // do nothing
      }

      if (rc_throttle < 1000 && rc_yaw > 1900) {
        if (arming == false) {
          arm_start = millis();
          arming = true;
        }
      }
      else {
        arm_start = 0;
        arming = false;
      }

      if (millis() > (arm_start + 2000) && arming == true) {
        arming = false;
        rc_throttle_min = rc_throttle;
        break;
      }
    }
  }
  //====================================================================================================================================
  // Activated =======================================================================================================================
  digitalWrite(STATUS_LED, HIGH);
  // Timer Updates ==============================================================================================
  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;

  sum += deltat; // sum for averaging filter update rate
  sumCount++;
  // Serial print and/or display at 0.5 s rate independent of data rates
  delt_t = millis() - count;


  // Collect Inputs =============================================================================================

  bool rc_valid = true;

  for (int i = 1; i < 8; i++) {
    if (myIn.read(i) < 900) {
      rc_valid = false;
    }
  }

  if (rc_valid == true) {
    rc_throttle    = myIn.read(1);
    rc_roll        = myIn.read(2);
    rc_pitch       = myIn.read(3);
    rc_yaw         = myIn.read(4);
    rc_mode        = myIn.read(5);
    rc_aux1        = myIn.read(6);
    rc_aux2        = myIn.read(7);
    rc_orientation = myIn.read(8);
  }

  // On-The-Fly Editing =========================================================================================
  if (rc_aux2 < 1000) {
    r_trim = map(rc_aux1, 900, 1900, -60, 60);

  }
  else if (rc_aux2 > 1900) {
    l_trim = map(rc_aux1, 900, 1900, -60, 60);

  }
  else {
    // do nothing
  }

