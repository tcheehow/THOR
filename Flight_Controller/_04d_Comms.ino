// Output for Recording =======================================================================================

if (delt_t > 500) {
  if (rc_mode < 1500) {
    if (debugger == true) {  // Print to Serial Monitor
      debug("[Fixed Wing]\tThrottle: ");
      debug(rc_throttle);
      debug(" \t");
      debug("Motor Left: ");
      debug(con_lmotor);
      debug(" \t");
      debug("Motor Right: ");
      debug(con_rmotor);
      debug("\t");
      debug("LFlap Command: ");
      debug(con_lflap);
      debug("\t");
      debug("RFlap Command: ");
      debug(con_rflap);
      debug("\t");
      debug("e_Pitch[0]: ");
      debug(e_Pitch[0]);
      debug("\t");
      debug("e_Roll[0]");
      debug(e_Roll[0]);
      debug("\t");
      debugln(" ");
    }
    else {  // Log to SD Card
      debug("@");
      debug(rc_throttle);
      debug(",");
      debug(rc_pitch);
      debug(",");
      debug(rc_roll);
      debug(",");
      debug(con_lmotor);
      debug(",");
      debug(con_rmotor);
      debug(",");
      debug(con_lflap);
      debug(",");
      debug(con_rflap);
      debug(",");
      debug(mx);
      debug(",");
      debug(my);
      debug(",");
      debug(mz);
      debug(",");
      debug(gx);
      debug(",");
      debug(gy);
      debug(",");
      debug(gz);
      debug(",");
      debug(ax);
      debug(",");
      debug(ay);
      debug(",");
      debug(az);
      debug(",");
      debug("#");
    }
  }
  else {
    if (debugger == true) {
      debug("[Monocopter]\tThrottle: ");
      debug(rc_throttle);
      debug(" \t");
      debug("Motor Left: ");
      debug(con_lmotor);
      debug(" \t");
      debug("Motor Right: ");
      debug(con_rmotor);
      debug("\t");
      debug("LFlap Command: ");
      debug(con_lflap);
      debug("\t");
      debug("RFlap Command: ");
      debug(con_rflap);
      debug("\t");
      debug("K_Mono: ");
      debug(K_Mono);
      debug("\t");
      debug("Mono_Trim:");
      debugln(mono_trim);
    }
    else {
      //"Throttle", "Pitch", "Roll","Motor Left", "Motor Right","LFlap","RFlap","mx","my","mz","K_Mono","Mono_Trim"
      debug("@");
      debug(rc_throttle);
      debug(",");
      debug(rc_pitch);
      debug(",");
      debug(rc_roll);
      debug(",");
      debug(con_lmotor);
      debug(",");
      debug(con_rmotor);
      debug(",");
      debug(con_lflap);
      debug(",");
      debug(con_rflap);
      debug(",");
      debug(mx);
      debug(",");
      debug(my);
      debug(",");
      debug(mz);
      debug(",");
      debug(gx);
      debug(",");
      debug(gy);
      debug(",");
      debug(gz);
      debug(",");
      debug(ax);
      debug(",");
      debug(ay);
      debug(",");
      debug(az);
      debug(",");
      debug(K_Mono);
      debug(",");
      debug(mono_trim);
      debug("#");
    }

  }

  count = millis();
  sumCount = 0;
  sum = 0;
}


// Disarmer

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
  mode = DEACTIVATED;
  arming = false;
}


}

