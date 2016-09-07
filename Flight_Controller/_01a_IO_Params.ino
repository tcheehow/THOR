// ===========================================================================================================
//  LED ==============================================================================================

const int8_t STATUS_LED       = 2;

// ===========================================================================================================
//  Radio Controller/Modes =========================================================================================

const int8_t RC = 9;
float rc_throttle, rc_yaw, rc_pitch, rc_roll, rc_mode, rc_aux1, rc_aux2, rc_orientation;

enum modes {  // set of allowable mag full scale settings
  DEACTIVATED = 0,
  FIXED_WING,
  MONOCOPTER
};

uint8_t mode = DEACTIVATED;
float arm_start = 0, rc_throttle_min = 1000;
bool arming = false;

PulsePositionInput myIn;

// ===========================================================================================================
//  Motors and Servos ========================================================================================

const int8_t L_MOTOR = 20;
const int8_t R_MOTOR = 21;

const int8_t L_FLAP = 22;
const int8_t R_FLAP = 23;

// ===========================================================================================================
// Fixed Wing Controller Variables ===========================================================================

float des_yaw, des_roll, des_pitch, des_heading, yaw_out, pitch_out, roll_out, fw_trim;

float e_Roll[3] = {0.0f, 0.0f, 0.0f};
float e_Pitch[3] = {0.0f, 0.0f, 0.0f};
float e_sYaw[3] = {0.0f, 0.0f, 0.0f};
float e_rYaw[3] = {0.0f, 0.0f, 0.0f};

float K_Roll[3] = {0.0f, 0.00f, 0.0f};
float K_Pitch[3] = {0.0f, 0.00f, 0.0f};
float K_sYaw[3] = {0.0f, 0.0f, 0.0f};
float K_rYaw[3] = {0.0f, 0.0f, 0.0f};

float fw_throttle;

// ===========================================================================================================
// Monocopter Controller Variables ===========================================================================

float phi_fwd, phi_curr, phi_pilot, comm_dir, comm_mag, servo_comm, mono_trim, mono_throttle;
float K_Mono = 2 ;

// ===========================================================================================================
// Outputs ===================================================================================================

Servo R_Flap;
Servo L_Flap;

Servo R_Motor;
Servo L_Motor;

float servo_min = 0;
float servo_max = 60;
float l_trim, r_trim, l_origin, r_origin;
float con_lmotor, con_rmotor, con_lflap = 0, con_rflap = 0;
float lflap_check, rflap_check;

// ===========================================================================================================
// Data Packets ===============================================================================================


