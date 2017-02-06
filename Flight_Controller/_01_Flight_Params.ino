// Define Flight Relevant Parameters

// Binary and Floating point (4 byte) converter.
typedef union {
  float floatingPoint;
  byte binary[4];
} binaryFloat;

// Comm Parameters
const uint8_t PACKET = 24;                    // Maximum 24 data items
float data[PACKET];                     // Array to store data for calculation
const uint8_t packet_size = PACKET*4;   // Equivalent bytes used (4 since float)

//  Radio Controller
const uint8_t RC_PIN = 9;               // PPM receiver port
float *rc_throttle, *rc_yaw,            //
*rc_pitch, *rc_roll, *rc_mode,          // Named RC variables for ease of reference
*rc_aux1, *rc_aux2, *rc_orientation;    //
PulsePositionInput receiver;            // Define the PPM input

// CC Link
uint32_t logTime;                       // Next scheduled logging time
byte packet[packet_size];               // Array to store data for sending

// Data Sending Config. Check here for auto-builder: https://docs.google.com/spreadsheets/d/1zc99wD7a6oJdFJYQSk1t636u66vdOulCESkB16MxtQM/edit#gid=2055303250
const byte fc_info[] = {B11111111, B10000001, B00000011, B10000111};

//  Notifiers
const uint8_t SYNC_LED_PIN = 13;        // LED pin for syncing with Optitrack data
