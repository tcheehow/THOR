// Define the communication parameters.

// Binary and Floating point (4 byte) converter.
typedef union {
  float floatingPoint;
  byte binary[4];
} binaryFloat;

// Header Parameters
String header = "";              // Variable to store header for recorded file.
const char *data_pool[] =        // Key that links binary data to data text.
{ "Time", "Throttle", "Roll Input", "Pitch Input", "Yaw Input",  // 5
  "Mode", "AUX1", "AUX2", "Orientation", "Current", "Voltage",   // 6
  "ax", "ay", "az", "mx", "my", "mz", "gx", "gy", "gz",          // 9
  "Altitude", "Roll Angle", "Pitch Angle", "Yaw Angle"           // 4
};

// Data Parameters
byte inData[96];                 // Incoming bytes of data from FC.
uint8_t data_tally = 0;          // Units of data per FC transmission.

// Comm Notifiers
const uint8_t LED_PIN = 13;      // Recording/Not Recording
const uint8_t TRIGGER_PIN = 2;   // Informs CC that FC has data available for recording.
