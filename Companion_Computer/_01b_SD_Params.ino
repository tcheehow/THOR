// ===========================================================================================================
//  SD Card Reader ===========================================================================================

#define FILE_BASE_NAME "Data"  // Log file base name.  Must be six characters or less.

SdFatSdio sd;
SdFile file;

// Recording Controller
bool rec_check = true;

// Logger Settings
const uint8_t chipSelect = SS;
const uint32_t SAMPLE_INTERVAL_MS = 50;
uint32_t logTime;

// Number of Data 
const uint8_t DATA_COUNT = 6;

//==============================================================================
// Error messages stored in flash.
#define error(msg) sd.errorHalt(F(msg))
