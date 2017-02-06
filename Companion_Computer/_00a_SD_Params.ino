// Define the SD card parameters.
#define FILE_BASE_NAME "LOG_"
#define error(msg) sd.errorHalt(F(msg))

SdFatSdio sd;
SdFile file;

boolean record = false;                // State of recording. It should only ever be not recording when the FC is disconnected.
float buff[5000];                      // Set an excessive buffer in case the Teensy misses/is unable to execute one write loop.
uint16_t charon = 0;                   // Keeps count of how much data is being stored in the buffer.
const uint32_t REC_INTERVAL_MS = 1000; // Interval of recording onto SD card in milliseconds.
uint32_t logTime;                      // Next scheduled logging time.





