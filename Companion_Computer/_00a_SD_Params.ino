// Define the SD card parameters.
#define FILE_BASE_NAME "LOG_"
#define error(msg) sd.errorHalt(F(msg))

SdFatSdio sd;
SdFile file;

const uint16_t BUFF_SIZE = 4096;        // Sdio seems to be able to write at a respectable 463 kB/sec with this limit.
float buff[BUFF_SIZE];                 // Note: Actual buffer is BUFF_SIZE * 4 since floating.
boolean record = false;                // State of recording. It should only ever be not recording when the FC is disconnected.
uint16_t charon = 0;                   // Keeps count of how much data is being stored in the buffer.





