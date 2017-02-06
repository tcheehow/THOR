// ===========================================================================================================
//  SD Card Reader ===========================================================================================

const bool useSharedSpi = false;  // In case of shared SPI bus

uint32_t startMicros;  // File start time in microseconds

bool rec_check = true;

float pool[ADC_DIM];

// Suddenly this section needs to be placed here instead of in 2b_Functions... weird... 
void acquireData(data_t* data, float pool[]) {
  data->time = micros();
  for (int i = 0; i < ADC_DIM; i++) {
    data->adc[i] = pool[i];
  }
}

// Print a data record.
void printData(Print* pr, data_t* data) {
  pr->print(data->time - startMicros);
  for (int i = 0; i < ADC_DIM; i++) {
    pr->write(',');
    pr->print(data->adc[i]);
  }
  pr->println();
}

// Print data header.
void printHeader(Print* pr) {
  pr->print(F("time"));
  for (int i = 0; i < ADC_DIM; i++) {
 //   pr->print(F(",adc"));
    pr->print(data_out[i]);
  }
  pr->println();
}
// ===========================================

const uint32_t LOG_INTERVAL_USEC = 10000;

const uint8_t SD_CS_PIN = SS;      // pin allocation is done in SdFat library

#undef ERROR_LED_PIN               // the lib apparently assigns a pin to an LED... so we are removing thatF
const int8_t ERROR_LED_PIN = -1;    // deactivates said LED

const uint32_t FILE_BLOCK_COUNT = 256000;  // suspect it's just an arbitrary max file size in block units (each block being 512)

#define FILE_BASE_NAME "data"

// With the Teensy 3.1/3.2, SRAM_L: 0x1FFF_8000 – 0x1FFF_FFFF and SRAM_U: 0x2000_0000 – 0x2000_7FFF. Still not sure how he got the addresses below.
#ifndef RAMEND
const uint8_t BUFFER_BLOCK_COUNT = 8;
//
#elif RAMEND < 0X8FF
#error Too little SRAM
//
#elif RAMEND < 0X10FF
// Use total of two 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 1;
//
#elif RAMEND < 0X20FF
// Use total of five 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 4;
//
#else  // RAMEND
// Use total of 13 512 byte buffers.
const uint8_t BUFFER_BLOCK_COUNT = 12;
#endif  // RAMEND
//

#define TMP_FILE_NAME "tmp_log.bin"

const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;

SdFat sd;

SdBaseFile binFile;

char binName[13] = FILE_BASE_NAME "00.bin";

const uint16_t DATA_DIM = (512 - 4) / sizeof(data_t);
const uint16_t FILL_DIM = 512 - 4 - DATA_DIM * sizeof(data_t);

struct block_t {
  uint16_t count;
  uint16_t overrun;
  data_t data[DATA_DIM];
  uint8_t fill[FILL_DIM];
};

const uint8_t QUEUE_DIM = BUFFER_BLOCK_COUNT + 2;

block_t* emptyQueue[QUEUE_DIM];
uint8_t emptyHead;
uint8_t emptyTail;

block_t* fullQueue[QUEUE_DIM];
uint8_t fullHead;
uint8_t fullTail;

inline uint8_t queueNext(uint8_t ht) {
  return ht < (QUEUE_DIM - 1) ? ht + 1 : 0;
}

