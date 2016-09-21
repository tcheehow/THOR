#ifndef UserDataType_h
#define UserDataType_h
const uint8_t ADC_DIM = 9;
struct data_t {
  unsigned long time;
  float adc[ADC_DIM];
};

#endif  // UserDataType_h
