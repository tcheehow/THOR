// ===================================== MS5637-02BA03 Registers =====================================
// ==================== http://www.te.com/usa-en/product-CAT-BLPS0037.html ===========================

#define MS5637_RESET      0x1E
#define MS5637_CONVERT_D1 0x40
#define MS5637_CONVERT_D2 0x50
#define MS5637_ADC_READ   0x00

// ===================================== ITG3701 Gyro Registers ========================================
// =============== https://store.invensense.com/datasheets/invensense/PS-ITG-3701.pdf ==================

#define ITG3701_XG_OFFS_TC_H     0x04
#define ITG3701_XG_OFFS_TC_L     0x05
#define ITG3701_YG_OFFS_TC_H     0x07
#define ITG3701_YG_OFFS_TC_L     0x08
#define ITG3701_ZG_OFFS_TC_H     0x0A
#define ITG3701_ZG_OFFS_TC_L     0x0B
#define ITG3701_XG_OFFS_USRH     0x13  // User-defined trim values for gyroscope
#define ITG3701_XG_OFFS_USRL     0x14
#define ITG3701_YG_OFFS_USRH     0x15
#define ITG3701_YG_OFFS_USRL     0x16
#define ITG3701_ZG_OFFS_USRH     0x17
#define ITG3701_ZG_OFFS_USRL     0x18
#define ITG3701_SMPLRT_DIV       0x19
#define ITG3701_CONFIG           0x1A
#define ITG3701_GYRO_CONFIG      0x1B
#define ITG3701_FIFO_EN          0x23
#define ITG3701_INT_PIN_CFG      0x37
#define ITG3701_INT_ENABLE       0x38
#define ITG3701_INT_STATUS       0x3A
#define ITG3701_TEMP_OUT_H       0x41
#define ITG3701_TEMP_OUT_L       0x42
#define ITG3701_GYRO_XOUT_H      0x43
#define ITG3701_GYRO_XOUT_L      0x44
#define ITG3701_GYRO_YOUT_H      0x45
#define ITG3701_GYRO_YOUT_L      0x46
#define ITG3701_GYRO_ZOUT_H      0x47
#define ITG3701_GYRO_ZOUT_L      0x48
#define ITG3701_USER_CTRL        0x6A
#define ITG3701_PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define ITG3701_PWR_MGMT_2       0x6C
#define ITG3701_FIFO_COUNTH      0x72
#define ITG3701_FIFO_COUNTL      0x73
#define ITG3701_FIFO_R_W         0x74
#define ITG3701_WHO_AM_I         0x75 // Should return 0x68

// ==================================== LSM303D Accel/Magneto (XM) Registers ========================================
// ====== http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00057547.pdf =========

#define  LSM303D_OUT_TEMP_L_XM   0x05
#define  LSM303D_OUT_TEMP_H_XM   0x06
#define  LSM303D_STATUS_REG_M    0x07
#define  LSM303D_OUT_X_L_M       0x08
#define  LSM303D_OUT_X_H_M       0x09
#define  LSM303D_OUT_Y_L_M       0x0A
#define  LSM303D_OUT_Y_H_M       0x0B
#define  LSM303D_OUT_Z_L_M       0x0C
#define  LSM303D_OUT_Z_H_M       0x0D
#define  LSM303D_WHO_AM_I_XM     0x0F
#define  LSM303D_INT_CTRL_REG_M  0x12
#define  LSM303D_INT_SRC_REG_M   0x13
#define  LSM303D_INT_THS_L_M     0x14
#define  LSM303D_INT_THS_H_M     0x15
#define  LSM303D_OFFSET_X_L_M    0x16
#define  LSM303D_OFFSET_X_H_M    0x17
#define  LSM303D_OFFSET_Y_L_M    0x18
#define  LSM303D_OFFSET_Y_H_M    0x19
#define  LSM303D_OFFSET_Z_L_M    0x1A
#define  LSM303D_OFFSET_Z_H_M    0x1B
#define  LSM303D_REFERENCE_X     0x1C
#define  LSM303D_REFERENCE_Y     0x1D
#define  LSM303D_REFERENCE_Z     0x1E
#define  LSM303D_CTRL_REG0_XM    0x1F
#define  LSM303D_CTRL_REG1_XM    0x20
#define  LSM303D_CTRL_REG2_XM    0x21
#define  LSM303D_CTRL_REG3_XM    0x22
#define  LSM303D_CTRL_REG4_XM    0x23
#define  LSM303D_CTRL_REG5_XM    0x24
#define  LSM303D_CTRL_REG6_XM    0x25
#define  LSM303D_CTRL_REG7_XM    0x26
#define  LSM303D_STATUS_REG_A    0x27
#define  LSM303D_OUT_X_L_A       0x28
#define  LSM303D_OUT_X_H_A       0x29
#define  LSM303D_OUT_Y_L_A       0x2A
#define  LSM303D_OUT_Y_H_A       0x2B
#define  LSM303D_OUT_Z_L_A       0x2C
#define  LSM303D_OUT_Z_H_A       0x2D
#define  LSM303D_FIFO_CTRL_REG   0x2E
#define  LSM303D_FIFO_SRC_REG    0x2F
#define  LSM303D_INT_GEN_1_REG   0x30
#define  LSM303D_INT_GEN_1_SRC   0x31
#define  LSM303D_INT_GEN_1_THS   0x32
#define  LSM303D_INT_GEN_1_DURATION 0x33
#define  LSM303D_INT_GEN_2_REG   0x34
#define  LSM303D_INT_GEN_2_SRC   0x35
#define  LSM303D_INT_GEN_2_THS   0x36
#define  LSM303D_INT_GEN_2_DURATION 0x37
#define  LSM303D_CLICK_CFG       0x38
#define  LSM303D_CLICK_SRC       0x39
#define  LSM303D_CLICK_THS       0x3A
#define  LSM303D_TIME_LIMIT      0x3B
#define  LSM303D_TIME_LATENCY    0x3C
#define  LSM303D_TIME_WINDOW     0x3D
#define  LSM303D_ACT_THS         0x3E
#define  LSM303D_ACT_DUR         0x3F

// =================================================================================================================
// Pressure and Temperature Conversion Rates =======================================================================

#define ADC_256  0x00
#define ADC_512  0x02
#define ADC_1024 0x04
#define ADC_2048 0x06
#define ADC_4096 0x08
#define ADC_8192 0x0A
#define ADC_D1   0x40
#define ADC_D2   0x50

// =================================================================================================================

#define ADO 0
#if ADO
#define LSM303D_ADDRESS   0x1D // Address of accel/magnetometer when ADO = 1
#define ITG3701_ADDRESS   0x69 // Address of gyro when ADO = 1
#define MS5637_ADDRESS    0x76 // Address of altimeter
#else
#define LSM303D_ADDRESS   0x1E // Address of accel/magnetometer when ADO = 0
#define ITG3701_ADDRESS   0x68 // Address of gyro when ADO = 0
#define MS5637_ADDRESS    0x76 // Address of altimeter
#endif

