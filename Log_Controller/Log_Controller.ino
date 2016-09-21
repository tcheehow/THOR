/* Project Monoco Hybrid Monocopter Flight Controller Software

    By: Low Jun En <junen_low@sutd.edu.sg>
    Date Created: 23/08/2016
    License: MIT

    Flight Controller for hybrid monocopter capable of fixed wing and rotor flight.

    Version: 0.2.1
    Date Modified: 07/09/2016

    Hardware Setup:
    Tindie ---------- FC
    3.3V  ----------- 3.3V
    GND ------------- GND
    SDA ------------- 17
    SCL  ------------ 16

    CC  ------------- FC
    0   -------------- 1
    1   -------------- 0
    GND -------------- GND

    Motors/Servos ------------ FC
    LMotor PWM --------------- 20
    RMotor PWM --------------- 21
    LServo PWM --------------- 22
    RServo PWM --------------- 23
    Motor/Servo Power -------- ESC/UBEC

    Receiver ----------------- FC
    8ch_PPM_Out  ------------- 9
    5V ----------------------- 5V (UBEC)
    GND ---------------------- GND (UBEC)

    Notifiers ------- FC
    Status LED ------ 2

    Available Pins
    2,3,4,5,6,7,8,10,11,12,13,14,15,18,19
*/

#define debugger 1      // On/Off the debug prints

#if debugger
#define debugbegin(a)  Serial.begin(a)
#define debug(a)       Serial.print(a)
#define debugln(a)     Serial.println(a)
#define debugt(a,b)    Serial.print(a,b)
#define debugtln(a,b)  Serial.println(a,b)
#else
#define debugbegin(a)  Serial1.begin(a)
#define debug(a)       Serial.print(a)
#define debugln(a)     Serial.println(a)
#define debugt(a,b)    Serial.print(a,b)
#define debugtln(a,b)  Serial.println(a,b)
#define debugw(a)      Serial1.write(a)
#define debugtw(a,b)   Serial1.write(a,b)
#endif

// ======================================================================================
// Libraries ============================================================================

// Motor/Servos
#include <Servo.h>
#include <PulsePosition.h>
#include <math.h>

// Sensors
#include <i2c_t3.h>

// Notifiers
#include <TimerOne.h>


// ======================================================================================
// Tuning Section

#define sens_calib true
#define rc_calib false

// ======================================================================================
// Logging Section

#include <SPI.h>
#include "SdFat.h"
#include "FreeStack.h"
#include "UserDataType.h"  // Edit this include file to change data_t.
//------------------------------------------------------------------------------

char* data_out[]={"mx", "my", "mz","gx", "gy","gz","ax","ay","az"};
