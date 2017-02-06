/* Project Monoco Hybrid Monocopter Flight Controller Software

    By: Low Jun En <junen_low@sutd.edu.sg>
    Date Created: 23/08/2016
    License: MIT

    Flight Controller for hybrid monocopter capable of fixed wing and rotor flight.

    Version: 0.2.1
    Date Modified: 07/11/2016

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
#define debug(a)       Serial1.write(a)
#define debugln(a)
#define debugt(a,b)
#define debugtln(a,b)
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

#define sens_calib false
#define rc_calib false

//char* data_out[]={"Throttle", "Pitch", "Roll","Motor Left", "Motor Right","LFlap","RFlap","mx","my","mz","K_Mono","Mono_Trim"};
