/* Project Monoco Hybrid Monocopter Flight Controller Software

    By: Low Jun En <junen_low@sutd.edu.sg>
    Date Created: 23/08/2016
    License: MIT

    Companion Computer for hybrid monocopter capable of fixed wing and rotor flight.

    Version: 0.2.1
    Date Modified: 23/08/2016

    Hardware Setup:

    SD Logger ------- CC
    3V  ----------- 3.3V
    GND ------------ GND
    CLK ------------- 13
    DO  ------------- 12
    DI  ------------- 11
    CS  ------------- 10

    FC  ------------- CC
    0   -------------- 1
    1   -------------- 0
    GND -------------- GND

    Notifiers ------- CC
    Buzzer ---------- 19
    Status LED ------ 18
    SD LED I -------- 15
    SD LED II ------- 14


    Available Pins
    2,3,4,5,6,7,8,9,16,17,20,21,22,23
*/

#define debugger 0      // On/Off the debug prints

#if debugger
#define debugbegin(a)
#define debug(a)
#define debugln(a)
#define debugt(a,b)
#define debugtln(a,b)
#else
#define debugbegin(a)  Serial.begin(a)
#define debug(a)       Serial.print(a)
#define debugln(a)     Serial.println(a)
#define debugt(a,b)    Serial.print(a,b)
#define debugtln(a,b)  Serial.println(a,b)
#endif

// ======================================================================================
// Libraries ============================================================================

// SD Card
#include <SPI.h>
#include "SdFat.h"
#include "FreeStack.h"
#include "UserDataType.h"  // Edit this include file to change data_t.

// Notifiers
#include <TimerOne.h>

// ======================================================================================
char* data_out[]={"Throttle", "Pitch", "Roll","Motor Left", "Motor Right","LFlap","RFlap","mx","my","mz","K_Mono","Mono_Trim"};
