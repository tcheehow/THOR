/* Project Monoco Hybrid Monocopter Flight Controller Software

    By: Low Jun En <junen_low@sutd.edu.sg>
    Date Created: 30/01/2017
    License: MIT

    Flight Controller for hybrid monocopter capable of fixed wing and rotor flight.

    Version: 1.0.1
    Date Modified: 30/01/2017

    Hardware Setup:

    FC   ============ BNO-055
    3.3V ------------ 3.3V
    GND  ------------ GND
    SDA  ------------ 17
    SCL  ------------ 16

    FC   ============ CC
    0    ------------- 1
    1    ------------- 0
    2    ------------- 2

    FC   ============ LEDs
    13   ------------ RED (+ve)
    GND  ------------ RED (-ve)
    12   ------------ YELLOW (+ve)
    GND  ------------ YELLOW (-ve)

    FC   ============ Receiver
    0    ------------ 8ch_PPM_Out
    Vin  ------------ 5V
    GND  ------------ GND

    Stuff to Implement
    * Kris has an interesting way of converting his data. CC defines union outside of setup
    * and loop. He defines it within function. I wonder which is better for what.

*/

#define debugger 1      // On/Off the debug prints

#if debugger
#define debugbegin(a)  Serial.begin(a)
#define debug(a)       Serial.print(a)
#define debugln(a)     Serial.println(a)
#define debugt(a,b)    Serial.print(a,b)
#define debugtln(a,b)  Serial.println(a,b)
#else
#define debugbegin(a)
#define debug(a)
#define debugln(a)
#define debugt(a,b)
#define debugtln(a,b)
#endif

// Libraries
#include <PulsePosition.h>

