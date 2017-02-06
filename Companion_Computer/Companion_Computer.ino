/* Project Monoco Hybrid Monocopter Companion Computer Software

    By: Low Jun En <junen_low@sutd.edu.sg>
    Date Created: 30/01/2017
    License: MIT

    Companion Computer (CC) for hybrid monocopter capable of fixed wing and rotor flight. 
    The purpose of the CC is to log data from the main flight controller during experiments.
    It also serves as the processing unit for any non-crucial operations for flight like
    LED pattern displays and parameter tuning.

    Version: 1.0.1
    Date Modified: 30/01/2017

    Hardware Setup:
    
    CC   ============ FC
    0    ------------- 1
    1    ------------- 0
    2    ------------- 2

    CC   ============ LEDs
    13   ------------ GREEN (+ve)
    GND  ------------ GREEN (-ve)

    Stuff to Implement
    * A more advanced comm_manager that can report urgent data faults and save logs at intervals.
    * Some EOL data detection in the event that bytes within a single data packet are corrupt.
    * Remove that damned comma at the end of the header string.
    * Switch the Serial1 reads to be check via ISR instead perhaps?
    * Find out why it takes forever for the header to be setup.
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
#include "SdFat.h"






