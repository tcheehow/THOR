/* Project Monoco Hybrid Monocopter Flight Controller Software

    By: Low Jun En <junen_low@sutd.edu.sg>
    Date Created: 14/11/2016
    License: MIT

    Companion Computer for hybrid monocopter capable of fixed wing and rotor flight.

    Version: 0.2.1
    Date Modified: 14/11/2016

    Hardware Setup:

    FC  ------------- CC
    0   -------------- 1
    1   -------------- 0
    GND -------------- GND

    WiFi ------------ CC
    ESP TX ---------- 9
    ESP RX ---------- 10

    Notifiers ------- CC
    Status LED ------ 18
    SD LED I -------- 15
    SD LED II ------- 14
    
*/

// ======================================================================================
// Libraries ============================================================================

// SD Card
#include <SPI.h>
#include "SdFat.h"

// WiFi Chip


// Notifiers
