/* 
 Read a load cell using the HX711 library.
 
 For 180kg load cells, wiring order left-to-right from outside:
  B- B+   A- A+        E- E+
         red-green   black-white   Right channel
  red-green          black-white   Left channel
*/
#ifndef __NANOSLOT_FIRMWARE_HX77_H
#define __NANOSLOT_FIRMWARE_HX77_H

#include "HX711.h" /* from https://github.com/bogde/HX711/ */
#include "HX711.cpp" /* easier linking, just add library here */

/*
//pins: plugged on top of Arduino nano (for testing)
const int HX711_gnd = 5; // soft power
const int HX711_dout = 4; //mcu > HX711 dout pin
const int HX711_sck = 3; // mcu > HX711 sck pin
const int HX711_vcc = 2; // soft power
*/

//pins: left subslot in nanoslot
const int HX711_gnd = 7; // (N/C)
const int HX711_dout = 4; //mcu > HX711 dout pin
const int HX711_sck = 2; // mcu > HX711 sck pin
const int HX711_vcc = 3; // soft power

/* Reads data from the HX711 board */
class HX711_reader {
public:
    // Last-read values: about 1/10000x from kilograms
    long readA=0, readB=0;
    
    // If true, we're reading channel A (default, highest res)
    bool use_channelA=true;
    
    // First time setup
    void begin(void) {
        // Apply soft power to the sensor (it only needs 1.6 mA)
        pinMode(HX711_vcc,OUTPUT); digitalWrite(HX711_vcc,HIGH);
        pinMode(HX711_gnd,OUTPUT); digitalWrite(HX711_gnd,LOW);
        scale.begin(HX711_dout,HX711_sck,64);
    }
    
    // Our readA and readB values are updated here
    void read(void) {
        if (scale.is_ready()) {
            scale.set_gain(use_channelA?64:32);
            long v = scale.read();
            
            if (last_channelA) readA=v>>1;
            else readB=v;
            last_channelA=use_channelA;
        } 
    }

private:
    HX711 scale;
    // Subtle: scale remembers last-requested channel
    bool last_channelA=true;
};


#endif


