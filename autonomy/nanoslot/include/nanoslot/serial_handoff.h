/*
 Utility functions for PC-side serial port handling.

 Dr. Orion Lawlor, lawlor@alaska.edu, 2023-01-22 (Public Domain)
*/
#ifndef NANOSLOT_SERIAL_HANDOFF_H
#define NANOSLOT_SERIAL_HANDOFF_H 1

#include <string>
#include <stdlib.h>

#include "config.h"
#include "sleep.h"
#include "serial.cpp"
#include "A_packet.h"


/** A nanoslot_comms manages communication with one Arduino */
class nanoslot_comms {
public:
    /// This is used to send/receive Arduino packets
    A_packet_formatter<SerialPort> pkt;
    
    /// Set up communications with an existing serial port opened by nanoboot,
    ///   or a new serial port as specified on the command line (like "--dev /dev/ttyUSB0")
    nanoslot_comms(int *argc,char ***argv)
        :pkt(Serial)
    {
        if (getenv(NANOSLOT_ENV)!=NULL) 
        { // hand off already opened serial port
            // FIXME: handle this case, maybe Serial.Open(fd)?
        }
        else if (*argc>2 && 0==strcmp("--dev",(*argv)[1]))
        { // command line case (used for development and testing)
            set_up_serial((*argv)[2]);
            
            // Move arguments down (skip over the --dev and device name).
            *argv+=2;
            *argc-=2;
        }
    }


    /// Set up communications with this serial port (like "/dev/ttyUSB0")
    /// Used by nanoboot.
    nanoslot_comms(const std::string &serial_port) 
        :pkt(Serial)
    {
        set_up_serial(serial_port);
    }
    
    bool set_up_serial(const std::string &serial_port) {
        Serial.Open(serial_port);
        Serial.Set_baud(NANOSLOT_BAUD_RATE);
        if(Serial.Is_open())
        {
            std::cout << "  Opened "<<serial_port<<std::endl; 
            data_exchange_sleep(NANOSLOT_BOOTLOADER_DELAY_MS); // wait through bootloader (which will hang if you immediately start sending it data)
            return true;
        }
        else 
        {
            std::cout << "  Can't open serial port "<<serial_port<<"\n";
            return false;
        }
    }
};




#endif


