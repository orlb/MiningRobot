/*
 Utility functions for PC-side serial port handling.

 Dr. Orion Lawlor, lawlor@alaska.edu, 2023-01-22 (Public Domain)
*/
#ifndef NANOSLOT_SERIAL_HANDOFF_H
#define NANOSLOT_SERIAL_HANDOFF_H 1

#include <iostream>
#include <stdio.h>
#include <string>
#include <stdlib.h>

#include "config.h" // overall nanoslot configuration
#include "sleep.h" // portable sleep
#include "serial.cpp" // talk on serial port
#include "A_packet.h" // format packets on serial port
#include "nanoslot_exchange.h" // data exchanged in A packets


/** A nanoslot_comms manages communication with one Arduino */
class nanoslot_comms {
public:
    /// This is used to send/receive Arduino packets
    A_packet_formatter<SerialPort> pkt;
    int packet_count=0; // valid packets received
    int fail_count=0; // serial receive calls that failed
    int weird_count=0; // serial data with weird packet type

    
    /// Set up communications with an existing serial port opened by nanoboot,
    ///   or a new serial port as specified on the command line (like "--dev /dev/ttyUSB0")
    nanoslot_comms(int *argc,char ***argv)
        :pkt(Serial)
    {
#if NANOSLOT_HANDOFF_FANCY
        /* We were just exec'd by nanoboot, and they already
           opened the serial device (and waited through the bootloader)
           so we can just use their still-open file descriptor. */
        if (*argc>2 && 0==strcmp("--fd",(*argv)[1]))
        { // hand off already opened serial port
            int fd=atoi((*argv)[2]);
            printf("Doing nanoslot serial handoff on fd %d\n",fd);
            Serial.OpenFd(fd);
            Serial.Set_baud(NANOSLOT_BAUD_RATE);
        }
        else 
#endif
        if (*argc>2 && 0==strcmp("--dev",(*argv)[1]))
        { // command line case (used for development and testing)
            set_up_serial((*argv)[2]);
        } 
        else {
            printf("Usage: slotprogram --dev /dev/ttyUSB0\n");
            exit(1);
        }
        
        // We used up two arguments via either startup method:
        *argc -=2;
        *argv +=2; //<- hacky, leaves argv[0] pointing to wrong thing
    }
    

    /// Set up communications with this serial port (like "/dev/ttyUSB0")
    /// Used by nanoboot.
    nanoslot_comms(const std::string &serial_port) 
        :pkt(Serial)
    {
        set_up_serial(serial_port);
    }

protected:
    bool set_up_serial(const std::string &serial_port) {
        Serial.Open(serial_port);
        Serial.Set_baud(NANOSLOT_BAUD_RATE);
        if(Serial.Is_open())
        {
            std::cout << "  Opened "<<serial_port<<std::endl; 
            data_exchange_sleep(NANOSLOT_BOOTLOADER_DELAY_MS); // wait through bootloader (which can hang if you immediately start sending it data)
            return true;
        }
        else 
        {
            std::cout << "  Can't open serial port "<<serial_port<<"\n";
            return false;
        }
    }

public:
#ifdef NANOSLOT_MY_ID
    //  read_packet / handle_standard_packet sets these flags according to what happened.
    bool is_connected=true; ///< If true, we are connected to the Arduino
    bool got_sensor=false; ///< If true, we just got an Arduino sensor data packet
    bool need_command=false; ///< If true, you should send the Arduino a command packet
    
    // Receive serial data from the Arduino.  
    bool read_packet(A_packet &p) {
        got_sensor=false;
        need_command=false;
        
        while (-1==pkt.read_packet(p)) { /* no data yet, keep reading */ }
        if (p.valid) {
            packet_count++;
            fail_count=0; // link is OK
            
            // Give caller a chance to look at this packet.
            //  They'll probably just call handle_standard_packet. 
            return true;
        }
        else { // No valid data, or error getting data
            fail_count++;
            if (fail_count>20) { // probably a disconnect
                is_connected=false;
                printf(" slot %02X arduino disconnect (%d good, %d weird, %d fail)\n",
                    NANOSLOT_MY_ID,packet_count,weird_count,fail_count);
                fflush(stdout);
            }
            return false;
        }
    }

    // Default serial data packet handling:
    //   receive sensor data into struct and set got_sensor
    //   set need_command if the Arduino wants command data
    //   handle normal debug commands
    template <class sensor_t>
    void handle_standard_packet(A_packet &p,sensor_t &sensor)
    {
        if (p.command==NANOSLOT_A_ID) { // ID response
            check_ID(p);
            need_command=true;
        }
        else if (p.command==NANOSLOT_A_SENSOR) { // incoming sensor data
            p.get(sensor);
            got_sensor=true;
            need_command=true;
        }
        else if (p.command==NANOSLOT_A_DEBUG) { // debug command
            printf("  Device debug 0xD: %s\n",
                (char *)p.data);
            fflush(stdout);
        }
        else if (p.command==NANOSLOT_A_ERROR) { // fatal error
            printf("  Device hit error 0xE: %s\n",
                (char *)p.data);
            fflush(stdout);
            exit(1); //<- just stop if firmware hits errors.
        }
        else { // unknown packet type 
            printf("  Got unknown packet type %02x / length %d\n",
                p.command,p.length);
            fflush(stdout);
            weird_count++;
        }
    }
    
    // Send this command to the Arduino now
    template <class command_t>
    void send_command(command_t &command)
    {
        pkt.write_packet(NANOSLOT_A_COMMAND,sizeof(command),&command);
    }
#endif
    
    // Sanity-check this ID packet with our struct sizes.
    //   (exit early and safely if struct sizes don't match)
    void check_ID(A_packet &p)
    {
        expected_value(p.length,4,"ID packet length");
#ifdef NANOSLOT_MY_ID
        expected_value(p.data[0],NANOSLOT_MY_ID,"ID value");
        expected_value(p.data[1],sizeof(NANOSLOT_COMMAND_MY),"command bytes");
        expected_value(p.data[2],sizeof(NANOSLOT_SENSOR_MY),"sensor bytes");
#endif
        expected_value(p.data[3],NANOSLOT_ID_SANITY,"ID packet sanity");
    }
    
    // Sanity-check this value, exit if we got something we didn't expect
    void expected_value(int got,int expected,const char *what) {
        if (got!=expected) {
            printf("  ERROR: %s on Arduino is %d (0x%x), but we wanted %d (0x%x) (mismatch!)\n",
                what, got,got, expected,expected);
            fflush(stdout);
            exit(1);
        }
    }
};




#endif


