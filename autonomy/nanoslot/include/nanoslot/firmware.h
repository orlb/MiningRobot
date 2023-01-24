/*
 Main firmware functions targeting Arduino microcontrollers, such as
 the Nano 328p or Uno.
 
 This is intended to be compiled in the Arduino IDE version 2.0 or higher.

 Dr. Orion Lawlor, lawlor@alaska.edu, 2023-01-22 (Public Domain)
*/
#ifndef NANOSLOT_FIRMWARE_H
#define NANOSLOT_FIRMWARE_H 1

// Global configuration values
#include "config.h"

// Structs used to exchange values with the PC
#include "nanoslot_exchange.h"
NANOSLOT_COMMAND_MY my_command; // last command received from PC
NANOSLOT_SENSOR_MY my_sensor; // sensor data to send to PC

// We track time via a global named milli
#include "firmware_milli.h"
milli_t milli=0;

/* 
 Update the milli global time counter.
 Subtle: millis() skips a count every so often.  
         micros() doesn't, so we use that instead. 
*/
inline void update_milli()
{
    milli=micros()>>10;
}

// All PC commands go via this (onboard USB) port
#include "A_packet.h" /* for A-packet serial comms */

// Send our ID packet
void send_ID_packet(A_packet_formatter<HardwareSerial> &pkt) {
    nanoslot_byte_t buf[4];
    buf[0]=NANOSLOT_MY_ID;
    buf[1]=sizeof(my_command);
    buf[2]=sizeof(my_sensor);
    buf[3]=NANOSLOT_ID_SANITY;
    pkt.write_packet(NANOSLOT_A_ID,sizeof(buf),buf); // send ID ping packet
}

/** This class manages communication via an A_packet_formatter,
 including timeouts. */
template <class HardwareSerial>
class CommunicationChannel {
public:
  HardwareSerial &backend;
  A_packet_formatter<HardwareSerial> pkt; // packet formatter
  bool is_connected; // 1 if we're recently connected; 0 if no response
  milli_t last_read; // the last time we got data back
  milli_t next_send; // the next time we should send off data

  CommunicationChannel(HardwareSerial &new_backend)
    :backend(new_backend), pkt(backend)
  {
    is_connected=0;
    last_read=milli;
    next_send=milli;
  }

  bool read_packet(A_packet &p) {
    p.valid=0;
    if (backend.available()) {
      while (-1==pkt.read_packet(p)) {}
      if (p.valid) {
        is_connected=true; // got valid packet
        last_read=milli;
        next_send=milli;
        return true; // caller should handle this
      }
    }
    if (milli-next_send>200) { // read timeout
      next_send=milli;
      pkt.reset();
      send_ID_packet(pkt); //<- synchronize with an ID packet
      is_connected=false;
    }
    return false;
  }
};

typedef A_packet_formatter<HardwareSerial> A_packet_serial;
HardwareSerial &PCport=Serial; // direct PC
CommunicationChannel<HardwareSerial> comm(PCport);

// Send error string up to mega (and on to main PC)
void debuglog(const char *why) {
    comm.pkt.write_packet(NANOSLOT_A_DEBUG,strlen(why)+1,why);
}

/* Initialize serial ports */
inline void nanoslot_firmware_start(void) 
{
    PCport.begin(NANOSLOT_BAUD_RATE);
}

// User-written functions:
void firmware_read_encoders(void); ///<- reads encoder / sensor data
bool firmware_handle_custom_packet(A_packet_serial &pkt,A_packet &p); ///<- talk to PC driver. Return false for error.
void firmware_send_motors(void); ///<- write to motors / actuators


/* Respond to standard packet types */
bool firmware_handle_standard_packet(A_packet_serial &pkt,A_packet &p)
{    
    if (p.length==0 && p.command==NANOSLOT_A_ID) 
    { // ID request
       send_ID_packet(pkt);
       return true; // we handled it
    }
    
    if (p.command==NANOSLOT_A_COMMAND)
    {
        if (!p.get(my_command)) 
        {
            debuglog("bad cmd sz");
        }
        pkt.write_packet(NANOSLOT_A_SENSOR,
            sizeof(my_sensor),&my_sensor);
        return true;
    }
    
    return false;
}

/* Run the main firmware loop */
inline void nanoslot_firmware_loop(int delayMs=4) {
    /* Pull encoder data (using fresh timestamp) */
    update_milli();
    milli_t start=milli;
    firmware_read_encoders();

    /* Check serial */
    A_packet p;
    if (comm.read_packet(p))
    {
        if (!firmware_handle_standard_packet(comm.pkt,p)) 
        if (!firmware_handle_custom_packet(comm.pkt,p))
        {
            /* weird to get a request we don't recognize--log it. */
            debuglog("bad cmd type");
        }
    } 

    /* Update motors */
    update_milli();
    firmware_send_motors();
    
    /* Limit the speed of the main loop (mostly for the motors) */
    update_milli();
    milli_t elapsed=milli-start;
    int waitMs=delayMs-(int)elapsed;
    if (waitMs>0)
        delay(waitMs);
}


#endif

