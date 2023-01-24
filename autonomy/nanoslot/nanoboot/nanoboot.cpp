/*
 Nanoboot: open an Arduino serial port, get the device ID, 
 hand off to a slot handler.
*/
#include <iostream>
#include <stdio.h>

#include "nanoslot/serial_handoff.h"

#include "nanoslot/nanoslot_exchange.h"
nanoslot_command_0xA0 my_command;


bool talk_to_device(const char *device)
{
    nanoslot_comms comm(device);
    
    int send_wait=0; // cycles to wait for next send attempt
    int fail_count=0;
    int weird_count=0;
    while (true) {
        // Consider sending an ID query packet
        if ((--send_wait)<0) 
        {
            send_wait=5;
            comm.pkt.write_packet(0x1,0,0); // packet type 1: ID query
        }
        
        // Receive data back from Arduino
        A_packet p;
        while (-1==comm.pkt.read_packet(p)) {}
        if (p.valid) {
            if (p.command==NANOSLOT_A_ID) { // ID response
                int ID=p.data[0];
                int sane=p.data[3];
                if (p.length==4 && ID!=0 && sane==NANOSLOT_ID_SANITY) 
                { // looks like a valid ID
                    printf("  Valid ID %02X %02X %02X\n", ID,p.data[1],p.data[2]);
                    
                    comm.pkt.write_packet(NANOSLOT_A_COMMAND,sizeof(my_command),&my_command);
                    // FIXME: handoff to slot handler now
                    //exit(0);
                } else {
                    printf("  Invalid ID packet: %02X %02X\n", ID, sane);
                    weird_count++;
                }
            }
            else if (p.command==NANOSLOT_A_SENSOR) {
                printf("  Device %s sensor: %02x\n",
                    device,p.data[0]); 
                comm.pkt.write_packet(NANOSLOT_A_COMMAND,sizeof(my_command),&my_command);
            }
            else if (p.command==NANOSLOT_A_DEBUG) {
                printf("  Device %s debug 0xD: %s\n",
                    device,(char *)p.data);
                exit(1); //<- just stop if firmware hits errors.
            }
            else if (p.command==NANOSLOT_A_ERROR) {
                printf("  Device %s hit error 0xE: %s\n",
                    device,(char *)p.data);
                exit(1); //<- just stop if firmware hits errors.
            }
            else {
                printf("  Device %s sent unknown packet type %02x / length %d\n",
                    device,p.command,p.length);
                weird_count++;
            }
        }
        else fail_count++;
        
        if (fail_count>200) {
            printf("  Device %s: too many failures\n",
                    device);
            return true; //<- disconnect?
        }
        if (weird_count>20) {
            printf("  Device %s: too many weird packets\n",
                    device);
            return false; //<- not the right serial port?
        }
        
        // Limit this loop to 200Hz (5ms/loop)
        data_exchange_sleep(5);
    }
    
}

int main(int argc,char **argv) 
{
    if (argc<=1) {
        fprintf(stderr,"Usage: nanoboot <device name>\n");
        exit(1);
    }
    
    talk_to_device(argv[1]);
}


