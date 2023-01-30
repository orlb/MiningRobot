/*
 Nanoboot: open an Arduino serial port, get the device ID, 
 hand off to a slot handler.
*/
#include <iostream>
#include <stdio.h>

#include "nanoslot/nanoboot_handoff.h"

#include <unistd.h> /* for exec magic */

#include "nanoslot/nanoslot_exchange.h"


bool talk_to_device(const char *device)
{
    nanoboot_comms comm(device);
    
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
                comm.check_ID(p);
                int ID=p.data[0];
                printf("  Got ID %02X, doing handoff\n", ID);
                
                // Path to the slot program is relative to /nanoslot/dir (our working directory):
                char exe[1000];
                snprintf(exe,sizeof(exe),"slot_%02X/slot_%02X",ID,ID);
                
                fflush(stdout); fflush(stderr);

#if NANOSLOT_HANDOFF_FANCY  
                int fd=Serial.GetFd();
                char fdName[100];
                snprintf(fdName,sizeof(fdName),"%d",fd);
                execlp(exe,  exe,"--fd",fdName,NULL);
#else
                // Simple call where the slot program re-opens the device (two bootloader waits)
                //  FIXME: do fancy handoff here where we leave the serial port open.
                execlp(exe,  exe,"--dev",device,NULL);
#endif
           
                // If we get here, the exec didn't work:
                perror("Error doing exec of slot program");
                int err=system("pwd");
                err=system("echo $LD_LIBRARY_PATH");
                printf("Attempted path to program: %s for device %s\n",exe,device);
                exit(1);
            }
            else if (p.command==NANOSLOT_A_DEBUG) {
                printf("  Device %s debug 0xD: %s\n",
                    device,(char *)p.data);
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


