/*
 Interface the lunatic data exchange with slot example.

 Dr. Orion Lawlor, lawlor@alaska.edu, 2023-01-25 (Public Domain)
*/
#define NANOSLOT_MY_ID 0xEE
#include "aurora/lunatic.h"
#include "nanoslot/serial_handoff.h"

int main(int argc,char **argv)
{
    nanoslot_comms comm(&argc,&argv);
    MAKE_exchange_nano_net(); 
    NANOSLOT_SENSOR_MY my_sensor={0};
    NANOSLOT_COMMAND_MY my_command={0};
    
    while (true) {
        bool send=false;
        
        // Receive data from Arduino
        A_packet p;
        while (-1==comm.pkt.read_packet(p)) {}
        if (p.valid) {
            if (p.command==NANOSLOT_A_ID) { // ID response
                comm.check_ID(p);
                send=true;
            }
            else if (p.command==NANOSLOT_A_SENSOR) {
                p.get(my_sensor);
                printf("  Arduino latency: %d ms, heartbeat %02x\n", my_sensor.latency, my_sensor.heartbeat); 
                send=true;
            }
            else 
                comm.handle_standard_packet(p);
        }
        
        if (send) 
        {
            aurora::nano_net_data nano=exchange_nano_net.read();
            my_command.LED=nano.command[0].speed[0]; //<- HACK: left side drive motor
            comm.pkt.write_packet(NANOSLOT_A_COMMAND,sizeof(my_command),&my_command);
        }
        
        // Limit this loop speed to this many milliseconds (varies by what's attached)
        data_exchange_sleep(50);
    }
    
}

