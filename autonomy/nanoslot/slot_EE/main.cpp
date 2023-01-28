/*
 Example where we interface the data exchange with slot serial comms.

 Dr. Orion Lawlor, lawlor@alaska.edu, 2023-01-25 (Public Domain)
*/
#define NANOSLOT_MY_ID 0xEE
#include "aurora/lunatic.h"
#include "nanoslot/serial_handoff.h"

int main(int argc,char **argv)
{
    nanoslot_comms comm(&argc,&argv);
    MAKE_exchange_nano_net(); 
    NANOSLOT_SENSOR_MY my_sensor={0}; // telemetry / sensor data received from Arduino
    NANOSLOT_COMMAND_MY my_command={0}; // command data sent from PC
    
    while (comm.is_connected) {
        // Receive data from Arduino
        A_packet p;
        if (comm.read_packet(p)) {
            comm.handle_standard_packet(p,my_sensor);

            if (comm.got_sensor) 
            {
                printf("  Arduino latency: %d ms, heartbeat %02x\n", my_sensor.latency, my_sensor.heartbeat);
            }
            
            if (comm.need_command) 
            {
                aurora::nano_net_data nano=exchange_nano_net.read();
                my_command.LED=nano.command[0].speed[0]; //<- HACK: left side drive motor
                comm.send_command(my_command);
            }
        }
        
        // Limit this loop speed to this many milliseconds (varies by what's attached)
        data_exchange_sleep(50);
    }
    
    return 0;
}

