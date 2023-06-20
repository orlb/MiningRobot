/*
 Interface the lunatic data exchange with slot F0 front nano.

 Dr. Orion Lawlor, lawlor@alaska.edu, 2023-02-27 (Public Domain)
*/
#define NANOSLOT_MY_ID 0xF0 /* my numeric slot ID */
#define NANOSLOT_MY_EX nano.slot_F0  /* my exchange struct */
#include "aurora/lunatic.h"
#include "nanoslot/nanoboot_handoff.h"

int main(int argc,char **argv)
{
    nanoslot_lunatic comm(&argc,&argv);
    
    int printcount=0;
    
    float filter_old=4.0f; // filtered cell voltage (avoid analogRead noise)
    float filter_percent=0.01f; // percent of new value to blend in at each step
    
    while (comm.is_connected) {
        // Receive data from Arduino
        A_packet p;
        if (comm.read_packet(p)) {
            comm.handle_standard_packet(p,comm.my_sensor);

            if (comm.got_sensor) 
            {
            /*
                if (comm.my_sensor.stop) {
                    printf(" F0 STOP requested\n");
                    fflush(stdout);
                }
            */
                const float voltScale=5.0*(1.0/1023);
                float cell1=voltScale*(comm.my_sensor.cell1);
                
                // Filter out temporal noise
                float filter=filter_old*(1.0f-filter_percent)+cell1*filter_percent;
                filter_old=filter;
                
                const float bias=0.32; // Arduino analogRead voltage offset
                comm.my_state.cell=filter-bias;
                
                const float cell80=3.25; // cell voltage at 80% state of charge
                const float cell20=2.85; // cell voltage at 20% state of charge (measured)
                comm.my_state.charge=20.0f+(comm.my_state.cell-cell20)*(60.0f/(cell80-cell20));
                
                if (printcount--<0) {
                    printf("   F0 driving: %.2fV filtered, %.2fV cell1\n",
                        filter-bias, cell1-bias);
                    fflush(stdout);
                    printcount=50;
                }
            }
            
            if (comm.lunatic_post_packet(p))
            {
                comm.send_command(comm.my_command);
                if (comm.verbose) {
                    printf("  F0 motors: %3d %3d %3d %3d\n",comm.my_command.motor[0],comm.my_command.motor[1],comm.my_command.motor[2],comm.my_command.motor[3]); fflush(stdout);
                }
            }
        }
        
        // Limit this loop speed to this many milliseconds
        data_exchange_sleep(50);
    }
    
    return 0;
}

