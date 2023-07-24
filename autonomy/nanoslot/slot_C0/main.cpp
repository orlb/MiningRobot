/*
 Interface the lunatic data exchange with slot C0 mining head tool.
 
 Dr. Orion Lawlor, lawlor@alaska.edu, 2023-04-12 (Public Domain)
*/
#define NANOSLOT_MY_ID 0xC0 /* my numeric slot ID */
#define NANOSLOT_MY_EX nano.slot_C0  /* my exchange struct */
#include "aurora/lunatic.h"
#include "nanoslot/nanoboot_handoff.h"

int main(int argc,char **argv)
{
    nanoslot_lunatic comm(&argc,&argv);
    
    const int delay_ms=30;
    nanoslot_counter_t last_spin=0;
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
                nanoslot_counter_t cur=comm.my_sensor.spincount;
                nanoslot_counter_t diff = cur - last_spin;
                last_spin=cur;
                comm.my_state.spin = diff * (1.0f / (0.001f * delay_ms));
                
                const float voltScale=4.6*(1.0/1023);
                float cell0=voltScale*(comm.my_sensor.cell0);
                float cell1=voltScale*(comm.my_sensor.cell1);
                comm.my_state.load=cell0;
                
                // Filter out temporal noise
                float filter=filter_old*(1.0f-filter_percent)+cell1*filter_percent;
                filter_old=filter;
                
                const float bias=0.0; // Arduino analogRead voltage offset
                comm.my_state.cell=filter-bias; // cell1-cell0;
                const float cell80=4.02; // cell voltage at 80% state of charge
                const float cell20=3.73; // cell voltage at 20% state of charge
                comm.my_state.charge=20.0f+(filter-bias-cell20)*(60.0f/(cell80-cell20));
                
                if (printcount--<0) {
                    printf("   C0 mining: %.2fV filtered, %.2fV cell1, %.2fV cell0, spin %d\n",
                        filter-bias, cell1-bias, cell0,
                        (int)cur);
                    fflush(stdout);
                    printcount=50;
                }
            }
            
            if (comm.lunatic_post_packet(p))
            {
                comm.send_command(comm.my_command);
                if (comm.verbose) {
                    printf("  C0 mining motor: %3d\n",comm.my_command.mine); fflush(stdout);
                }
            }
        }
        
        // Limit this loop speed to this many milliseconds (varies by what's attached)
        data_exchange_sleep(delay_ms);
    }
    
    return 0;
}

