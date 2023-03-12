/*
 Interface the lunatic data exchange with slot A1 arm nano.

 Dr. Orion Lawlor, lawlor@alaska.edu, 2023-01-25 (Public Domain)
*/
#define NANOSLOT_MY_ID 0xF1 /* my numeric slot ID */
#define NANOSLOT_MY_EX nano.slot_F1  /* my exchange struct */
#include "aurora/lunatic.h"
#include "nanoslot/nanoboot_handoff.h"
#include "nanoslot/nanoslot_IMU_filter.h"
#include "nanoslot/FusionAhrs.cpp"

const int delayMs=30; // set filtering loop speed (milliseconds)
int printCount=0;
int printInterval=30;

nanoslot_IMU_filter frame_filter(delayMs);
nanoslot_IMU_filter boom_filter(delayMs);
nanoslot_IMU_filter fork_filter(delayMs);
nanoslot_IMU_filter dump_filter(delayMs);

int main(int argc,char **argv)
{
    nanoslot_lunatic c(&argc,&argv);

#define ST c.my_state /* shorter name for my state variables */
    // TESTING: initialize frame state to rotated
    frame_filter.set(ST.frame,FusionQuaternionZdegrees(170.0f));    
    
    while (c.is_connected) {
        // Receive data from Arduino
        A_packet p;
        if (c.read_packet(p)) {
            c.handle_standard_packet(p);

            if (c.got_sensor) 
            {
                //frame_filter.update_base(ST.frame, c.my_sensor.imu[0]);
                boom_filter.update_parent(ST.boom, c.my_sensor.imu[1],ST.frame);
                fork_filter.update_parent(ST.fork, c.my_sensor.imu[2],ST.frame);
                dump_filter.update_parent(ST.dump, c.my_sensor.imu[3],ST.frame);
                
                if (printCount++ >=printInterval)
                {
                    printCount=0;
                    printf("   F1: ");
                    if (1) { // filtered IMU data
                        ST.frame.print("\n      frame");
                        ST.boom.print("\n      boom");
                        ST.fork.print("\n      fork");
                        ST.dump.print("\n      dump");
                        printf("\n      ");
                    }
                    
                    if (1) { // raw IMU data
                        for (int i=0;i<NANOSLOT_SENSOR_MY::n_imu;i++)
                        {
                            c.my_sensor.imu[i].acc.print("  acc ");
                            c.my_sensor.imu[i].gyro.print(" gyro ");
                        }
                    }
                    printf("\n");
                    fflush(stdout);
                }
            }
            
            if (c.need_command)
            {
                c.send_command(c.my_command);
            }
        }
        
        // Limit this loop speed to this many milliseconds (varies by what's attached)
        data_exchange_sleep(delayMs);
    }
    
    return 0;
}

