/*
 Interface the lunatic data exchange with slot F1 front nano.

 Dr. Orion Lawlor, lawlor@alaska.edu, 2023-03-07 (Public Domain)
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

/* The vec3 here are hardware offset values, collected with autonomy/kinematics/IMU_calibrate
 The accelerometer values are collected in reference orientation, might be off a degree or two.
*/
nanoslot_IMU_filter frame_filter(delayMs, vec3(0.0722,0.0306,-0.0191),vec3(-2.0,1.9971,-1.0437));
nanoslot_IMU_filter boom_filter(delayMs, vec3(0.052,0.09,0.08),vec3(4.1016,0.8854,-5.1388));
nanoslot_IMU_filter fork_filter(delayMs, vec3(-0.0494,-0.0589,-0.0449),vec3(-0.9815,-6.3307,0.6161));
nanoslot_IMU_filter dump_filter(delayMs, vec3(0.0045,0.0225,0.0816),vec3(-21.1348,4.0780,6.3213));


int main(int argc,char **argv)
{
    nanoslot_lunatic c(&argc,&argv);

#define ST c.my_state /* shorter name for my state variables */ 
    
    while (c.is_connected) {
        // Receive data from Arduino
        A_packet p;
        if (c.read_packet(p)) {
            c.handle_standard_packet(p,c.my_sensor);

            if (c.got_sensor) 
            {
                frame_filter.update_base(ST.frame, 
                    fix_coords_side(c.my_sensor.imu[0]));
                boom_filter.update_parent(ST.boom, 
                    fix_coords_front(c.my_sensor.imu[1]),ST.frame);
                fork_filter.update_parent(ST.fork, 
                    fix_coords_side(c.my_sensor.imu[2]),ST.frame);
                dump_filter.update_parent(ST.dump, 
                    fix_coords_front(c.my_sensor.imu[3]),ST.frame);
                
                ST.load_L = HX711_read_scale(c.my_sensor.load_L,0.0f);
                ST.load_R = HX711_read_scale(c.my_sensor.load_R,-2.9f);
                
                if (printCount++ >=printInterval)
                {
                    printCount=0;
                    printf("   F1:  load cell LR %.1f %.1f", ST.load_L, ST.load_R);
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
            
            if (c.lunatic_post_packet(p)) { // Arduino needs command
                c.send_command(c.my_command);
            }
        }
        
        // Limit this loop speed to this many milliseconds (varies by what's attached)
        data_exchange_sleep(delayMs);
    }
    
    return 0;
}

