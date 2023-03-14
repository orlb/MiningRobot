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


/**
 Fix coordinate system of raw IMU data, with MPU-6050 mounted 
 on the side of a frame sidebar. 
   Incoming for frame IMU:
     +X down -> -Z out
     +Y forwards -> +Y out
     +Z right -> +X out
*/
nanoslot_vec3_t fix_coords_side(const nanoslot_vec3_t &src)
{
    nanoslot_vec3_t ret;
    ret.type=src.type;
    ret.x=src.z;
    ret.y=src.y;
    ret.z=-src.x;
    return ret;
}

// Fix both the accelerometer and gyro coordinates (the same way)
nanoslot_IMU_t fix_coords_side(const nanoslot_IMU_t &src)
{
    nanoslot_IMU_t ret;
    ret.acc = fix_coords_side(src.acc);
    ret.gyro = fix_coords_side(src.gyro);
    return ret;
}

/**
 Fix coordinate system of raw IMU data, with MPU-6050 mounted 
 on the front of a frame member. 
   Incoming for dump IMU:
     +Y down -> -Z out
     +X left -> -X out
     +Z back -> -Y out
*/
nanoslot_vec3_t fix_coords_front(const nanoslot_vec3_t &src)
{
    nanoslot_vec3_t ret;
    ret.type=src.type;
    ret.x=-src.x;
    ret.y=-src.z;
    ret.z=-src.y;
    return ret;
}

// Fix both the accelerometer and gyro coordinates (the same way)
nanoslot_IMU_t fix_coords_front(const nanoslot_IMU_t &src)
{
    nanoslot_IMU_t ret;
    ret.acc = fix_coords_front(src.acc);
    ret.gyro = fix_coords_front(src.gyro);
    return ret;
}

int main(int argc,char **argv)
{
    nanoslot_lunatic c(&argc,&argv);

#define ST c.my_state /* shorter name for my state variables */ 
    
    while (c.is_connected) {
        // Receive data from Arduino
        A_packet p;
        if (c.read_packet(p)) {
            c.handle_standard_packet(p);

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

