/*
 Interface the lunatic data exchange with slot A1 arm nano.

 Dr. Orion Lawlor, lawlor@alaska.edu, 2023-01-25 (Public Domain)
*/
#define NANOSLOT_MY_ID 0xA1 /* my numeric slot ID */
#define NANOSLOT_MY_EX nano.slot_A1  /* my exchange struct */
#include "aurora/lunatic.h"
#include "nanoslot/nanoboot_handoff.h"
#include "nanoslot/nanoslot_IMU_filter.h"
#include "nanoslot/FusionAhrs.cpp"

const int delayMs=30; // set filtering loop speed (milliseconds)
int printCount=0;
int printInterval=30;

nanoslot_IMU_filter stick_filter(delayMs);
nanoslot_IMU_filter tool_filter(delayMs);

/**
 Fix coordinate system of raw IMU data, with MPU-6050 mounted underneath a crossbar:
   Incoming for stick IMU:
     +X behind stick -> -Y out
     +Y across stick -> +X out
     +Z above stick -> +Z out
   Tool is same but flip sign on XY.
   
   sign=+1: underneath, pins facing robot forwards
   sign=-1: underneath, pins facing backwards
*/
nanoslot_vec3_t fix_coords_cross(const nanoslot_vec3_t &src,int sign=+1)
{
    nanoslot_vec3_t ret;
    ret.type=src.type;
    ret.x=sign*src.y;
    ret.y=-sign*src.x;
    ret.z=src.z;
    return ret;
}

// Fix both the accelerometer and gyro coordinates (the same way)
nanoslot_IMU_t fix_coords_cross(const nanoslot_IMU_t &src,int sign=+1)
{
    nanoslot_IMU_t ret;
    ret.acc = fix_coords_cross(src.acc,sign);
    ret.gyro = fix_coords_cross(src.gyro,sign);
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
                // Grab boom orientation from the exchange:
                const nanoslot_exchange &nano=c.exchange_nanoslot.read();
                stick_filter.update_parent(ST.stick, 
                    fix_coords_cross(c.my_sensor.imu[1]),nano.slot_F1.state.boom);
                
                tool_filter.update_parent(ST.tool, 
                    fix_coords_cross(c.my_sensor.imu[0],-1),ST.stick);
                
                if (printCount++ >=printInterval)
                {
                    printCount=0;
                    printf("   A1: ");
                    if (1) { // print filtered IMU data
                        ST.stick.print("\n      stick");
                        ST.tool.print("\n      tool");
                        printf("\n      ");
                    }
                    if (1) { 
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
        data_exchange_sleep(50);
    }
    
    return 0;
}

