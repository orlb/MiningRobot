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

/* The vec3 here are hardware offset values, collected with autonomy/kinematics/IMU_calibrate
 The accelerometer values are collected in reference orientation, might be off a degree or two, more in hot weather.
*/
nanoslot_IMU_filter stick_filter(delayMs,vec3(-0.0136,0.09,-0.0111),vec3(-1.5821,1.9100,-0.1994),vec3(1,1.1f,1));
nanoslot_IMU_filter tool_filter(delayMs,vec3(-0.0094,0.0073,0.0372),vec3(0.1127,3.3704,-26.7998));

// Tool IMU is rotated 180 degrees around Y axis of stick.
//  Quaternion for 180 degree rotation has W 0 and XYZ = axis of rotation.
FusionQuaternion stick_to_tool_rotate={0.0f, 0.0f,1.0f,0.0f};

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
                // Grab boom orientation from the exchange:
                const nanoslot_exchange &nano=c.exchange_nanoslot.read();
                stick_filter.update_parent(ST.stick, 
                    fix_coords_cross(c.my_sensor.imu[1]),nano.slot_F1.state.boom);
                
                tool_filter.update_parent(ST.tool, 
                    fix_coords_cross(c.my_sensor.imu[0],-1),ST.stick,&stick_to_tool_rotate);
                
                ST.load_L = HX711_read_scale(c.my_sensor.load_L,-6.6f);
                ST.load_R = HX711_read_scale(c.my_sensor.load_R,-1.7f);
                
                if (printCount++ >=printInterval)
                {
                    printCount=0;
                    printf("   A1:  load cell LR %.1f %.1f", ST.load_L, ST.load_R);
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
            
            if (c.lunatic_post_packet(p))
            {
                c.send_command(c.my_command);
            }
        }
        
        // Limit this loop speed to this many milliseconds (varies by what's attached)
        data_exchange_sleep(delayMs);
    }
    
    return 0;
}

