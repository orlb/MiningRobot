/**
 Aurora Robotics general robot structs: sensors, power, states.

 Orion Sky Lawlor, lawlor@alaska.edu, 2014--2023 (Public Domain)
*/
#ifndef __AURORA_ROBOTICS__ROBOT_BASE_H
#define __AURORA_ROBOTICS__ROBOT_BASE_H
#include <stdint.h> /* for uint32_t */
#include "field_geometry.h"
#include "coords.h" /* coordinate systems needed for localization */


/** This is the Arduino's AREF analog reference voltage.
  It's the scale factor that gives true voltage output,
  and should be measured from the AREF pin against Arduino ground. */
#define AD_AREF_voltage (4.78)

/** This scale factor converts an
	Arduino Analog/Digital Data Number (0-1023)
	to a real voltage, assuming direct feed-in. */
#define AD_DN2low_voltage (AD_AREF_voltage/(1024.0))

/** This scale factor converts an
	Arduino Analog/Digital Data Number (0-1023)
	to a real voltage, after the resistor divider scaling.
*/
#define AD_DN2high_voltage ((AD_AREF_voltage)*(11.0)/1024.0)


    
// Possible connected tools
typedef enum {
    tool_none=0,
    tool_rockgrinder=1
} tool_type;

/** This class contains all the robot's sensors, on Arduino, backend, or front end.
Raw sensor values go as bitfields, because many of them are 10-bit quantities:
	- Arduino A/D values are 10 bits each
	- Arena positions in cm are 9-10 bits each (arena is 378x738cm)
	- Blinky angle reports are about 9 bits each (500 samples per rotation)
*/
class robot_sensors_arduino
{
public:
    float load_TL, load_TR; ///< load cells on tool: left and right (kgf)
    float load_SL, load_SR; ///< load cells on scoop: left and right (kgf)
    
    float cell_M, cell_D; ///< raw voltages on mining and driving battery cells
    float charge_M, charge_D; ///< percent charge on mining and driving batteries
    
    float spin; ///< mining head spin rate in counts/sec

    // SUBTLE: these bitfields need to add up to a multiple of 32 bits
	uint32_t stop:1; ///< EMERGENCY STOP button engaged
    uint32_t heartbeat:4;
    
    uint32_t Mstall:1;
    uint32_t DLstall:1;
    uint32_t DRstall:1;

    uint32_t Mcount:8; /// Raw encoder tick count for mining head
	uint32_t DLcount:8; /// Raw encoder tick count for left drive wheel
	uint32_t DRcount:8; /// Raw encoder tick count for right drive wheel

	uint32_t encoder_raw:8;
    uint32_t stall_raw:8;
    
    // Bits inside connected:
    enum {
        connected_D0=0, // in big rear box
        connected_F0=1,
        connected_F1=2, // in micro front box
        connected_A0=3, // in arm box
        connected_A1=4, 
        connected_C0=5 // on rockgrinder tool
    };
    uint32_t connected:8;
    uint32_t pad:8;
    
    // Return the tool that is currently connected, or tool_none
    tool_type connected_tool() const {
    	if (0!=(connected & (1<<connected_C0))) return tool_rockgrinder;
    	return tool_none;
    }
};

/**
 This class contains a power setting for each of the robot's actuators.

 The float power values run from -1.0 (full backwards) to 0 (stop) to +1.0 (full forwards)
*/
class robot_power {
public:
	enum { drive_stop=0 };

	float left; // left drive wheels: + is forward
	float right; // right drive wheels: + is forward

	float fork; // front scoop lift fork: + is up
	float dump; // front scoop curl: + is raise

	float boom; // arm first link: + extends the boom
	float stick; // arm second link: + extends the stick
	float tilt; // arm third link: + extends tool forward
	
	float spin; // tool coupler rotation: + is right handed
	float tool; // excavation tool: + is normal forward cut
	
	unsigned char torque; // one bit per power: 0=torque control; 1=speed or position control
	unsigned char read_L:1; // set this bit to read the left load cells (0 for right cells).

	robot_power() { stop(); }
	
	/// Set all power values to 0
	void stop(void) {
		left=right=fork=dump=boom=stick=tilt=spin=tool=drive_stop; // all-stop
		torque=0;
		read_L=0;
	}
	
	/// Blend a scaled copy of this power value into ours.
	///  frac=0.0 means no change.  frac=1.0 means this=src.
	///  This is designed to allow smoother changes.
	void blend_from(const robot_power &src,float frac)
	{
	    left=left*(1.0-frac)+src.left*frac;
	    right=right*(1.0-frac)+src.right*frac;
	    
	    
	    fork=fork*(1.0-frac)+src.fork*frac;
	    dump=dump*(1.0-frac)+src.dump*frac;
	    
	    
	    boom=boom*(1.0-frac)+src.boom*frac;
	    stick=stick*(1.0-frac)+src.stick*frac;
	    tilt=tilt*(1.0-frac)+src.tilt*frac;
	    
	    spin=spin*(1.0-frac)+src.spin*frac;
	    tool=tool*(1.0-frac)+src.tool*frac;
	}

#ifdef _STDIO_H
    void print(const char *what) {
        printf("%s: LR %.1f %.1f  FD %.1f %.1f  BST %.1f %.1f %.1f  PO %.1f %.1f\n",
            what, left,right, fork,dump, boom,stick,tilt, spin,tool);
    }
#endif
};



/**
  This is a list of possible robot states.
  It's mostly maintained on the backend, but
  can be commanded from the front end.
  
  This must match the strings in include/aurora/robot_states.cpp
*/
typedef enum {
	state_STOP=0, ///< EMERGENCY STOP (no motion)
	state_drive, ///< normal manual driving
	state_backend_driver, ///< drive from backend UI

	state_autonomy, ///< full autonomy start state
	
	state_calibrate, ///< Calibrate internal gyros (stationary)
	
	state_scan, ///< Scan terrain before mining
	state_mine_start, ///< Prepare for mining (deploy scoop)
	state_mine, ///< Autonomously mining
	state_mine_stall, ///< Clearing mining head stall
	state_mine_finish, ///< Finish up mining (pick up scoop)
	
	state_weigh, ///< Weigh material in front scoop
	state_haul_out, ///< Haul material out of pit
	state_haul_dump, ///< Dump material out of scoop
	state_haul_back, ///< Drive back into pit
	
	state_stow, ///< Begin stowing
	state_stowed, ///< Finished stowing (wait forever)

	state_last ///< end state (repeat from mine_drive)
} robot_state_t;
const char *state_to_string(robot_state_t state);



/** The angles, in degrees, of each robot joint. 
  For example, joints.angle.boom is the angle of the boom over the frame.
  This is a union, so joints.angle.boom == joints.array[2]
  
  See aurora/kinematics.h for details on each link.
*/
union robot_joint_state {
    struct {
        /// Joint angles in degrees
        float fork, dump, boom, stick, tilt, spin;
    } angle;

    // This allows you to access the same angles as an array
    enum {count=6};
    float array[count];
};


// Everything about the visible computer vision markers / apriltags
class robot_markers_all {
public:
  // Integrated robot position
  robot_localization pose;

  // Raw sensed pose for each marker
  enum {NMARKER=4}; // <- all tag25h9 markers
  robot_localization markers[NMARKER];
};

/**
 This class contains everything we currently know about the robot.
*/
class robot_base {
public:
	robot_state_t state; ///< Current control state
	robot_joint_state joint; ///< Current joint angles
	robot_sensors_arduino sensor;  ///< Current hardware sensor values
	robot_localization loc; ///< Location
	robot_power power; // Current drive commands
};


#endif

