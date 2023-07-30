/**
 Robot-specific kinematics: inverse kinematic solver, and 
 exact numeric details on each of the robot's links.
 
 Dr. Orion Lawlor, lawlor@alaska.edu, 2023-03 (Public Domain)
*/
#include <stdexcept>
#include <stdio.h>
#include "kinematics.h"

namespace aurora {

/** List of all robot links that include joints with angles */
const static std::vector<robot_link_index> links_with_revolute_joints = {
    link_fork, link_dump,
    link_boom, link_stick, link_tilt, link_spin
};

/** Coarse sanity-check this set of joint angles (check limits on angles) */
bool joint_state_sane(const robot_joint_state &joint)
{
    for (robot_link_index L : links_with_revolute_joints) {
        const robot_link_geometry &G=link_geometry(L);
        if (G.joint_index>=0) {
            float angle = joint.array[G.joint_index];
            if (angle<G.angle_min || angle>G.angle_max) return false;
        }
    }
    return true;
}

/**
 Solves inverse kinematics (positions to joint angles)
 problems for the excahauler robot.  This robot is mostly 2D motion
 in the YZ plane, so it's much easier than general multi-link IK. 
*/
class excahauler_IK {
public:
	/** Given a 3D vector in frame coords, return the angle of this
	 direction vector in the YZ plane.
	 The Y axis has an angle of 0, the Z axis has an angle of +90 degrees.
	 Angle is returned in degrees around the X axis, rotating the Y axis upward.
	*/
	static float frame_degrees(const vec3 &v) {
		float rad=atan2(v.z,v.y);
		return RAD2DEG*rad;
	}
    
	/**
	  Given a 3D vector for the origin of the tilt link at the end of the stick,
	  update the boom and stick joint angles to reach that point,
	  and the tilt angle to reach that tool orientation (excahauler_IK::frame_degrees(tool.Y)).
	  
	  Returns 1 if the joint was reachable, -1 if too far
	  (Future: -2 if collision?)
	*/
	int solve_tilt(robot_joint_state &joint, const vec3 &tilt_loc, float tool_deg)
	{
		vec3 tilt_rel = tilt_loc - boomG.origin; // to target from boom start
		float tilt_len = length(tilt_rel);
		float tilt_deg = frame_degrees(tilt_rel); // angle of vector from boom start to tilt pivot
		
		// Use law of cosines to solve for the angle from boom to tilt (BT)
		//  Side a = boom
		//  Side b = tilt
		/// Side c = stick
		float a=boom_len, b=tilt_len, c=stick_len;
		float cos_tb = (a*a + b*b - c*c)/(2.0f*a*b);
		if (cos_tb>1.0f || cos_tb<-1.0f) return -1; // no good
		float tb_deg=RAD2DEG*acos(cos_tb);
		joint.angle.boom=tilt_deg + tb_deg - boom_start; // frame to boom = frame to tilt - boom to tilt
        
        // Use law of cosines again on angle from stick to boom (SB)
		float cos_sb = (a*a + c*c - b*b)/(2.0f*a*c);
		if (cos_sb>1.0f || cos_sb<-1.0f) return -1; // no good
		float sb_deg=RAD2DEG*acos(cos_sb);
		joint.angle.stick=sb_deg - stick_start + boom_start - 180.0f ;
		
		// Update the stick-to-tool tilt angle (ST)
		joint.angle.tilt = tool_deg - joint.angle.stick - joint.angle.boom; 
		if (joint.angle.tilt<-180.0f) joint.angle.tilt+=360.0f;
        
		return 1;
	}


	excahauler_IK() 
		:frameG(link_geometry(link_frame)),
		 boomG(link_geometry(link_boom)),
		 stickG(link_geometry(link_stick)),
		 tiltG(link_geometry(link_tilt))
	{
		boom_len=length(stickG.origin); // boom connects frame to stick
		stick_len=length(tiltG.origin); // stick connects boom to tilt
		boom_start=frame_degrees(stickG.origin);
		stick_start=frame_degrees(tiltG.origin);
	}

private:
	const robot_link_geometry &frameG, &boomG, &stickG, &tiltG; 
	// Length of arm links relative to frame
	float boom_len, stick_len;
	// Angle of arm link origins
	float boom_start, stick_start;

};




const robot_link_geometry &link_geometry(robot_link_index L) 
{
    static const robot_link_geometry geom[link_count] = {
        { 
            "pit",
            link_pit, linktype_fixed,
            link_pit, vec3(0,0,0),
            axisNONE, 0.0f, -1
        },
        { 
            "frame",
            link_frame, linktype_revolute,
            link_pit, vec3(0,0,0),
            axisZ, 0.0f, -1
        },
        
        { /* Lift the whole front scoop assembly */
            "fork",
            link_fork, linktype_revolute,
            link_frame, vec3(0,0.455,0.150),
            axisX, 0.0f, 0,
            -58.7, +10
        },
        { /* Dump the front scoop out */
            "dump",
            link_dump, linktype_revolute,
            link_fork, vec3(0,0.250,0.020),
            axisX, 0.0f, 1,
            -80, -10
        },
        
        { /* First arm link */
            "boom",
            link_boom, linktype_revolute,
            link_frame, vec3(0,0.570,0.215),
            axisX, 0.0f, 2,
            -58, +52
        },
        {
            "stick",
            link_stick, linktype_revolute,
            link_boom, vec3(0,-0.312,0.750),
            axisX, 0.0f, 3,
            -32, +70
        },
        {
            "tilt",
            link_tilt, linktype_revolute,
            link_stick, vec3(0,0.735,0.012),
            axisX, 0.0f, 4,
            -75, +52
        },
        {
            "spin",
            link_spin, linktype_revolute,
            link_tilt, vec3(0,0.000,-0.075),
            axisY, 0.0, 5,
            -30, +30
        },
        {
            "coupler",
            link_coupler, linktype_fixed,
            link_spin, vec3(0,0.0,0.085),
            axisNONE, 0.0f, -1
        },
        {
            "grinder",
            link_grinder, linktype_fixed,
            link_coupler, vec3(0,0.50,-0.311+0.05),
            axisNONE, 0.0f, -1
        },
        /*
        {
            "spin",
            link_spin, linktype_revolute,
            link_tilt, vec3(0,0.060,-0.075),
            axisY, 0.0, 5,
            -30, +30
        },
        {
            "coupler",
            link_coupler, linktype_fixed,
            link_spin, vec3(0,0.0,0.085),
            axisNONE, 0.0f, -1
        },
        {
            "grinder",
            link_grinder, linktype_fixed,
            link_coupler, vec3(0,0.513,-0.311),
            axisNONE, 0.0f, -1
        },
        */
        
        
        { /* Realsense depth camera on top of stick */
            "depthcam",
            link_depthcam, linktype_revolute,
            link_stick, vec3(0,0.490,0.500),
            axisX, -180+57+1, -1
        },
        {
            "drivecamflip",
            link_drivecamflip, linktype_revolute,
            link_frame, vec3(0,-0.575,0.270+0.215),
            axisZ, 180.0f, -1
        },
        { /* Genius 120 FOV camera on back electronics box */
            "drivecam",
            link_drivecam, linktype_revolute,
            link_drivecamflip, vec3(0,0,0),
            axisX, -90.0f, -1
        },
    };
    
    if (L<0 || L>=link_count) throw std::runtime_error("Invalid link index passed to link_geometry");
    return geom[L];
}


}; /* end namespace aurora */


