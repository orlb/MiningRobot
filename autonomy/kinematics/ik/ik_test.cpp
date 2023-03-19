/* 
 Inverse kinematics testing
*/
#include <stdio.h>
#include "aurora/lunatic.h"
#include "aurora/kinematics.h"
#include "aurora/kinematic_links.cpp"


namespace aurora {

/**
 Solves inverse kinematics (positions to joint angles)
 problems for the excahauler robot.  This robot is mostly 2D motion
 in the YZ plane, so it's much easier than general IK. 
*/
class excahauler_IK {
public:
	/** Given a 3D vector in frame coords, return the angle of this
	 direction vector in the YZ plane.
	 The Y axis has an angle of 0, the Z axis has an angle of +90 degrees.
	 Angle is returned in degrees above the Y axis.
	*/
	static float frame_degrees(const vec3 &v) {
		float rad=atan2(v.z,v.y);
		return RAD2DEG*rad;
	}


	/**
	  Given a 3D vector for the origin of the tilt link at the end of the stick,
	  update the boom and stick joint angles to reach that point,
	  Returns 1 if the joint was reachable, -1 if too far
	  (Future: -2 if collision?)
	*/
	int solve_tilt(robot_joint_state &joint, const vec3 &tilt_loc)
	{
		vec3 tilt_rel = tilt_loc - boomG.origin; // to target from boom start
		float tilt_len = length(tilt_rel);
		float tilt_deg = frame_degrees(tilt_rel);
		
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



} // end namespace


using namespace aurora;

float rand_float(float lo,float hi)
{
    int limit=0xfffff;
    float scale=(rand()%limit)*(1.0/limit);
    return lo+scale*(hi-lo);
}

int main() {
    MAKE_exchange_backend_state();
    excahauler_IK ik;
    
    srand(5);
    for (int rep=0;rep<10000;rep++) 
    {
        // Make a test point using known angles
        robot_joint_state test_joint={0};
        test_joint.angle.stick=rand_float(+1.0,+72.0);
        test_joint.angle.boom=rand_float(-58.0,+63.0);
        
        robot_link_coords test_fk(test_joint);
        vec3 tilt = test_fk.coord3D(link_tilt).origin;
        
        robot_joint_state joint={0};
        int ret = ik.solve_tilt(joint,tilt);
        bool fail=true;
        if (ret>0) { // successful solve
            // Recompute forward kinematics (verifies angles are correct)
            robot_link_coords fk(joint);
            vec3 re = fk.coord3D(link_tilt).origin; // recompute tilt origin
            
            // Compute error on reconstructed position
            float epsilon_m=1.0e-5; // error tolerance (meters)
            float epsilon_deg=0.001; // error tolerance (degrees)
            if (length(tilt - re)<epsilon_m &&
                fabs(joint.angle.boom-test_joint.angle.boom)<epsilon_deg &&
                fabs(joint.angle.stick-test_joint.angle.stick)<epsilon_deg
               ) 
            { // all values look good--test pass
               fail=false;
            }
            
            if (fail || rep%1000 == 0) // print angles occasionally
            printf("Tilt %.3f %.3f  -> Angle BS %.1f  %.1f -> %.3f %.3f\n", 
                   tilt.y, tilt.z,
                   joint.angle.boom, joint.angle.stick,
                   re.y, re.z);

        }
        
        if (fail) {
            // Print details on failed solve:
            printf(" IK fail %d: test BS %.1f  %.1f\n",
                ret, 
                test_joint.angle.boom, test_joint.angle.stick);
        }
    }

}

