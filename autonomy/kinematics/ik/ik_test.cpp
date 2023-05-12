/* 
 Inverse kinematics testing: generate random joint angles, 
 compute forward kinematics to get tool position, then compute
 inverse kinematics to get back to joint angles.
 
 This is useful for developing and debugging the kinematics code.
 
 Dr. Orion Lawlor, lawlor@alaska.edu, 2023-03 (Public Domain)
*/
#include <stdio.h>
#include "aurora/lunatic.h"
#include "aurora/kinematics.h"
#include "aurora/kinematic_links.cpp"

using namespace aurora;

// Make a random float between lo and hi.  Calls rand()
float rand_float(float lo,float hi)
{
    int limit=0xfffff;
    float scale=(rand()%limit)*(1.0/limit);
    return lo+scale*(hi-lo);
}

int main() {
    MAKE_exchange_backend_state();
    excahauler_IK ik;
    long fail_count=0;
    
    for (int rep=0;rep<10000;rep++) 
    {
        srand(rep); //<- allow us to jump back to this pseudorandom test
        
        // Make a test point using known angles
        robot_joint_state test_joint={0};
        test_joint.angle.stick=rand_float(+1.0,+72.0);
        test_joint.angle.boom=rand_float(-58.0,+63.0);
        test_joint.angle.tilt=rand_float(-77.2, +59.7);
        
        // Forward kinematics
        robot_link_coords test_fk(test_joint); 
        const robot_coord3D &fk_tilt=test_fk.coord3D(link_tilt);
        vec3 tilt = fk_tilt.origin;
        float tilt_deg = excahauler_IK::frame_degrees(fk_tilt.Y);
        
        // Test inverse solve back to tilt coords:
        robot_joint_state joint={0};
        int ret = ik.solve_tilt(joint,tilt,tilt_deg);
        bool fail=true;
        if (ret>0) { // successful solve
            // Recompute forward kinematics (verifies angles are correct)
            robot_link_coords fk(joint);
            vec3 re = fk.coord3D(link_tilt).origin; // recompute tilt origin
            
            // Compute error on reconstructed position
            float epsilon_m=1.0e-4; // error tolerance (meters)
            float epsilon_deg=0.01; // error tolerance (degrees)
            if (length(tilt - re)<epsilon_m &&
                fabs(joint.angle.boom-test_joint.angle.boom)<epsilon_deg &&
                fabs(joint.angle.stick-test_joint.angle.stick)<epsilon_deg &&
                fabs(joint.angle.tilt-test_joint.angle.tilt)<epsilon_deg
               ) 
            { // all values look good--test pass
               fail=false;
            }
            
            if (fail) // || rep%1000 == 0) // print angles occasionally
            printf("Tilt %.3f %.3f  -> Angle BST %.1f  %.1f  %.1f -> %.3f %.3f\n", 
                   tilt.y, tilt.z,
                   joint.angle.boom, joint.angle.stick, joint.angle.tilt,
                   re.y, re.z);

        }
        
        // Test tool tip positioning:
        robot_coord3D grinder = test_fk.coord3D(link_grinder);
        //robot_coord3D fk_tilt = test_fk.coord3D(link_tilt);
        robot_coord3D ik_tilt = robot_link_coords::parent_from_child(link_tilt, link_grinder, grinder);
        vec3 grinder_in_tilt = fk_tilt.local_from_world(grinder.origin);
        if (length(grinder_in_tilt - ik_tilt.origin)>0.0001) {
            fail=true;
            printf(" fk_tilt: ");
            fk_tilt.print();
            printf(" ik_tilt: ");
            ik_tilt.print();
        }
        
        if (fail) {
            // Print details on failed solve:
            printf(" IK fail %d: test BS %.1f  %.1f\n",
                ret, 
                test_joint.angle.boom, test_joint.angle.stick);
            fail_count++;
        }
    }
    
    printf("Tests finished: %ld failures\n",fail_count);
    return 0;
}

