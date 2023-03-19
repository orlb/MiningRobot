/**
 Static data about each of the robot's links.
*/
#include <stdexcept>
#include <stdio.h>
#include "kinematics.h"

namespace aurora {

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
            -58.7, +15.2
        },
        { /* Dump the front scoop out */
            "dump",
            link_dump, linktype_revolute,
            link_fork, vec3(0,0.250,0.020),
            axisX, 0.0f, 1,
            -72.7, +31.6
        },
        
        { /* First arm link */
            "boom",
            link_boom, linktype_revolute,
            link_frame, vec3(0,0.570,0.215),
            axisX, 0.0f, 2,
            -58, +63
        },
        {
            "stick",
            link_stick, linktype_revolute,
            link_boom, vec3(0,-0.312,0.750),
            axisX, 0.0f, 3,
            -15.3, +71.6
        },
        {
            "tilt",
            link_tilt, linktype_revolute,
            link_stick, vec3(0,0.735,0.012),
            axisX, 0.0f, 4,
            -77.2, +59.7
        },
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


