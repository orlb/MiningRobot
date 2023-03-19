/*
  This file computes:
XYZ coordinate systems from joint angles (forward kinematics), and 
joint angles from an XYZ coordinate system (inverse kinematics). 

Dr. Orion Lawlor, lawlor@alaska.edu, 2023-03-08 (Public Domain)
*/
#ifndef __AURORA_ROBOT_KINEMATICS_H
#define __AURORA_ROBOT_KINEMATICS_H

#include "../aurora/coords.h"

namespace aurora {

/** This specifies the numbering of the robot links.
A "link" is a coordinate system of some rigid part of the robot, 
 usually connected to other links via joints. 
We also specify links for the cameras, since this lets us use the 
 same coordinate system transform and parenting as for moving parts.
Link order should put parent before child coordinates. 
*/
typedef enum { 
    link_pit=0, ///< Mining pit coordinates == world coordinates
    link_frame=1, ///< Robot's frame, for driving

    link_fork=2, ///< Front scoop raise / lower
    link_dump=3, ///< Front scoop pivot to dump

    link_boom=4, ///< Arm first link, moves everything left-right
    link_stick=5, ///< Arm second link, up-down, with depth camera
    link_tilt=6, ///< Arm third link, tilt forward-back
    link_spin=7, ///< Arm last link, spin tool
    link_coupler=8, ///< Center of tool coupler top pin
    link_grinder=9, ///< Front center tooth of rockgrinder wheel

    link_depthcam=10, ///< Front depth camera, mounted on stick
    link_drivecamflip=11,  ///< Flip frame coords for back drive camera
    link_drivecam=12,  ///< Back drive camera, mounted on frame
    link_count
} robot_link_index;

/// Link types:
typedef enum {
    linktype_fixed=0, ///< Fixed, like world coordinates.
    linktype_revolute=1, ///< Rotate relative to a parent.
} robot_linktype;

/// Rotation axes:
typedef enum {
    axisNONE=0, ///< Do not rotate
    axisX=1, ///< Rotate around X axis (in YZ plane)
    axisY=2, ///< Rotate around Y axis (in XZ plane)
    axisZ=3, ///< Rotate around Z axis (in XY plane)
} robot_axis;

/// Stores static (const) information about each robot link's geometry.
///  Used to reduce dependencies on specific robot geometry in solvers.
struct robot_link_geometry {
    /// Human-readable short name string, like "frame"
    const char *name; 
    
    /// Our own link index, one of the enum above.
    robot_link_index index; 

    /// Our link type (some fields below are not used for some types)
    robot_linktype type;

    /// Index of our parent link: if the parent moves, we move.
    /// If parent==index, then this is a root link like the pit coords.
    robot_link_index parent; 
    
    /// Our parent-relative 3D origin point.
    ///   Our origin is at these coordinates in our parent.
    ///   revolute joints rotate around this point.
    vec3 origin; 
    
    /// Our parent-relative 3D axis of rotation, 
    ///   or axisNONE if we do not rotate.
    robot_axis axis;
    
    /// Our fixed rotation angle, in degrees.  
    float fixed_angle;
    
    /// Our index in the robot_joint_state.array of our rotation angle.
    ///  If this is -1, we're not in that array.
    /// If present, this is added to the joint rotation above.
    int joint_index;
    
    /// Physical robot joint angle limits, in degrees, on raw angle (pre fixed angled)
    ///   0.0 means no such limit applies.
    float angle_min, angle_max; 
};

/// Look up this robot link's geometry in a fast constant table. 
///   Throws if L is not a valid link index.
const robot_link_geometry &link_geometry(robot_link_index L);

/** This class does coordinate system transforms for all robot links. 
*/
class robot_link_coords {
public:
    
/* Static utility functions that don't need a constructed object */
    /// Convert this quaternion representing world-coordinates orientation into a coord3D
    
    
    /// Return this link's relative rotation angle, in degrees, in this state. 
    static float link_degrees(robot_link_index L,const robot_joint_state &state)
    {
        const robot_link_geometry &g=link_geometry(L);
        float angle = g.fixed_angle;
        if (g.joint_index>=0) angle += state.array[g.joint_index];
        return angle;
    }

#ifdef __gl_h_ /* OpenGL support */
    /// Apply the incremental OpenGL transform to get from this link to its parent.
    static void glTransform(robot_link_index L,const robot_joint_state &state)
    {
        const robot_link_geometry &g=link_geometry(L);
        glTranslatef(g.origin.x,g.origin.y,g.origin.z);
        if (g.axis==axisNONE) return;
        float angle=link_degrees(L,state);
        switch (g.axis) {
        case axisX: glRotatef(angle,1,0,0); break;
        case axisY: glRotatef(angle,0,1,0); break;
        case axisZ: glRotatef(angle,0,0,1); break;
        default: break;
        };
    }
#endif

/** Initialize this object from a joint state, to get coordinate transforms for all the robot's links. */
    robot_link_coords(const robot_joint_state &state_, robot_coord3D frameCoords=robot_coord3D())
        :state(state_) 
    {
        links[link_pit].reset(); // pit has identity coordinate system
        links[link_frame]=frameCoords;
    // Preemptively fill in all coordinate system transforms. 
    //  (future alternative option: memoize on-demand computed coordinate systems)
        for (int i=link_frame+1;i<link_count;i++)
        {
            robot_link_index L=robot_link_index(i);
            const robot_link_geometry &G=link_geometry(L);
            const robot_coord3D &parent=(G.parent>=0)?links[G.parent]:links[link_pit];
            
            links[L].origin=parent.world_from_local(G.origin);
            float angle = link_degrees(L,state);
            float rad = angle * DEG2RAD;
            float s=sinf(rad), c=cosf(rad);
            switch (G.axis) {
            case axisX: 
                links[L].X=parent.X;
                links[L].Y= c*parent.Y +s*parent.Z;
                links[L].Z=-s*parent.Y +c*parent.Z;
                break;
            case axisY: 
                links[L].Z= c*parent.Z -s*parent.X;
                links[L].X= s*parent.Z +c*parent.X;
                links[L].Y=parent.Y;
                break;
            case axisZ: 
                links[L].X= c*parent.X -s*parent.Y;
                links[L].Y= s*parent.X +c*parent.Y;
                links[L].Z=parent.Z;
                break;
            default:
                links[L].X=parent.X;
                links[L].Y=parent.Y;
                links[L].Z=parent.Z;
                break;       
            };
        }
    }

/** Access the coordinate transform for this robot link */
    const robot_coord3D &coord3D(robot_link_index L) {
        if (L>=0 && L<link_count) return links[L];
        else throw std::runtime_error("Invalid link index in robot_link_coords::coord3D");
    }
    
private:
    robot_joint_state state;
    robot_coord3D links[link_count];
};



}; /* end namespace */

#endif

