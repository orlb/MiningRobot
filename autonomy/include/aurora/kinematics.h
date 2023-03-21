/*
  This file computes:
XYZ coordinate systems from joint angles (forward kinematics), and 
joint angles from an XYZ coordinate system (inverse kinematics). 

Robot-specific details and inverse kinematics are in kinematic_links.cpp.

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

Link order MUST put parent before child coordinates. 
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
    
    /// Return this link's relative rotation angle, in degrees, in this state. 
    static float link_degrees(robot_link_index L,const robot_joint_state &state)
    {
        const robot_link_geometry &g=link_geometry(L);
        float angle = g.fixed_angle;
        if (g.joint_index>=0) angle += state.array[g.joint_index];
        return angle;
    }
    
    /** Rotate this link's XYZ coordinate system by this angle
      (such as from link_degrees) around this axis (such as from robot_link_geometry.axis)
    */
    static void rotate_link(robot_coord3D &dest,const robot_coord3D &src,robot_axis axis,float deg)
    {
        float rad = deg * DEG2RAD;
        float s=sinf(rad), c=cosf(rad);
        switch (axis) {
        case axisX: 
            dest.X=src.X;
            dest.Y= c*src.Y +s*src.Z;
            dest.Z=-s*src.Y +c*src.Z;
            break;
        case axisY: 
            dest.Z= c*src.Z -s*src.X;
            dest.X= s*src.Z +c*src.X;
            dest.Y=src.Y;
            break;
        case axisZ: 
            dest.X= c*src.X -s*src.Y;
            dest.Y= s*src.X +c*src.Y;
            dest.Z=src.Z;
            break;
        default:
            dest.X=src.X;
            dest.Y=src.Y;
            dest.Z=src.Z;
            break;       
        };        
    }
    
    /**
     Return the coordinate system of this child link relative to this parent.
     (Only works for non-joint coordinate systems, for now).
    */
    static robot_coord3D parent_from_child(robot_link_index parentL,robot_link_index childL,const robot_coord3D &child)
    {
        robot_coord3D parent;
        parent.origin = vec3(0.0); // we will add up this origin
        parent.X = child.X; // we assume no rotation (angle 0)
        parent.Y = child.Y;
        parent.Z = child.Z;
        
        // Walk from child up to parent
        robot_link_index curL = childL;
        while (curL != parentL) {
            const robot_link_geometry &curG=link_geometry(curL);
            
            parent.origin += curG.origin; // shift back up to parent coords
            //  FIXME: this assumes no rotation, so all origins are in same XYZ basis.
            
            if (curG.parent<0) throw std::runtime_error("robot_link_coords::parent_from_child missing parent in walk");
            else if (curG.parent == curL) throw std::runtime_error("robot_link_coords::parent_from_child loop in parent walk");
            else curL = curG.parent;
        }
        
        return parent;
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
            rotate_link(links[L],parent,G.axis,link_degrees(L,state));
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

