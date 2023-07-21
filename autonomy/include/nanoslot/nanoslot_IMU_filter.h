/*
 Inertial Measurement Unit (IMU) filtering code implementation,
 used by slot programs.
 
 Dr. Orion Lawlor, lawlor@alaska.edu, 2023-03-11 (Public Domain)
*/
#ifndef __NANOSLOT_IMU_IMPLEMENTATION_H
#define __NANOSLOT_IMU_IMPLEMENTATION_H 1

#include "nanoslot_IMU.h" /* interface */
#include "FusionAhrs.h" /* filter implementation */

// Return the absolute value of v, but return 0 if the abs is less than minv
inline float abs_min(float v,float minv) {
    if (v>+minv) return v;
    else if (v<-minv) return -v;
    else return 0.0f;
}

// Return the absolute value of each element of this vector
inline vec3 abs_vec3(const vec3 &v,float minv) { 
    return vec3(abs_min(v.x,minv), abs_min(v.y,minv), abs_min(v.z,minv)); 
}


// Unpack a serial link 3D vector, applying this scale factor
vec3 vec_unpack(const nanoslot_xyz10_t &v,float s)
{
    switch (v.type) {
    case 0: s*=1.0f; break;
    case 1: s*=2.0f; break;
    case 2: s*=4.0f; break;
    default: s=0;
    }
    return vec3(
        v.x*s, v.y*s, v.z*s
    );
}

// Make this vec3 become a FusionVector 
inline FusionVector fusion(const vec3 &v) {
    const FusionVector result = {.axis = {
            .x = v.x,
            .y = v.y,
            .z = v.z,
    }};
    return result;
}

/**
 This filter class takes raw IMU readings, and
 filters them down to clean states.
*/
class nanoslot_IMU_filter
{
public:
    vec3 offset_acc; /// g force acclerometer offset (subtracted off of raw readings)
    vec3 scale_acc; /// scale factor to apply to incoming accelerometer data
    vec3 offset_gyro; /// degrees/sec gyro offset (subtracted off of raw readings)
    
    /** State update for a base (world) link, like the robot main frame */
    void update_base(nanoslot_IMU_state &state,const nanoslot_IMU_t &reading) 
    {
        update_reading(state,reading,delayMs);
        if (state.valid) angles_absolute(state,state.orient);
    }
    
    /** State update for a relative link, with a parent.
        Our coordinate system and parent will have the same X axis (if possible),
        after rotating our parent by parent_spin (if passed).
    */
    void update_parent(nanoslot_IMU_state &state,const nanoslot_IMU_t &reading,const nanoslot_IMU_state &parent, const FusionQuaternion *parent_spin=0)
    {
        //float heading=parent.yaw-90; // yaw has 0 along X axis, heading along Y axis.
        // update_reading(state,reading,delayMs,true,heading); //<- not great
        
        update_reading(state,reading,delayMs);
        
        if (state.valid && parent.valid) {
            FusionQuaternion P = parent.orient;
            if (parent_spin) { // does this ever help?
                // P = FusionQuaternionMultiply(parent.orient, *parent_spin);
                P = FusionQuaternionMultiply(*parent_spin, parent.orient);
            }
            
            // Rotate around Z, until our X axis matches our parent's:
            FusionAhrsMatchX(&ahrs,FusionQuaternionXaxis(P));
        
            // Compute roll, pitch, yaw relative to parent
            angles_relative(state,parent.orient);
        }
    }

    /** Create a filter designed to run every delayMs milliseconds */
    nanoslot_IMU_filter(int delayMs_, vec3 offset_acc_=vec3(0,0,0), vec3 offset_gyro_=vec3(0,0,0), vec3 scale_acc_=vec3(1,1,1))
        :offset_acc(offset_acc_), scale_acc(scale_acc_),
         offset_gyro(offset_gyro_), delayMs(delayMs_)
    {
        FusionAhrsInitialise(&ahrs);
        FusionOffsetInitialise(&offset,1000/delayMs);
    }
    
    /** Manually set our orientation */
    void set(nanoslot_IMU_state &state,const FusionQuaternion &Q)
    {
        state.orient=Q;
        state.valid=true;
        ahrs.quaternion=Q;
        ahrs.initialising=false;
        angles_absolute(state,state.orient);
    }
    
protected:
    /** Update this IMU state with this next unfiltered incoming reading. 
       Heading, if requested, gives the target yaw angle in degrees. 
    */
    void update_reading(nanoslot_IMU_state &state,
        const nanoslot_IMU_t &reading,
        int deltaMs)
    {
        // Don't contaminate the filter with invalid data
        if (!reading.acc.valid() || !reading.gyro.valid()) {
            state.valid=false;
            return;
        }
        
        // Scaling: 6 bit shift during pack, +2g = 32768
        vec3 acc = vec_unpack(reading.acc, (1<<6)/16384.0f);
        // Scaling: 4 bit shift during pack, +250deg/sec = 32768
        vec3 gyro_scale = vec_unpack(reading.gyro, (1<<4)/131.07f);
        
        // Apply offsets, now that we're converted to float
        acc = (acc-offset_acc)*scale_acc;
        gyro_scale -= offset_gyro;
        
        state.rate=gyro_scale;
        
        // Update the gyro offset (this only works at low rates, below 1 deg/sec)
        FusionVector gyro=FusionOffsetUpdate(&offset,fusion(gyro_scale));
        
        // Update the quaternion
        float deltaTime=deltaMs*0.001f; // milliseconds to seconds
        /* // This didn't work well 
        if (useHeading) {
            FusionAhrsUpdateExternalHeading(&ahrs,
                gyro,fusion(acc), heading, deltaTime);
        }
        */
        FusionAhrsUpdateNoMagnetometer(&ahrs,
             gyro,fusion(acc), deltaTime);
        
        // Export updated state
        state.orient=ahrs.quaternion;
        
        float g_a=9.8f; //< 1g in m/s^2 (Scale baked into the IMU, *not* local gravity!)
        
        vec3 last_local=state.local;
        state.local=g_a*make_vec3(ahrs.accelerometer);
        
        state.global=g_a*make_vec3(FusionAhrsGetGlobalAcceleration(&ahrs));
        
        float min_vibe=0.05f; //<- device itself has this much jitter
        float exp_filter=0.1f;
        vec3 cur_vibe=abs_vec3(state.local-last_local,min_vibe);
        vec3 new_vibe=cur_vibe*exp_filter + state.vibe*(1.0f-exp_filter);
        state.vibe=new_vibe;
        
        state.valid=true;
    }

    /** Derive our angles (roll, pitch, yaw) relative to this quaternion.
    */
    void angles_relative(nanoslot_IMU_state &state,const FusionQuaternion &parent)
    {
        /* Divide by the parent orientation, giving a relative orientation */
        FusionQuaternion rel=FusionQuaternionMultiply(
            FusionQuaternionConjugate(parent),state.orient);
        angles_absolute(state,rel);
    }
    
    /** Derive our angles (roll, pitch, yaw), ZYZ from this quaternion */
    void angles_absolute(nanoslot_IMU_state &state,const FusionQuaternion &q)
    {
        FusionEuler e=FusionQuaternionToEuler(q);
        state.yaw = e.angle.yaw;
        state.pitch = e.angle.roll; //<- they define these the other way (motion along X, we move along Y)
        state.roll = e.angle.pitch;
    }
    
    int delayMs; ///< sample-to-sample delay time in milliseconds
    FusionAhrs ahrs; 
    FusionOffset offset;
};




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


/**
 Fix coordinate system of raw IMU data, with MPU-6050 mounted 
 on the inside of a frame sidebar. 
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
 on the front face of a frame member. 
   Incoming for boom IMU:
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









#endif

