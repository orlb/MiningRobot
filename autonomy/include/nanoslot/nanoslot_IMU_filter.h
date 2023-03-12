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
    
    /** State update for a base (world) link, like the robot main frame */
    void update_base(nanoslot_IMU_state &state,const nanoslot_IMU_t &reading) 
    {
        update_reading(state,reading,delayMs);
        if (state.valid) angles_absolute(state,state.orient);
    }
    
    /** State update for a relative link, with a parent.
        Our coordinate system and parent will have the same X axis (if possible)
    */
    void update_parent(nanoslot_IMU_state &state,const nanoslot_IMU_t &reading,const nanoslot_IMU_state &parent)
    {
        //float heading=parent.yaw-90; // yaw has 0 along X axis, heading along Y axis.
        // update_reading(state,reading,delayMs,true,heading); //<- not great
        
        update_reading(state,reading,delayMs);
        
        if (state.valid && parent.valid) {
            // Rotate around Z, until our X axis matches our parent's:
            FusionAhrsMatchX(&ahrs,FusionQuaternionXaxis(parent.orient));
        
            angles_relative(state,parent.orient);
        }
    }

    /** Create a filter designed to run every delayMs milliseconds */
    nanoslot_IMU_filter(int delayMs_)
        :delayMs(delayMs_)
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
        
        // Update the gyro offset
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









#endif

