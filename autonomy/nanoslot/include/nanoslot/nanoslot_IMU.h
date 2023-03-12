/*
 Inertial Measurement Unit (IMU) filtering code interface,
 used by the robotics backend and navigation system.
 
 Dr. Orion Lawlor, lawlor@alaska.edu, 2023-03-11 (Public Domain)
*/
#ifndef __NANOSLOT_IMU_H
#define __NANOSLOT_IMU_H 1

#include "FusionMath.h" /* for quaternion type */
typedef FusionQuaternion nanoslot_quat;

#ifdef __AVR__
// Simpler version for Arduino (that only can get to nanoslot headers) 
class vec3 {
    float x,y,z;
    vec3() { x=y=z=0.0f; }
    vec3(float x_, float y_, float z_) :x(x_), y(y_), z(z_) {}
};
#else
// Full vec3 for PC
#include "../../include/aurora/vec3.h" /* 3D vector type used for output */

#endif // __AVR__ conditional

// Convert a fusion vector to a vec3
inline vec3 make_vec3(const FusionVector &v) { 
    return vec3(v.axis.x,v.axis.y,v.axis.z); 
}



/**
 Packed bitfield struct holding 10-bit XYZ values.
  Used for gyro and accelerometer data.  Will be padded to 4-byte alignment.
 Size critical: sent on serial link between Arduino and PC.
*/
struct nanoslot_xyz10_t {
	int32_t x:10;
	int32_t y:10;
	int32_t z:10;
	uint32_t type:2; // Round out to 32 bits with scaling/valid flag
	enum {
	    type_1x = 0, // scale 1x
	    type_2x = 1, // scale 2x
	    type_4x = 2, // scale 4x
	    type_invalid = 3
	};
	
	void invalidate(void) {
	    x=y=z=0;
	    type=type_invalid;
	}
	bool valid(void) const {
	    return type!=type_invalid;
	}
	
#if _STDIO_H
    void print(const char *name) {
        printf("%s %4d %4d %4d (%d) ",
            name,x,y,z,type);
    }
#endif
};

// 3D vector type
typedef nanoslot_xyz10_t nanoslot_vec3_t;

/**
 Inertial measurement unit (IMU) data reading.
  Will be padded to 4-byte alignment.
 Sent on serial link between Arduino and PC.
*/
struct nanoslot_IMU_t {
    nanoslot_vec3_t acc; /// Accelerometer down vector (gravity)
    nanoslot_vec3_t gyro; /// Gyro rotation rates
    
    void invalidate(void) {
        acc.invalidate(); gyro.invalidate();
    }
};


/**
 Calibrated / derived IMU reading: one stored per robot link.
 Only used on PC side, so space is less critical.
*/
class nanoslot_IMU_state {
public:
    /// Normalized quaternion indicating our orientation,
    ///  in world space (absolute orientation).
    FusionQuaternion orient; 
    
    /// Our world-coordinates X axis vector.
    vec3 X(void) const { return make_vec3(FusionQuaternionXaxis(orient)); }
    /// Our world-coordinates Y axis vector.
    vec3 Y(void) const { return make_vec3(FusionQuaternionYaxis(orient)); }
    /// Our world-coordinates Z axis vector.
    vec3 Z(void) const { return make_vec3(FusionQuaternionZaxis(orient)); }
    
    /// Angles derived from the above quaternion.
    ///   Measured in world coords (absolute) for base links.
    ///   Measured in parent coords (relative) for other links.
    float yaw; ///< degrees rotation around Z, vertical direction (in XY plane)
    float roll; ///< degrees rotation around Y, motion direction (in XZ plane)
    float pitch; ///< degrees rotation around X, right hand direction (in YZ plane)
    
    /// Accelerometer in local coords, m/s^2
    vec3 local;
    /// Accelerometer rotated to world coords, m/s^2
    vec3 global; 
    /// Vibration estimate in local coords, m/s^2
    vec3 vibe;
    
    /// Valid flag: false if last report wasn't OK
    bool valid;
    
    
    void print(const FusionQuaternion &q) const {
        printf(" Q(%+.2f,+%.2f,+%.2f,+%.2f) ",q.element.w,q.element.x,q.element.y,q.element.z);
    }
    void print(const char *what,const vec3 &v) const {
        printf(" %s(%.2f,%.2f,%.2f) ", what,v.x,v.y,v.z);
    }
    
    void print(const char *what) const
    {
        printf("  %s { ",what);
        print(orient);
        printf(" YRP: %.0f %.0f %.0f  ",yaw,roll,pitch);
        print("local",local);
        print("global",global);
        print("vibe",vibe);
        printf(" } ");
    }
};



#endif

