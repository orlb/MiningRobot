/*
 Plain-old-bytes structs used to exchange data with the Arduino nanoslots.
 These structs are sent on the serial connection between PC and Arduino,
 connecting slot_ID program swith the firmware_ID Arduino programs.

 Dr. Orion Lawlor, lawlor@alaska.edu, 2023-01-23 (Public Domain)
*/
#ifndef NANOSLOT_EXCHANGE_H
#define NANOSLOT_EXCHANGE_H 1

/* Datatypes used */
typedef uint8_t nanoslot_byte_t; ///< generic data byte
typedef uint8_t nanoslot_heartbeat_t; ///< heartbeat (watchdog-type counter)
typedef int8_t nanoslot_motorpercent_t; ///< -100 for full reverse, 0 for stop, +100 for full forward
typedef int8_t nanoslot_padding_t[3]; ///< padding to avoid false sharing between slots

// Packed bitfield struct holding 10-bit XYZ values.
//   Used for gyro and accelerometer data.  Will be padded to 4-byte alignment.
struct nanoslot_xyz10_t {
	int32_t x:10;
	int32_t y:10;
	int32_t z:10;
	uint32_t type:2; // Round out to 32 bits with a type field (scaling?  extra bits?)
	
#if _STDIO_H
    void print(const char *name) {
        printf("%s %d %d %d (%d) ",
            name,x,y,z,type);
    }
#endif
};

// 3D vector type
typedef nanoslot_xyz10_t nanoslot_vec3_t;

// Inertial measurement unit (IMU) data
//  Will be padded to 4-byte alignment.
struct nanoslot_IMU_t {
    nanoslot_vec3_t acc; /// Accelerometer down vector (gravity)
    nanoslot_vec3_t gyro; /// Gyro rotation rates
};


/** Info about autonomous operation shared with all firmware */
struct nanoslot_autonomy {
    /**
        Autonomous operation mode:
          mode==0 is STOP, safe mode, all actuators off.
          mode==1 or 2 is manual driving
          mode>2 is autonomous driving
    */
    nanoslot_byte_t mode;
};


/* 
 General per-slot format:
    NANOSLOT_MY_ID is my hex ID, with leading "0x".
 nanoslot_command_<ID> is the command data sent by the PC to the Arduino.
 nanoslot_sensor_<ID> is the sensor data reported back by the Arduino.
*/

/** slot ID 0xA0: arm motor controllers */
struct nanoslot_command_0xA0 {
    nanoslot_autonomy autonomy; 
    
    enum {n_motors=2};
    nanoslot_motorpercent_t motor[n_motors]; // brushed DC linear actuator motors
};
struct nanoslot_sensor_0xA0 {
    // IMU needs to be listed first in the struct, for alignment
    nanoslot_IMU_t imu0;
    nanoslot_IMU_t imu1;
    nanoslot_IMU_t imu2;
    
    // Single-byte fields go after IMU data
    nanoslot_heartbeat_t heartbeat;
    nanoslot_byte_t feedback;
    // need a multiple of 4 bytes for Arduino and PC to agree on struct padding
    nanoslot_byte_t spare1,spare2;
};

/** slot ID 0xD0: drive motor controllers */
struct nanoslot_command_0xD0 {
    nanoslot_autonomy autonomy; 
    enum {n_motors=4};
    nanoslot_motorpercent_t motor[n_motors]; // brushed DC drive motors
};
struct nanoslot_sensor_0xD0 {
    nanoslot_heartbeat_t heartbeat;
    nanoslot_byte_t raw; // raw bit version of sensors
    nanoslot_byte_t stall; // raw bit version of sensors
    enum {n_sensors=6};
    nanoslot_byte_t counts[n_sensors]; // counts for each sensor channel
};


/** slot ID 0xEE: example nano */
struct nanoslot_command_0xEE {
    nanoslot_autonomy autonomy; 
    nanoslot_motorpercent_t LED; // pin 13 debug
};
struct nanoslot_sensor_0xEE {
    nanoslot_heartbeat_t heartbeat;
    nanoslot_byte_t latency;
};

/** Debug data kept per slot */
struct nanoslot_debug_t {
    nanoslot_byte_t flags; // 0: no extra debug info.  Bits request various debug features (TBD)
    nanoslot_byte_t packet_count; // serial packets recv'd (like a heartbeat)
    
    
};

/** Each slot keeps this data on the exchange.
    The idea is we can send commands like nano.slot_A0.command.motor[1]=100;
*/
template <typename command_t, typename sensor_t>
struct nanoslot_exchange_slot 
{
    command_t command; // commands to send to Arduino
    sensor_t sensor; // sensor data received back from Arduino
    nanoslot_debug_t debug; // debug data
};


/** One struct with all nano slot data, 
   for example to live in the data exchange, 
   or for logging & debugging. */
struct nanoslot_exchange {
    uint16_t size; // size, in bytes, of this struct (exit early if mismatch here)
    void sanity_check_size(void);
    
    // The backend increments this every time it writes commands
    nanoslot_heartbeat_t backend_heartbeat;
    // Autonomy mode is shared by all slots.  This value is published by the backend.
    nanoslot_autonomy autonomy;
    
    nanoslot_padding_t pad_0; ///<- padding prevents false sharing slowdown
    
    // Each slot stores its data here:
    nanoslot_exchange_slot<nanoslot_command_0xA0,nanoslot_sensor_0xA0> slot_A0;
    nanoslot_padding_t pad_A0;
    
    nanoslot_exchange_slot<nanoslot_command_0xD0,nanoslot_sensor_0xD0> slot_D0;
    nanoslot_padding_t pad_D0;
    
    nanoslot_exchange_slot<nanoslot_command_0xEE,nanoslot_sensor_0xEE> slot_EE;
    nanoslot_padding_t pad_EE;
};


#ifdef NANOSLOT_MY_ID

#define NANOSLOT_TOKENPASTE(a,b) a##b
#define NANOSLOT_TOKENPASTE2(a,b) NANOSLOT_TOKENPASTE(a,b)
#define NANOSLOT_COMMAND_MY  NANOSLOT_TOKENPASTE2(nanoslot_command_,NANOSLOT_MY_ID)
#define NANOSLOT_SENSOR_MY  NANOSLOT_TOKENPASTE2(nanoslot_sensor_,NANOSLOT_MY_ID)

#endif

#endif

