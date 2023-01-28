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



/* 
 General format:
    NANOSLOT_MY_ID is my hex ID, with leading "0x".
 nanoslot_command_<ID> is the command data sent by the PC to the Arduino.
 nanoslot_sensor_<ID> is the sensor data reported back by the Arduino.
*/

/* slot ID 0xA0: main arm motor controllers */
struct nanoslot_command_0xA0 {
    nanoslot_byte_t mode; // mode 0 == stop.  mode 1 == normal. 
    nanoslot_motorpercent_t stick; // linear actuators that pivot the stick
    nanoslot_motorpercent_t wrist; // linear actuators that pivot the wrist
};
struct nanoslot_sensor_0xA0 {
    nanoslot_byte_t latency;
    nanoslot_heartbeat_t heartbeat;
};


/* slot ID 0xEE: example nano */
struct nanoslot_command_0xEE {
    nanoslot_byte_t mode; // mode 0 == stop.  mode 1 == normal. 
    nanoslot_motorpercent_t LED;
};
struct nanoslot_sensor_0xEE {
    nanoslot_byte_t latency;
    nanoslot_heartbeat_t heartbeat;
};




#ifdef NANOSLOT_MY_ID

#define NANOSLOT_TOKENPASTE(a,b) a##b
#define NANOSLOT_TOKENPASTE2(a,b) NANOSLOT_TOKENPASTE(a,b)
#define NANOSLOT_COMMAND_MY  NANOSLOT_TOKENPASTE2(nanoslot_command_,NANOSLOT_MY_ID)
#define NANOSLOT_SENSOR_MY  NANOSLOT_TOKENPASTE2(nanoslot_sensor_,NANOSLOT_MY_ID)

#endif

#endif

