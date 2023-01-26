/* 
 Crosscutting nanoslot configuration info.
*/
#ifndef __NANOSLOT_CONFIG_H
#define __NANOSLOT_CONFIG_H

// Serial data rate
#define NANOSLOT_BAUD_RATE 115200

// Arduino bootloader delay (milliseconds)
//   Safest: 2000ms delay, guaranteed past the bootloader.
//   More responsive with shorter delay though.
#define NANOSLOT_BOOTLOADER_DELAY_MS 100

// A-packet command field for ID, command, error
#define NANOSLOT_A_ID 0x1 /* ID byte request / response */
#define NANOSLOT_A_SENSOR 0xB /* sensor data from device side */
#define NANOSLOT_A_COMMAND 0xC /* command data from PC side */
#define NANOSLOT_A_DEBUG 0xD  /* human-readable trace data */
#define NANOSLOT_A_ERROR 0xE  /* human-readable fatal error */

#define NANOSLOT_ID_SANITY 0xA5 /* sanity-check for ID packet */


// Exec-time handoff of serial port from nanoboot to slot program.
#define NANOSLOT_HANDOFF_FANCY 1  /* 0: simple exec and reopen.  1: fancy re-use file descriptor */

#endif


