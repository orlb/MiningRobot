/*
 Autonomous mine planning code. 
*/
#ifndef __AURORA_MINING_H
#define __AURORA_MINING_H


namespace aurora {

/// Mining tool max power level (limited to keep gearbox from breaking too often)
const float mine_power_limit=0.6;

/// Distance away from scoop tip to start mining
const float mine_start_distance=0.25;

/// Distance up from scoop floor to start mining
const float mine_floor_height=-0.05;

/// Pit excavation angle, measured up from horizontal.
const float mine_pit_angle=57.0; 



/// Scoop downforce preload, kgf (right hand side only)
const float mine_preload_force=15.0; 

/// Minimum minerate before we declare a stall
const float minerate_stall_threshold=40.0; 








};







#endif

