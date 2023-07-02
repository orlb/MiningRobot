/* This file is #included by the frontend and backend to present
   human-readable string names for the robot's states. 
   The list here must EXACTLY match include/aurora/robot_base.h
*/

const char *state_to_string(robot_state_t state)
{
	if (state<0 || state>=state_last) return "unknown_invalid";
	const static char *table[state_last+1]={
	"STOP", ///< EMERGENCY STOP (no motion)
	"drive", ///< normal manual driving from frontend
	"driveraw", ///< normal manual driving, no sanity checking
	"backend_driver", ///< drive from backend UI
	
	"autonomy", ///< Begin fully autonomous operation (mine cycles)
	"daily_start",  ///< clear accumulated data and start a new day
	
	"calibrate", ///< Calibrate internal gyros (stationary)
	
	"scan", ///< Scan terrain before mining
	"mine_start", ///< Prepare for mining (deploy scoop)
	"mine", ///< Autonomously mining
	"mine_stall", ///< Clearing mining head stall
	"mine_finish", ///< Finish up mining (pick up scoop)
	
	"weigh", ///< Weigh material in front scoop
	"haul_out", ///< Haul material out of pit
	"haul_dump", ///< Dump material out of scoop
	"haul_back", ///< Drive back into pit
	
	"stow", ///< Begin stowing
	"stowed", ///< Finished stowing (wait forever)

	"last"
	};

	return table[state];
}


