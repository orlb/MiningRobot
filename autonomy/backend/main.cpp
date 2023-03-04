/**
  Aurora Robotics Backend Code

  Orion Sky Lawlor, lawlor@alaska.edu, 2014-03-23 (Public Domain)
*/
#define AURORA_IS_BACKEND 1

#include <iostream>
#include <fstream>
#include <cmath>
#include <thread>
#include <mutex>

#include "gridnav/gridnav_RMC.h"

#include "aurora/robot.h"
#include "aurora/robot_states.cpp"
#include "aurora/display.h"
#include "aurora/network.h"
#include "aurora/ui.h"

#include "ogl/event.cpp"
#include "osl/socket.cpp"

#include "osl/porthread.h" /* for threading */
#include "osl/porthread.cpp"

#include "aurora/simulator.h"
#include <iostream>


#include "../include/aurora/lunatic.h"
#include "../nanoslot/include/nanoslot/nanoslot_sanity.h"

// Global variables for lunatic data exchange with Arduinos via nanoslot
MAKE_exchange_nanoslot();
void arduino_setup_exchange()
{
    nanoslot_exchange &nano=exchange_nanoslot.write_begin();
    nano.sanity_check_size();
    nano.backend_heartbeat=0;
    exchange_nanoslot.write_end();
}

// Before we shut down, we need to unplug the exchange
void arduino_exit_exchange()
{
    nanoslot_exchange &nano=exchange_nanoslot.write_begin();
    nano.autonomy.mode=(int)0;
    nano.backend_heartbeat=0xDE;
    exchange_nanoslot.write_end();
}

void arduino_sensor_read(robot_base &robot)
{
    // Read sensor data from the exchange
    const nanoslot_exchange &nano=exchange_nanoslot.read();
    const auto &driveslot = nano.slot_D0;
    int left_wire = 0;
    int right_wire = 1;
    robot.sensor.DR1count= - driveslot.sensor.counts[right_wire];
    robot.sensor.DRstall =   driveslot.sensor.stall&(1<<right_wire);
    
    robot.sensor.DL1count=   driveslot.sensor.counts[left_wire];
    robot.sensor.DLstall =   driveslot.sensor.stall&(1<<left_wire);
    
    robot.sensor.heartbeat = driveslot.debug.packet_count;
    
    robot.sensor.encoder_raw=int(driveslot.sensor.raw);
    robot.sensor.stall_raw=int(driveslot.sensor.stall);
}

/*
 Convert -1.0 to +1.0 float power to discrete -100 to +100 motor percent.  
*/
nanoslot_motorpercent_t motor_scale(float power,const char *what)
{
    const float sanity_limit=4.0;
    if (power<-sanity_limit || power>sanity_limit || power!=power) {
        printf("Power %s ERROR: value %f insane, using 0\n", what,power);
        return 0;
    }
    if (power>1.0) power=1.0;
    if (power<-1.0) power=-1.0;
    
    const float send_limit=100.0;
    return (nanoslot_motorpercent_t)(send_limit*power);
}

void arduino_command_write(robot_base &robot)
{
    // Write commands to the exchange
    nanoslot_exchange &nano=exchange_nanoslot.write_begin();
    nano.autonomy.mode=(int)robot.state;
    
    auto &armslot = nano.slot_A0;
    armslot.command.motor[0]=-motor_scale(robot.power.spin,"spin");
    armslot.command.motor[1]=0; // spare
    armslot.command.motor[2]=motor_scale(robot.power.tilt,"tilt");
    armslot.command.motor[3]=motor_scale(robot.power.stick,"stick");
    
    auto &frontslot = nano.slot_F0;
    frontslot.command.motor[0]=-motor_scale(robot.power.dump,"dump");
    frontslot.command.motor[1]=-motor_scale(robot.power.fork,"fork");
    frontslot.command.motor[2]=0; // spare
    frontslot.command.motor[3]=motor_scale(robot.power.boom,"boom");
    
    auto &driveslot = nano.slot_D0;
    nanoslot_motorpercent_t L=motor_scale(robot.power.left,"left");
    nanoslot_motorpercent_t R=motor_scale(robot.power.right,"right");
    driveslot.command.motor[0]=-R;
    driveslot.command.motor[1]=-L;
    driveslot.command.motor[2]=-R;
    driveslot.command.motor[3]=-L;
    
    nano.slot_EE.command.LED=robot.power.right; // just for debugging
    
    nano.backend_heartbeat++;
    exchange_nanoslot.write_end();
}


MAKE_exchange_drive_encoders();
MAKE_exchange_stepper_request();
MAKE_exchange_plan_target();
MAKE_exchange_drive_commands();
//Needed for localization
MAKE_exchange_plan_current();
aurora::robot_loc2D currentLocation;

bool show_GUI=true;
bool simulate_only=false; // --sim flag
bool should_plan_paths=true; // --noplan flag
bool driver_test=false; // --driver_test, path planning testing

bool nodrive=false; // --nodrive flag (for testing indoors)

/* Bogus path planning target when we don't want any path planning to happen. */
aurora::robot_navtarget no_idea_loc(0.0f,0.0f,0.0f);

/** X,Y field target location where we drive to, before finally backing up */
aurora::robot_navtarget dump_target_loc(field_x_trough_center,field_y_trough_stop+20.0,field_angle_trough,
    20.0,30.0,70.0); // get back to starting area

aurora::robot_navtarget dump_align_loc(field_x_trough_center,field_y_trough_stop,field_angle_trough,
    20.0,10.0,5.0); // final alignment

/** X,Y field target location that we target for mining */
aurora::robot_navtarget mine_target_loc(field_x_trough_center,field_y_size-45,90,
    aurora::robot_navtarget::DONTCARE, 30.0,80.0);

/* Convert this unsigned char difference into a float difference */
float fix_wrap256(unsigned char diff) {
  if (diff>128) return diff-256;
  else return diff;
}

int last_Mcount=0;
int speed_Mcount=0;
float smooth_Mcount=0.0;

/**
  This class is used to localize the robot
*/
class robot_locator {
public:
  /** Merged location */
  robot_localization merged;
};


/**
 This class represents everything the back end knows about the robot.
*/
class robot_manager_t
{
public:
  robot_base robot; // overall integrated current state

  // Read (write?) copy of nano data
  nanoslot_exchange nano;
  
  robot_locator locator; // localization
  robot_telemetry telemetry; // next-sent telemetry value
  robot_command command; // last-received command
  robot_comms comms; // network link to front end
  robot_ui ui; // keyboard interface
  robot_realsense_comms realsense_comms;

  robot_simulator sim;
  int robot_insanity_counter = 0;

  robot_manager_t() {
    robot.sensor.limit_top=1;
    robot.sensor.limit_bottom=1;
    
    arduino_setup_exchange();
    atexit(arduino_exit_exchange);

    // Start simulation in random real start location
    sim.loc.y=80.0;
    sim.loc.x= (rand()%10)*20.0+100.0;
    sim.loc.angle=((rand()%8)*8)/360;
    sim.loc.percent=50.0;

    // Boot into BTI robot config
    robot.state=state_backend_driver;
    ui.power.torque=0;
  }

  // Do robot work.
  void update(void);
  
  
  void point_camera(int target) {
    if (simulate_only) {
      telemetry.autonomy.markers.beacon=target;
    }
    exchange_stepper_request.write_begin().angle=target;
    exchange_stepper_request.write_end();
  }


private:

  /* Use OpenGL to draw this robot navigation grid object */
  template <class grid_t>
  void gl_draw_grid(grid_t grid)
  {
    glPointSize(4.0f);
    glBegin(GL_POINTS);
    for (int y=0;y<rmc_navigator::GRIDY;y++)
    for (int x=0;x<rmc_navigator::GRIDX;x++)
    {
      int height=grid.at(x,y);
      if (height>0) {
        if (height>50) glColor3f(0.0f,1.0f,1.0f); // cyan trough / walls
        else if (height<15) glColor3f(1.0f,0.5f,1.0f); // purple very short
        else if (height<20) glColor3f(1.0f,0.0f,0.0f); // red short-ish
        else  glColor3f(1.0f,1.0f,1.0f); // white tall
        glVertex2f(
          rmc_navigator::GRIDSIZE*x,
          rmc_navigator::GRIDSIZE*y);
      }
    }
    glEnd();
  }

  // Autonomy support:
  double cur_time; // seconds since start of backend program
  double state_start_time; // cur_time when we entered the current state
  double mine_start_time; // cur_time when we last started mining
  double autonomy_start_time; // cur_time when we started full autonomy
  
  // If true, the mining head has been extended
  bool mining_head_extended=false;
  // If true, the mining head is down in the dirt
  bool mining_head_lowered=true;

  robot_state_t last_state;

  // Enter a new state (semi)autonomously
  void enter_state(robot_state_t new_state)
  {
    // Flush old planned path on state change
    exchange_plan_target.write_begin()=no_idea_loc;
    exchange_plan_target.write_end();

    if (new_state==state_autonomy) { autonomy_start_time=cur_time; }
    // if(!(robot.autonomous)) { new_state=state_drive; }

    // Log state timings to dedicate state timing file:
    static FILE *timelog=fopen("timing.log","w");
    fprintf(timelog,"%4d spent %6.3f seconds in %s\n",
      (int)(cur_time-autonomy_start_time),
      cur_time-state_start_time, state_to_string(robot.state));
    fflush(timelog);

    // Make state transition
    last_state=robot.state; // stash old state
    robot.state=new_state;
    robotPrintln("Entering new state %s",state_to_string(robot.state));
    state_start_time=cur_time;
  }

  // Advance autonomous state machine
  void autonomous_state(void);

  // Dump bucket encoder target a/d values


  // Limit this value to lie in this +- range
  double limit(double v,double range) {
    if (v>range) return range;
    if (v<-range) return -range;
    else return v;
  }

  // Run autonomous mining, if possible
  bool tryMineMode(void) {
    //if (drive_posture()) {    
    robot.power.tool=0.5; // TUNE THIS mining head rate
    robot.power.dump=0; // TUNE THIS lowering rate
    mining_head_lowered=true;
    
    
    return true;
  }

  // Set the mining head linear and dump linear to natural driving posture
  //  Return true if we're safe to drive
  bool drive_posture() {
    if(mining_head_lowered && cur_time-state_start_time <10)
      robot.power.dump = 1.0;
    if (sim.bucket>0.9) { // we're back up in driving range
      mining_head_lowered=false;
    }

    return true; // Kept for compatiiblity
  }

  // Autonomous driving rate:
  //  Returns 0-1.0 float power value.
  float drive_speed(float forward,float turn=0.0) {
    return 0.2; // confident but conservative
  }

  // Autonomous drive power from float values:
  //   "drive": forward +1.0, backward -1.0
  //   "turn": left turn +1.0, right turn -1.0 (like angle)
  void set_drive_powers(double forward,double turn=0.0)
  {
    double max_autonomous_drive=1.0; //<- can set a cap for debugging autonomous

    double drive_power=drive_speed(+1.0);
    double t=limit(turn*0.5,drive_power);
    double d=limit(forward*0.5,drive_power);
    double L=d-t;
    double R=d+t;
    robot.power.left= limit(L,max_autonomous_drive);
    robot.power.right=limit(R,max_autonomous_drive);
  }

  // Autonomous feeler-based backing up: drive backward slowly until both switches engage.
  //  Return true when we're finally backed up properly.
  bool back_up()
  {
    if(!(drive_posture())) {return false;}
    else {
      set_drive_powers(-0.1);

      // FIXME: back-up sensors?
      return true; // (robot.sensor.backL && robot.sensor.backR);
    }
  }

  //  Returns true once we're basically at the target location.
  bool autonomous_drive(const aurora::robot_navtarget &target) {
    if (!drive_posture()) return false; // don't drive yet
     vec2 cur(locator.merged.x,locator.merged.y); // robot location
    // Send off request to the path planner
    exchange_plan_target.write_begin()=target;
    exchange_plan_target.write_end();
    
    // Check for a response from the path planner
    static aurora::drive_commands last_drive={0.0f,0.0f};
    static double last_drive_update=0.0;
    const double max_drive_seconds=1.0; // drive this many long on an old plan
    
    if (exchange_drive_commands.updated()) {
      last_drive=exchange_drive_commands.read();
      last_drive_update=cur_time;
    }
    if (cur_time - last_drive_update<max_drive_seconds && last_drive.is_sane()) 
    {
      robot_insanity_counter = 0;
      if(last_drive.left < 0 && last_drive.right < 0)
      {
        point_camera(180);
      }
      else 
      {
        point_camera(0);
      }
      float autonomous_drive_power = .5 ; // scale factor for drive in autonomous
      robot.power.left =last_drive.left * autonomous_drive_power;
      robot.power.right=last_drive.right * autonomous_drive_power;
    }
    else 
    { // Fall back to greedy local autonomous driving: set powers to drive toward this field X,Y location
      robotPrintln("Invalid drive command dectected increasing robot insanity counter");
      robot_insanity_counter ++;
      // Tune this value based on path planning time on pi.
      if (robot_insanity_counter >= 10) 
      {
        robotPrintln("Robot insanity counter has reached 10.. exiting autonomy");
        enter_state(state_drive);
      }
      
    }

    return target.matches(locator.merged); // we're basically there
  }

  // Force this angle (or angle difference) to be between -180 and +180,
  //   by adding or subtracting 360 degrees.
  void reduce_angle(double &angle) {
    while (angle>=180) angle-=360; // reduce
    while (angle<-180) angle+=360; // reduce
  }

  // Autonomous turning: rotate robot so it's facing this direction.
  //  Returns true once we're basically at the target angle.
  // ToDo: Point camera to an appropriate angle as you turn
  bool autonomous_turn(double angle_target_deg=0.0,bool do_posture=true)
  {
    if (do_posture) { if (!drive_posture()) return false; } // don't drive yet 
    double angle_err_deg=locator.merged.angle-angle_target_deg;
    reduce_angle(angle_err_deg);
    robotPrintln("Autonomous turn to %.0f from %.0f deg\n",
      angle_target_deg, locator.merged.angle);
   
    double turn=angle_err_deg*0.1; // proportional control
    double maxturn=drive_speed(0.0,1.0);
    turn=limit(turn,maxturn);
    set_drive_powers(0.0,turn);
    return fabs(angle_err_deg)<5.0; // angle error tolerance
  }

  // Make sure we're still facing the collection bin.  If not, pivot to face it.
  bool check_angle() {
    if (locator.merged.percent<20.0) return true; // we don't know where we are--just keep driving?
    double target=180.0/M_PI*atan2(locator.merged.y+200.0,locator.merged.x);
    double err=locator.merged.angle-target;
    robotPrintln("check_angle: cur %.1f deg, target %.1f deg",locator.merged.angle,target);
    
    reduce_angle(err);
    if (fabs(err)<10.0) return true; // keep driving--straight enough
    else return autonomous_turn(target,false); // turn to face target
  }
};


// Return true if the mining head is stalled (according to our sensors
bool is_stalled(const robot_base &robot) {
  return robot.sensor.Mstall;
}


/* Utility function: slow down speed as cur approaches target
  Returns false if already past target.
*/
bool speed_limit(int &howfast,int cur,int target,int dir=+1)
{
  int dist_left=(target-cur)*dir;
  if (dist_left<=0) {
    return false;
  }
  int max_speed=10+dist_left/5;
  if (howfast>max_speed) howfast=max_speed;
  return true;
}


void robot_manager_t::autonomous_state()
{
  robot.power.stop(); // each state starts from scratch

  double time_in_state=cur_time-state_start_time;
  robotPrintln("In state %s for %.1f seconds...\n", state_to_string(robot.state), time_in_state);

  // full autonomy start
  if (robot.state==state_autonomy) {
    robot.autonomous=true;
    enter_state(state_scan);
  }
  
  // scan terrain before mining
  else if (robot.state==state_scan)
  {
    
    if(time_in_state<2.0) // stare at terrain
    {
      // FIXME: activate arm and depth camera
    }
    else{
      enter_state(state_mine_start);
    }
  }
  //state_mine_lower: enter mining state
  else if (robot.state==state_mine_start) {
    tryMineMode();
    mine_start_time=cur_time; // update mine start time
    enter_state(state_mine);
  }
  else if (robot.state==state_mine)
  {
    if (!tryMineMode()) { // too high to mine (sanity check)
      robot.power.dump=-1.0f; // lower bucket
      mining_head_lowered=true;
    }

    double mine_time=cur_time-mine_start_time;
    double mine_duration=25.0;
    if(mine_time>mine_duration)
    {
        enter_state(state_mine_finish);
    } // done mining
    
    if (robot.sensor.Mstall) enter_state(state_mine_stall);
  }

  // state_mine_stall: Detect mining head stall. Raise head until cleared
  else if (robot.state==state_mine_stall)
  {
    tryMineMode(); // Start PID based mining
    if(robot.sensor.Mstall && time_in_state<1)
    {
      robot.power.boom=-1.0f; // retract bucket
    }
    else {enter_state(state_mine);} // not stalled? Then back to mining
  }

  //Done mining: Raise scoop
  else if (robot.state==state_mine_finish)
  {
    if(drive_posture())
      enter_state(state_weigh);
  }
  
  //Weigh material beofre leaving pit
  else if (robot.state==state_weigh)
  {
    if(time_in_state<2.0) // let settle
    {
      // FIXME: tilt scoop so it's level
      // FIXME: record loads on left and right
    }
    if(drive_posture())
      enter_state(state_haul_out);
  }

  // Drive back to trough
  else if (robot.state==state_haul_out)
  {
    if (autonomous_drive(dump_target_loc) 
     || locator.merged.y<dump_target_loc.y+50.0)
    {
      enter_state(state_haul_dump);
    }
  }
  // Dump material
  else if (robot.state==state_haul_dump)
  { 
    // FIXME: do this
    enter_state(state_haul_back);
  }
  // Drive back into pit
  else if (robot.state==state_haul_back)
  {
    if (autonomous_drive(mine_target_loc))
    {
      enter_state(state_scan);
    }
  }
  
  // Stow the robot (like for moving it)
  else if (robot.state==state_stow)
  {
    if(mining_head_lowered)
      drive_posture();
    if(time_in_state<20)
      robot.power.dump=-1.0f;
    enter_state(state_stowed);

  }
  else if (robot.state==state_stowed)
  {
    /* wait here forever */
  }
  else
  { // what?  unrecognized state?!  manual mode...
    robotPrintln("Autonomy: unrecognized state %s (%d)!?\n",state_to_string(robot.state), robot.state);
    enter_state(state_drive);
  }

  if (nodrive)
  { // do not drive!  (except for state_drive)
    robotPrintln("NODRIVE");
    set_drive_powers(0.0,0.0);
  }
}




robot_manager_t *robot_manager;

unsigned int video_texture_ID=0;

void robot_manager_t::update(void) {
  cur_time=0.001*glutGet(GLUT_ELAPSED_TIME);

#if 1 /* enable for backend UI: dangerous, but useful for autonomy testing w/o frontend */
  // Keyboard control
  ui.update(oglKeyMap,robot);

  // Click to set state:
  if (robotState_requested<state_last) {
    robot.state=robotState_requested;
    robotPrintln("Entering new state %s (%d) by backend UI request",
      state_to_string(robot.state),robot.state);
    robotState_requested=state_last; // clear UI request
  }
#endif

// Show real and simulated robots
//needs to be updated for new data exchange?
  robot_display(locator.merged);

	robot_display_autonomy(telemetry.autonomy);

// Check for a command broadcast (briefly)
  int n;
  while (0!=(n=comms.available(10))) {
    if (n==sizeof(command)) {
      comms.receive(command);
      if (command.command==robot_command::command_STOP)
      { // ESTOP command
        enter_state(state_STOP);
        robot.power.stop();
        robotPrintln("Incoming STOP command");
      }
      else if (command.command==robot_command::command_state)
      {
        if (command.state>=state_STOP && command.state<state_last)
        {
          robot.state=(robot_state_t)command.state;
          telemetry.ack_state=robot.state;
          robotPrintln("Entering new state %s (%d) by frontend request",
            state_to_string(robot.state),robot.state);
        } else {
          robotPrintln("ERROR!  IGNORING INVALID STATE %d!!\n",command.state);
        }
      }
      else if (command.command==robot_command::command_power)
      { // manual driving power command
        robotPrintln("Incoming power command: %d bytes",n);
        if (robot.state==state_drive)
        {
          robot.autonomous=false;
          robot.power=command.power;
        }
        else
        {
          robotPrintln("IGNORING POWER: not in drive state\n");
        }
      }
      if (command.realsense_comms.command=='P')
      {
        point_camera(command.realsense_comms.requested_angle);
      }
    } else {
      robotPrintln("ERROR: COMMAND VERSION MISMATCH!  Expected %d, got %d",
        sizeof(command),n);
    }

  }

// Perform action based on state recieved from FrontEnd
  //E-Stop command
  if(robot.state==state_STOP)
  {// All stop
    robot.power.stop();
    state_start_time=cur_time;
  }
  else if (robot.state==state_drive)
  { // do nothing-- already got power command
    state_start_time=cur_time;
  }
  else if (robot.state==state_backend_driver)
  { // set robot power from backend UI
    robot.power=ui.power;
  }
  else if (robot.state>=state_autonomy) { // autonomous mode!
    autonomous_state();
  }


  // Send commands to Arduino
  robot_sensors_arduino old_sensor=robot.sensor;
    
  if (simulate_only) { // build fake arduino data
    robot.status.arduino=1; // pretend it's connected
    robot.sensor.McountL=0xff&(int)sim.Mcount;
    robot.sensor.Rcount=0xffff&(int)sim.Rcount;
    robot.sensor.DL1count=0xffff&(int)sim.DLcount;
    robot.sensor.DR1count=0xffff&(int)sim.DRcount;
    robot.sensor.limit_top=0;
    robot.sensor.limit_bottom=0;
  }
  else { // Send data to/from real arduino
    arduino_sensor_read(robot);
    nano=exchange_nanoslot.read();
  }
  
  
  if (nano.slot_A0.sensor.stop && robot.state!=state_STOP) {
    enter_state(state_STOP);
    robot.power.stop();
    robotPrintln("Slot A0 STOP command");
  }
  
  if (robot.state==state_mine) 
  { // TESTING ONLY: keep the front tool level
    const float gain=5.0;
    float drive=nano.slot_A1.sensor.imu[0].acc.x;
    float rate=0.0;
    float pid = gain*drive + rate;
    int send=ui.limit(pid,30); //<- calmer driving
    robot.power.tilt=send;
    if (fabs(drive)<5) robot.power.stop(); //<- deadband, bigger than noise
  }
  
  speed_Mcount=robot.sensor.McountL-last_Mcount;
  float smoothing=0.3;
  smooth_Mcount=speed_Mcount*smoothing + smooth_Mcount*(1.0-smoothing);
  robotPrintln("Mcount smoothed: %.1f, speed %d\n",
     smooth_Mcount, speed_Mcount);
  last_Mcount=robot.sensor.McountL;

  // Fake the bucket sensor from the sim (no hardware sensor for now)
  robot.sensor.bucket=sim.bucket*(950-179)+179;

  // some values for the determining location. needed by the localization.
  // FIXME: tune these for real tracks!
  //float fudge=1.06; // fudge factor to make blue printed wheels work mo betta
  //float drivecount2cm=fudge*6*5.0/36; // cm of driving per wheel encoder tick == pegs on drive sprockets, space between sprockets, 36 encoder counts per revolution
  float drivecount2cm = 10.0/40.0;
  float driveL = fix_wrap256(robot.sensor.DL1count-old_sensor.DL1count)*drivecount2cm;
  float driveR = fix_wrap256(robot.sensor.DR1count-old_sensor.DR1count)*drivecount2cm;
  
  // Update drive encoders data exchange
  static aurora::drive_encoders::real_t totalL = 0.0; //<- hacky!  Need to total up distance
  static aurora::drive_encoders::real_t totalR = 0.0;
  totalL -= driveL;
  totalR += driveR;
  aurora::drive_encoders enc;
  enc.left =totalL;
  enc.right=totalR;
  exchange_drive_encoders.write_begin()=enc;
  exchange_drive_encoders.write_end();
  
  locator.merged=exchange_plan_current.read();


// Send out telemetry
  arduino_command_write(robot);

  static double last_send=0.0;
  if (cur_time>last_send+0.050)
  {
    last_send=cur_time;
    // robotPrintln("Sending telemetry, waiting for command");
    telemetry.count++;
    telemetry.state=robot.state; // copy current values out for send
    telemetry.status=robot.status;
    telemetry.sensor=robot.sensor;
    telemetry.power=robot.power;
    telemetry.loc=locator.merged; 
    locator.merged.percent*=0.999; // slowly lose location fix

    comms.broadcast(telemetry);
  }


  static double last_time=0.0;
  double dt=cur_time-last_time;
  if (dt>0.1) dt=0.1;
  last_time=cur_time;

  if (locator.merged.percent>=10.0)  // make sim track reality
    sim.loc=locator.merged;

  if (simulate_only) // make reality track sim
  {
    float view_robot_angle=0;
    float beacon_FOV=30; // field of view of beacon (markers)
    if (beacon_FOV>fabs(telemetry.autonomy.markers.beacon - view_robot_angle))
      locator.merged.percent+=10.0;
    locator.merged.percent=std::min(100.0,locator.merged.percent*(1.0-dt));
  }
  sim.simulate(robot.power,dt);
}


void display(void) {
  robot_display_setup(robot_manager->robot);

  robot_manager->update();

  if (video_texture_ID) {
    glTranslatef(field_x_GUI+350.0,100.0,0.0);
    glScalef(300.0,200.0,1.0);
    glBindTexture(GL_TEXTURE_2D,video_texture_ID);
    glEnable(GL_TEXTURE_2D);
    glBegin(GL_QUAD_STRIP);
    glTexCoord2f(0.0,0.0); glVertex2f(0.0,0.0);
    glTexCoord2f(1.0,0.0); glVertex2f(+1.0,0.0);
    glTexCoord2f(0.0,1.0); glVertex2f(0.0,+1.0);
    glTexCoord2f(1.0,1.0); glVertex2f(+1.0,+1.0);
    glEnd();
    glBindTexture(GL_TEXTURE_2D,0);
  }

  glutSwapBuffers();
  glutPostRedisplay();
}

int main(int argc,char *argv[])
{
  // setenv("DISPLAY", ":0",1); // never forward GUI over X
  glutInit(&argc,argv);

  // Set screen size
  int w=1200, h=700;
  for (int argi=1;argi<argc;argi++) {
    if (0==strcmp(argv[argi],"--sim")) {
      simulate_only=true;
      if (argi+1<argc) srand(atoi(argv[++argi])); // optional seed argument
      else srand(1);
    }
    else if (0==strcmp(argv[argi],"--noplan")) {
      should_plan_paths=false;
    }
    else if (0==strcmp(argv[argi],"--driver_test")) {
      simulate_only=true;
      driver_test=true;
    }
    else if (0==strcmp(argv[argi],"--nogui")) { // UNTESTED!
      show_GUI=false;
    }
    else if (0==strcmp(argv[argi],"--nodrive")) {
      nodrive=true;
    }
    else if (2==sscanf(argv[argi],"%dx%d",&w,&h)) {}
    else {
      printf("Unrecognized argument '%s'!\n",argv[argi]);
      exit(1);
    }
  }

  robot_manager=new robot_manager_t;
  robot_manager->locator.merged.y=100;
  if (simulate_only) robot_manager->locator.merged.x=150;

  if (show_GUI) {
    glutInitDisplayMode(GLUT_RGBA + GLUT_DOUBLE);
    glutInitWindowSize(w,h);
    glutCreateWindow("Robot Backend");
    robotMainSetup();

    glutDisplayFunc(display);
    glutMainLoop();
  }
  else
  { // no-GUI version
    while (true) {
      robot_manager->update();
    }
  }
  return 0;
}

