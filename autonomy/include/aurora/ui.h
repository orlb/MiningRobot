/**
  Aurora Robotics keyboard user interface code.
  
  This is used by both frontend (drive) and backend (backend_driver)
  to convert keyboard and joystick input into robot power commands.
  
  This is the place to add new user interface modes and features.

  Orion Sky Lawlor, lawlor@alaska.edu, 2014-03-23 (Public Domain)
*/
#ifndef __AURORA_ROBOTICS__UI_H
#define __AURORA_ROBOTICS__UI_H

#include "ogl/event.h" /* for joystick */

#ifdef MSL
	#include "msl/joystick.hpp"
	#include "msl/joystick.cpp"
#endif
#include <iostream>

/**
 Keyboard-based user interface for robot.
 Inherits from robot_power to control those variables.
*/
class robot_ui : private robot_power {
public:
	float driveLimit=1.0; /* robot driving power (0.0 - 1.0) */
	float toolLimit=0.5; /* tool power */
	
	int joyMode=0; // joystick mode selected
	
	robot_power power; // Last output power commands
	
	robot_realsense_comms realsense_comms;
	#ifdef MSL
		msl::joystick_t* joystick;
	#endif

	// Human-readable description of current state
	std::string description;

	bool js_button(int button,const std::string& label)
	{
		#ifdef MSL
			if(button==1)
				button=15;
			else if(button==2)
				button=12;
			else if(button==3)
				button=14;
			else if(button==4)
				button=13;
			else if(button==5)
				button=0;
			else if(button==6)
				button=3;
			else
				button=-1;
			if(button>-1&&joystick!=NULL&&joystick->good()&&button<(int)joystick->button_count())
				return joystick->button(button);
		#else
			return oglButton(button,label.c_str());
		#endif
		return false;
	}

	bool js_button_once(const int button,const std::string& label)
	{
		#ifdef MSL
			return js_button(button,label);
		#else
			return oglButtonOnce(button,label.c_str());
		#endif
		return false;
	}

	float js_axis(int axis,const std::string& label)
	{
		#ifdef MSL
			--axis;
			if(axis==2)
				axis=3;
			else if(axis==3)
				axis=2;
			std::cerr<<axis<<"="<<joystick->axis(axis)<<std::endl;
			if(joystick!=NULL&&joystick->good()&&axis<(int)joystick->axis_count())
				return joystick->axis(axis);
		#else
			return oglAxis(axis,label.c_str());
		#endif
		return false;
	}

	void stop(void) {
		robot_power::stop();
		power.stop();
		description="Sending STOP";
	}

	// Respond to these keystrokes.
	//  The "keys" array is indexed by the character, 0 for up, 1 for down.
	void update(int keys[],const robot_base &robot);

	robot_ui()
	{
		#ifdef MSL
		joystick=NULL;
		#endif
		stop();
		description="Starting up";
	}

	// Clamp this float within this maximum range
	float limit(float v,float maxPower) const {
		if (v>maxPower) v=maxPower;
		if (v<-maxPower) v=-maxPower;
		return v;
	}

	// Convert a raw float to a motor command, with this maximum range
	byte toMotor(float v,float maxPower) const {
		v=limit(v,maxPower);
		int iv=(int)(v*100);
		if (iv<-100) iv=-100;
		if (iv>100) iv=100;
		return iv;
	}
	
	
    /* Set this keyboard-controlled power limit--
      Use P + number keys to set drive power, in percent:
        P-1 = 10%, P-2=20%, etc. 
      Returns a human-readable description of the current limit.
    */
    std::string setPowerLimit(int keys[],char lowercase,char uppercase,const std::string &description,float &limit)
    {
	    if (keys[lowercase]||keys[uppercase]) 
	        for (int num=1;num<=9;num++)
	            if (keys['0'+num]) 
	                limit=0.1f*num;
	    
	    return "\n  "+description+": "+std::to_string(limit)+"\n";
    }

};


void robot_ui::update(int keys[],const robot_base &robot) {
	#ifdef MSL
		if(joystick==NULL)
		{
			auto joysticks=msl::joystick_t::list();
			if(joysticks.size()>0)
			{
				joystick=new msl::joystick_t(joysticks[0]);
				joystick->open();
			}
		}
		if(joystick!=NULL&&!joystick->good())
		{
			delete joystick;
			joystick=NULL;
		}
	#endif

	static int keys_last[256];
	int keys_once[256]; // key down only *one* time per press
	for (int i=0;i<256;i++) {
		keys_once[i]= (keys[i] && !keys_last[i]);
		keys_last[i]=keys[i];
	}
	description="UI:\n";

// Power limits:
	float scoopLimit=1.0; // limit on fork & dump
	float armLimit=1.0; // limit on boom, stick, tilt, spin

// Prepare a command:
	if (keys[' ']) { // spacebar--full stop
		stop();
		robotState_requested=state_STOP;
		return; // don't do anything else.  Just stop.
	}
	// else spacebar not down--check other keys for manual control
	
	left=right=0.0;
	
	fork=dump=0.0;
	
	boom=stick=tilt=0.0; 
	spin=0.0;
	
	tool=0.0;

    // These are subtle state variables (why?)
	static bool joyDrive=false; 
	bool joyDone=false; 


	/* Uses the left analog stick for driving*/
	float forward=-js_axis(2,"Go Forward or Reverse"); // left Y
	float turn=js_axis(1,"Turn Left or Right"); //left X, scaled for gentle turns

    /* Use the right analog stick for modal control */
    float ry=-js_axis(3,"Vertical by mode");
    float rx=js_axis(4,"Horizontal by mode");

	if(forward!=0.0 || turn!=0.0 || ry!=0 || rx!=0)
	{
		joyDrive=true; // using joystick
	}
	else {joyDone=true;}
	
	
	if(js_button(1,"")) joyMode=1;
	if(js_button(2,"")) joyMode=2;

	if (js_button(3,"Stop")) 
	{
	    joyMode=0;
		stop();
		robotState_requested=state_STOP;
	}
	
	if(js_button(3,"")) joyMode=3;
	if(js_button(4,"")) joyMode=4;
	if(js_button(5,"")) joyMode=5;
	if(js_button(6,"")) joyMode=6;

    switch (joyMode) {
    case 0: break;
    case 1: 
        description += " fork-dump ";
        fork=ry;
        dump=-rx;
        break;
    case 2:
        description += " stick-boom ";
        stick=ry;
        boom=rx;
        break;
    case 4:
        description += " stick-tilt ";
        stick=ry;
        tilt=rx;
        break;
    case 5:
        description += " tilt-spin ";
        tilt=ry;
        spin=rx;
        break;
    case 6: 
        description += " tilt-mine";
        tilt=ry;
        tool=0.5;
        break;
    default: break;
    }


	if(joyDrive)
	{
		left=driveLimit*(forward+turn);
		right=driveLimit*(forward-turn);
        
        
		description += "joystick\n";
	}
	joyDrive=!joyDone;

// Adjust power limits
	description+=setPowerLimit(keys,'p','P',"Drive power",driveLimit);
	description+=setPowerLimit(keys,'t','T',"Tool power",toolLimit);
	

// Drive keys:
	float speed=driveLimit;
    if(keys['w']||keys['W'])
    {
        left+=speed;
        right+=speed;
    }
    if(keys['s']||keys['S'])
    {
        left-=speed;
        right-=speed;
    }
    if(keys['a']||keys['A'])
    {
        left-=speed;
        right+=speed;
    }
    if(keys['d']||keys['D'])
    {
        left+=speed;
        right-=speed;
    }
    if(keys_once['t']||keys_once['T'])
    {
	    power.torque=~power.torque;
    }

	if(power.torque==0)
		description+="  torque control\n";
	else
		description+="  speed control\n";


// Limit powers, and write them to the struct
	left=limit(left,driveLimit);
	right=limit(right,driveLimit);
	
	fork=limit(fork,scoopLimit);
	dump=limit(dump,scoopLimit);
	
	boom=limit(boom,armLimit);
	stick=limit(stick,armLimit);
	tilt=limit(tilt,armLimit);
	spin=limit(spin,armLimit);
	
	tool=limit(tool,toolLimit);

	// Use a complementary filter to smooth our motion commands, for less jerky operation
    power.blend_from(*this,0.2);
    
	robotPrintln("Arduino Heartbeat: %d",robot.sensor.heartbeat);
	robotPrintLines(description);
	power.print("UI power");
}



#endif

