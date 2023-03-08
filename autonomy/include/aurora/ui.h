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
	
	enum {
	    joyModeSTOP=0, // don't drive
	    joyModeLow=1, // bottom of robot
	    joyModeMed=2, // middle of robot
	    joyModeHigh=3, // top of robot (tool)
	} joyMode=joyModeLow; // joystick mode selected
	
	robot_power power; // Last output power commands
	
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
	
	// Filter a raw joystick axis (remove jittery deadband)
	float filter_axis(float v) {
	    const float minV=0.03f;
	    if (v>minV) {
	        return v-minV;
	    }
	    else if (v<-minV) {
	        return v+minV;
	    }
	    else /* v is tiny, between min and -min */ {
	        return 0.0f;
	    }
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
	float forward=0.0, turn=0.0; //<- turned into left and right
	
	fork=dump=0.0;
	
	boom=stick=tilt=0.0; 
	spin=0.0;
	
	tool=0.0;

    // These state variables are used to display whether you're on a joystick.
	static bool joyDrive=false; 
	bool joyDone=false; 
/*
Joysticks have different axis and button numbering:

 Logitech Gamepad F310:
    axis 1: left analog X
    axis 2: left analog Y
    axis 3: left trigger analog
    axis 4: right analog X
    axis 5: right analog Y    
    axis 6: right trigger analog
        (Releasing trigger analog drops it to -max, but we start it at 0)
    
    button 1: A (green)
    button 2: B (red)
    button 3: X (blue)
    button 4: Y (orange)
    button 5: left top trigger
    button 6: right top trigger
 
 Saitek PLC Cyborg Force Rumble Pad
    axis 1: left analog X
    axis 2: left analog Y
    axis 3: right analog Y
    axis 4: right analog X
    
    buttons 1-6 as labelled
    button 7: left trigger
    button 8: right trigger
*/
    // Defaults are for Logitech
    int axis_lx=1, axis_ly=2;
    int axis_rx=4, axis_ry=5;
    int button_stop=3, button_low=1, button_med=2, button_high=4;
//    int button_topleft=5, button_topright=6; // shoulder buttons
    
    const char *joystick_name=oglJoystickName();
    if (joystick_name[0]=='S') { // Saitek
        axis_ry=3; // for some reason this uses axis 3
        button_stop=1; button_low=3; button_med=4; button_high=2;
//        button_topleft=7; button_topright=8;
    }

	/* Read left analog stick X and Y axes*/
	float ly=filter_axis(-js_axis(axis_ly,"")); 
	float lx=filter_axis(js_axis(axis_lx,"")); 

    /* Read the right analog stick */
    float ry=filter_axis(-js_axis(axis_ry,""));
    float rx=filter_axis(js_axis(axis_rx,""));

	if(lx!=0.0 || ly!=0.0 || ry!=0 || rx!=0)
	{
		joyDrive=true; // using joystick
	}
	else { // joystick stopped
	    joyDone=true;
	}
	
	// Pressing a button changes the mode persistently
	if (js_button(button_stop,"")) joyMode=joyModeSTOP;
	if (js_button(button_low,"")) joyMode=joyModeLow;
	if (js_button(button_med,"")) joyMode=joyModeMed;
	if (js_button(button_high,"")) joyMode=joyModeHigh;

    switch (joyMode) {
    case joyModeSTOP: default: 
		stop();
		robotState_requested=state_STOP;
		break;
    case joyModeLow: 
        description += " Low: drive fork-dump ";
        forward=ly;
        turn=lx;
        fork=ry;
        dump=-rx;
        break;
    case joyModeMed:
        description += " Med: drive stick-boom ";
        forward=ly;
        turn=lx;
        stick=ry;
        boom=rx;
        break;
    case joyModeHigh:
        description += " High: spin tilt-mine ";
        spin=lx;
        tilt=ry;
        tool=rx;
        break;
    }

	if(joyDrive)
	{   
		description += "joystick\n";
	}
	joyDrive=!joyDone;

// Adjust power limits
	description+=setPowerLimit(keys,'p','P',"Drive power",driveLimit);
	description+=setPowerLimit(keys,'t','T',"Tool power",toolLimit);
	

// Drive keys:
    if(keys['w']||keys['W']) forward+=1.0;
    if(keys['s']||keys['S']) forward-=1.0;
    if(keys['a']||keys['A']) turn-=1.0;
    if(keys['d']||keys['D']) turn+=1.0;
    
	left=driveLimit*(forward+turn);
	right=driveLimit*(forward-turn);
	
    if(keys_once['t']||keys_once['T'])
    {
	    power.torque=~power.torque;
    }

	if(power.torque==0)
		description+="  torque control\n";
	else
		description+="  speed control\n";


// Limit powers, and write them to the output struct
	left=limit(left,driveLimit);
	right=limit(right,driveLimit);
	
	fork=limit(fork,scoopLimit);
	dump=limit(dump,scoopLimit);
	
	boom=limit(boom,armLimit);
	stick=limit(stick,armLimit);
	tilt=limit(tilt,armLimit);
	spin=limit(spin,armLimit);
	
	tool=limit(tool,toolLimit);

	// Blend in power to smooth our motion commands, for less jerky operation
    power.blend_from(*this,0.2);
    
	robotPrintLines(description);
	power.print("UI power");
}



#endif

