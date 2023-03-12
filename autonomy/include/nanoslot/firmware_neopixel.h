/* 
 Very opinionated function to display the robot state to a set of neopixel LEDs.
 
 Usage: select the drive pin via a #define before including this header.
#define NeoPIN        A1

 Drive NeoPixel color display:
   Recommend approx 500 ohm dropping resistor on data line
   Recommend approx 100 uF capacitor between 5v and gnd
   Each pixel actually pulls only 30mA, peak is 60 mA (bigger with brighter colors)

*/
#ifndef __NANOSLOT_FIRMWARE_NEOPIXEL_H
#define __NANOSLOT_FIRMWARE_NEOPIXEL_H

#include <Adafruit_NeoPixel.h>
#define NUMPIXELS 4
Adafruit_NeoPixel neopixels(NUMPIXELS, NeoPIN, NEO_GRB + NEO_KHZ400);


// Update the neopixel colors to reflect this state
void sendNeopixels(int state,int animationFrame)
{
  // Animate even/odd pattern of LEDs:
  unsigned char animate=(animationFrame&1)?0x55:0xAA;
  unsigned char animateOut=0x0; // black out marked pixels
  unsigned char animateDim=0x0; // dim marked pixels

  //const int bright=31, medium=23; // subtle, <20mA
  const int bright=127, medium=63; // some flicker
  //const int bright=255, medium=191; // painfully bright (needed for sunlight?)
  
  int32_t baseColor=neopixels.Color(0, 50, 0); // bright green
  if (state==0) {
    baseColor=neopixels.Color(0, medium, bright); // safe blue
    // steady color (less annoying in safe mode)
  } 
  else if (state==1 || state==2) {
    baseColor=neopixels.Color(medium+17,medium+1,0); // manual yellow
    animateDim=animate;
  }
  else /* state>2, autonomy */ {
    baseColor=neopixels.Color(bright, 0, 0); // autonomy: caution red
    animateOut=animate;
  }
  for(int i=0; i<NUMPIXELS; i++) {
    int32_t c=baseColor;
    if (animateOut & 1<<i) c=0;
    if (animateDim & 1<<i) c=baseColor>>2;//<- color must be multiple of 4 for this to work!
    neopixels.setPixelColor(i, c);
  }
  neopixels.show();
}


int lastState=123;
milli_t lastNeopixel=0; // milli at last call to update
milli_t betweenNeopixels=1024; // update every this many ms
milli_t frameNeopixel=0; // frame counter
void updateNeopixels(int state)
{
  if (lastState!=state || (milli - lastNeopixel)>betweenNeopixels)
  { // time to update the display
    lastState=state;
    lastNeopixel=milli;
    sendNeopixels(state,frameNeopixel++);
  }
}

#endif

