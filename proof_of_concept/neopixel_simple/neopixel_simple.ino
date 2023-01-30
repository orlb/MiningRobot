/* 
 Drive NeoPixel color display:
   Recommend approx 500 ohm dropping resistor on data line
   Recommend approx 100 uF capacitor between 5v and gnd
   Each pixel actually pulls only 30mA, peak is 60 mA (bigger with brighter colors)

*/
#include <Adafruit_NeoPixel.h>
#define NeoPIN        A1
#define NUMPIXELS 4
Adafruit_NeoPixel pixels(NUMPIXELS, NeoPIN, NEO_GRB + NEO_KHZ400);

void setup() {
  // put your setup code here, to run once:
  pixels.begin();
}

// Update the neopixel colors to reflect this state
void sendNeopixels(int state,int animationFrame)
{
  // Animate even/odd pattern of LEDs:
  unsigned char animate=(animationFrame&1)?0x55:0xAA;
  unsigned char animateOut=0x0; // black out marked pixels
  unsigned char animateDim=0x0; // dim marked pixels

  const int bright=31, medium=23; // subtle, <20mA
  //const int bright=127, medium=63; // some flicker
  //const int bright=255, medium=191; // painfully bright (needed for sunlight?)
  
  int32_t baseColor=pixels.Color(0, 50, 0); // bright green
  if (state==0) {
    baseColor=pixels.Color(0, medium, bright); // safe blue
    // steady color (less annoying in safe mode)
  } 
  else if (state==1 || state==2) {
    baseColor=pixels.Color(medium+1,medium+1,0); // manual yellow
    animateDim=animate;
  }
  else /* state>2, autonomy */ {
    baseColor=pixels.Color(bright, 0, 0); // autonomy: caution red
    animateOut=animate;
  }
  for(int i=0; i<NUMPIXELS; i++) {
    int32_t c=baseColor;
    if (animateOut & 1<<i) c=0;
    if (animateDim & 1<<i) c=baseColor>>2;//<- color must be multiple of 4 for this to work!
    pixels.setPixelColor(i, c);
  }
  pixels.show();
}

int state=0;
int frame=0;
void loop() {
  sendNeopixels(state,frame++);
  if (frame>6) {
    frame=0;
    state++;
    if (state>4) state=0;
  }
  delay(1000);
}
