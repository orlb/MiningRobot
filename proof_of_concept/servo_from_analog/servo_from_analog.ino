/*
 Convert a potentiometer to a voltage divider,
 and send a servo speed based on the provided voltage.
 
 Wiring: 
    Potentiometer ends to plus/minus pins
    Potentiometer wiper to wiper pin
    
    Servo white wire to Arduino pin 4
    Servo red wire to Arduino Vin *if* you want to power from it
    Servo black wire to Arduino ground
 */
#include <Servo.h>

int servo_pin=4; // servo white wire to this Arduino pin

int pot_plus=A0; // positive end of potentiometer
int pot_wiper=A1; // wiper (signal) from potentiometer
int pot_minus=A2; // negative end of potentiometer

Servo servo;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(30); //<- hack to make parseInt less laggy.
  Serial.println("Servo driver from potentiometer");

  servo.attach(servo_pin); 
  servo.writeMicroseconds(1500); // needs center at startup
  delay(3500); // wait for ESC to "arm" before sending commands

  pinMode(pot_plus,OUTPUT); digitalWrite(pot_plus,HIGH);
  pinMode(pot_minus,OUTPUT); digitalWrite(pot_minus,LOW);

}

int servo_width=1500; // current time, in microseconds
void loop() {
  bool check=true;
  if (check)
  {
    digitalWrite(pot_plus,HIGH);
    int w1=analogRead(pot_wiper);
    digitalWrite(pot_plus,LOW);
    int w0=analogRead(pot_wiper);
    
    servo_width = w1*2 + 500;
  }
  servo.writeMicroseconds(servo_width);// width in microseconds, usually 1000-2000 range

  Serial.println(servo_width);

  delay(10);
}
