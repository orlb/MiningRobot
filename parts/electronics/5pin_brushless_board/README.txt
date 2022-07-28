5-pin Brushless motor control board
Designed as a direct plug-compatible replacement for the UN178 brushed motor controller, to let us mix and match brushed/brushless motors. 
Designed in summer 2022 by Daniel Kling

Designed in Autodesk EAGLE (free version limited to 12.4 square inches, this board is under 5 square inches). 


Board input: same male DIP (2.54mm pin spacing) 2x5 pin grid as the UN178. 

Board mounting: same M3 screw holes as UN178, at least the left two.  Probably doesn't need to be as big as the UN178.

Board style: ideally large trace (0.8mm trace width minimum), single layer back side copper only to allow local fabrication on a CNC mill if needed. Can use front side copper traces as occasional jumpers, which just work on a commercially fabricated board, and we can make ourselves using jumpers. 

Board outputs: probably a row of 3-pin male DIP 1x3 pin headers, which would let us plug in R/C ESC or servo headers directly.  Should be able to do like 5 outputs for different motors or encoders/sensors/etc.

5-pin output pinout:
	GND pin shared across all outputs
	5v pin shared across all outputs, fed from a non-PWM Arduino I/O pin?  (I don't think I have a 5V on the header right now)
	Signal pin, from individual Arduino I/O pins
	(so far, this is a 3-pin R/C servo connector)
	12/24V + motor supply pin (for motors only needing a few amps)
	gnd motor supply pin

Dupont pins are limited to 1-3 amps maximum (depending on the exact pin). 
For big motors, like drive or mine motors, they'll only take the 3-pin R/C plug and run dedicated motor supply wires. 




