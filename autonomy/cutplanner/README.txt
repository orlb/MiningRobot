This tool does computational geometry for planning excavation passes:
specifically planning a path for the robot's mining head that will cut an even slice of material (cutting too shallow wastes time, cutting too deep can stall out the head).

We have code to scan the existing material with our depth camera and build a height map (basically an image with a Z elevation for each XY pixel) of the existing cut face.  The mining head is a cylinder, and the arm can move it in a variety of directions, but it might be easier to approximate the cut as a tilted rectangle or something.  The hard part is finding useful software that can do these 3D calculations automatically.

In particular, given a height map (think a top-down image with grayscale elevations), figure out the (1) robot position where the wheels are on flat ground and front scoop is against the cut face, and (2) arm path that will cut an even slice of material off. 
