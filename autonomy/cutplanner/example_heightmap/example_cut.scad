/* OpenSCAD example calculating the cut face. */
$fs=0.01; // file units are meters, planning precision is cm

difference()  //<- show the cut face
//intersection() //<- calculate volume of excavated material
{
    translate([-1,4,-0.3]) // put height map at reasonable origin
        import("heightmap.stl",convexity=6);

   # translate([1,2,0]) // bottom point of cut
        rotate([0,30,0]) // angle of cut (60 deg up from horizontal)
            cube([0.1,0.3,1.0]); // actual cut face
}

