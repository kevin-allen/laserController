// spacer between layers on actuator platform for storing laser cable

$fn = 50;

// height: screw = 2cm
// platforms: 2mm each (2x bottom, 1x top)
// remaining height for nut: 2mm

h = 20-3*2 - 2;

h=13; // set manually
echo("h=",h);

innerdiam = 4.5;
outerdiam = 6.5;

difference(){
    cylinder(h,outerdiam/2,outerdiam/2);
    cylinder(h,innerdiam/2,innerdiam/2);
}