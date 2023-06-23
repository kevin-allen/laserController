/*
 Parts needed to install transparent plexiglass walls on an arena.
 
 Assumes we have 4 walls of 60 cm in width. The walls were heated and bent into a circular shape. Each wall covers 90 degrees of the arena.
 
 The total circumference of the wall is 2*pi*r = 60cm*4, we need to know the radius
 240/(2*pi) = r
 r = 38.197 cm
 
 
*/
$fa=1;
$fs=1;


module optic_fiber_coil(thickness=2, wallHeight=10, coilDiameter=100, width=10){
    difference(){
        cylinder(d=coilDiameter,h=wallHeight);
        translate([0,0,-0.1])cylinder(d=coilDiameter-width*2,h=wallHeight+1); // hole in the middle
        // track for the optic fibre
        difference(){
            translate([0,0,thickness])cylinder(d=coilDiameter-thickness*2,h=wallHeight+1); // space for fibre
            translate([0,0,thickness-0.1])cylinder(d=coilDiameter-width*2+thickness*2,h=wallHeight+2); // space for fibre
        }
        // entry points for the fibre
        translate([50,0,thickness])rotate([0,0,180])cube([width-thickness*2,100,wallHeight]);
        translate([-50+width-thickness*2,0,thickness])rotate([0,0,180])cube([width-thickness*2,100,wallHeight]);
          
    }
    
}

module optic_fibre_straight(thickness=2,wallHeight=10, width=10, length=100){
    
    difference(){
    cube([length,width,wallHeight]);   
    translate([-1,thickness,thickness])cube([length+2,width-thickness*2,wallHeight]);
    }
}

module join_coil_straight(thickness=2,width=25,length=30){
    translate([-length/2,-width/2,0])cube([length,width,thickness]);
}

optic_fiber_coil();
translate([0,-60,0])join_coil_straight();
translate([-50,-80,0])optic_fibre_straight();