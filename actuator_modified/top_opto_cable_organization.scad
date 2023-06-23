// top mount for laser cable organization

$fn = 50;


wall_height = 50;
wall_thick = 2;
//translate([-47.5,-25,20])

module opto_connector_neg(){

diam=15;
translate([-diam/2,-diam/2,0])
cube([diam,diam,1]);
connectordiam = 9;

translate([0,0,-2])
cylinder(5,connectordiam/2,connectordiam/2);

screwdiam = 2.5;

delta=9.5; // inter hole distance 7.5 - 11.5
translate([0,0,-2])
for ( dx = [-1:2:1] ){
for ( dy = [-1:2:1] ){
translate([dx*delta/2,dy*delta/2])
cylinder(5,screwdiam/2,screwdiam/2);
}
}

}

difference(){
    // main plate and walls
    union(){
        cube([80,50,2]);
        
        cube([80,wall_thick,wall_height]);
        translate([0,50-wall_thick,0]) cube([80,wall_thick,wall_height]);
        translate([80-wall_thick,0,0]) cube([wall_thick,50,wall_height]);
    }
    
    // inser wjole for the cables
    //translate([78,15,2])
    //cube([2,20,20]);
    
    // have another opto cable connector:
    
    
    translate([20,50,40])    rotate([90,0,0])    cylinder(99,2,2);
    translate([30,50,40])    rotate([90,0,0])    cylinder(99,2,2);
    translate([50,50,40])    rotate([90,0,0])    cylinder(99,2,2);
    translate([60,50,40])    rotate([90,0,0])    cylinder(99,2,2);
    
    
    // space for commutator
    translate([5,25,0])
        cylinder(5,10,10);
    
    // opto 1
    translate([6.5,7.5,0])
        cylinder(5,6,6);
    //translate([0,7,0])
    //    cube([8,1,2]);
    
    // opto 2
    translate([6.5,50-7.5,0])
        cylinder(5,6,6);
    //translate([0,50-7-1,0])
    //    cube([8,1,2]);
    
    // 4 screw holes
    translate([80-11,5,0])
        cylinder(5,3.5/2,3.5/2);
    translate([80-11,50-5,0])
        cylinder(5,3.5/2,3.5/2);
    translate([80-11-45,5,0])
        cylinder(5,3.5/2,3.5/2);
    translate([80-11-45,50-5,0])
        cylinder(5,3.5/2,3.5/2);
    
    // 4 nuts recessed
    translate([80-11,5,1])
        cylinder(5,8/2,8/2);
    translate([80-11,50-5,1])
        cylinder(5,8/2,8/2);
    translate([80-11-45,5,1])
        cylinder(5,8/2,8/2);
    translate([80-11-45,50-5,1])
        cylinder(5,8/2,8/2);    
    
}