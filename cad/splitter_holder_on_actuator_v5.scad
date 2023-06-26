/*
 Parts needed to install transparent plexiglass walls on an arena.
 
 Assumes we have 4 walls of 60 cm in width. The walls were heated and bent into a circular shape. Each wall covers 90 degrees of the arena.
 
 The total circumference of the wall is 2*pi*r = 60cm*4, we need to know the radius
 240/(2*pi) = r
 r = 38.197 cm
 
 
*/
$fa=1;
$fs=1;

module opto_connector_neg(){

diam=15;
//translate([-diam/2,-diam/2,0])
//cube([diam,diam,2]);
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
     // space for the fiber to comeout_1
     rotate([90,0,45])translate([-35,2,-35])cube([20,20,7]);
     // space for the fiber to comeout_2
     rotate([90,0,75])translate([-1,2,35])cube([20,20,7]);
     // space for the fiber to comeout_3
     rotate([90,0,10])translate([-30,2,-35])cube([20,20,7]);
     // space for the fiber to comeout_4
    rotate([90,0,-10])translate([10,2,-35])cube([20,20,7]);
    }
    
}
 

module optic_fibre_straight(thickness=2,wallHeight=10, width=10, length=100){
    
    difference(){
    cube([length,width,wallHeight]);   
    translate([-1,thickness,thickness])cube([length+2,width-thickness*2,wallHeight]); 
    }
    translate([0,0,wallHeight-thickness])cube([length,width-4,thickness]);
}

module join_coil_straight(thickness=2,width=25,length=30){
    difference(){
    translate([-length/2,-width/2,0])cube([length,width,thickness]);
    rotate([0,90,0])translate([-30,-10,20])cube([20,20,8]);
    }
}

module hood_coil(){
    /////
thickness=2;
wallHeight=2;
coilDiameter=100;
width=10;

 difference(){
        cylinder(d=coilDiameter,h=wallHeight,$fn = 100);
        //translate([0,0,-0.1])cylinder(d=coilDiameter-width*2-20,h=wallHeight+1,$fn = 100); // hole in the middle
     translate([0,0,-0.1])cylinder(d=coilDiameter-width*2+10,h=wallHeight+1,$fn = 100);
        //cut the bottom
        difference(){
            translate([0,0,thickness])cylinder(d=coilDiameter-thickness*2,h=wallHeight+1,$fn = 100); // space for fibre
            translate([0,0,thickness-0.1])cylinder(d=coilDiameter-width*2+thickness*2,h=wallHeight+2); // space for fibre
        }
        
    }
        
}


difference(){

    rotate([0,90,0])translate([-25,-70,25])cube([25,22,3]);
    rotate([0,90,0])translate([-24,-66,25])translate([7.5,7.5])opto_connector_neg();
    rotate([0,90,0])translate([-24,-67,25])cube([15,17,1]);

}

module opto_connector(length=15, thickness=2,opticHoleDiameter=9,screwHoleDiameter=2){
    difference(){
    translate([-length/2,-length/2,0])cube([length,length,2]);
    translate([0,0,-1]) cylinder(d=opticHoleDiameter,h=thickness+2);
    for (rot = [0,90,180,270]){
        rotate([0,0,45+rot])translate([6.6,0,-1])cylinder(d=screwHoleDiameter,h=thickness+2);
    }
    }
}

module actuator_top(thickness=2,width=50, length=77,allParts=true){
    
    
    difference(){
        cube([length,width,thickness]);
        // screw holes to attach to lower part
        translate([11,5,-1])cylinder(d=3,h=thickness+2);
        translate([67-11,width-5,-1])cylinder(d=3,h=thickness+2);
        translate([11,width-5,-1])cylinder(d=3,h=thickness+2);
        translate([67-11,5,-1])cylinder(d=3,h=thickness+2);
    }
    
    
    
    if(allParts==true){
    translate([45,0,thickness])cube([2,16,12]); // 1 of 2 parallel vertical walls
    translate([45-12.,0,thickness])cube([2,16,12]); // 1 of 2 parallel vertical walls
    // vertical walls at 45 degrees
    translate([26+13.6,32,2])rotate([0,0,45]){
    difference(){
        cube([thickness,18,12]);
        translate([-1,9,6])rotate([0,90,0])cylinder(d=3,h=10);
    }
    }
    // hole for the rotary joint
    translate([length+15/2,width/2])difference(){
    union(){
        hull(){
        translate([-20,-15/2,0])cube([15,15,thickness]);
        cylinder(d=15,h=thickness);
        }
        translate([0,0,thickness])cylinder(d=15,h=12);
        }
    
    translate([0,0,thickness])cylinder(d=12,h=13);
    translate([0,0,-thickness])cylinder(d=7,h=12);    
    }
    
    // holder for 2 optic fiber 
    
    translate([15/2+length,15/2+width-15,0])opto_connector();
    translate([15/2+length,15/2,0])opto_connector();
}
}



module optic_fiber_all(){
    translate([0,10,0])optic_fiber_coil();
    translate([-50,-80,0])optic_fibre_straight();
    translate([50/2,-98,0]) rotate([0,0,90])actuator_top(allParts=false);
}






//translate([50/2,-98,-15]) rotate([0,0,90])actuator_top();
optic_fiber_all();
