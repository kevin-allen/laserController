$fn=50;


difference(){

    // main plate (import & add)
    
    union(){
        import("../Actuator_marcel_edition/MiddleTop.stl");

        translate([-48,-25,0])
            cube([17,21,2]);
        
        translate([-48,4,0])
            cube([17,21,2]);
        
        translate([-48,-4,0])
            cube([1,8,2]);
        
        
    }
    
    // remove space for opto laser socket
    
    translate([-48,-25,1.5])
        translate([7.5,7.5])
        opto_connector_neg();
    translate([-48,10,1.5])
        translate([7.5,7.5])
        opto_connector_neg();

}



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

/*
translate([-48,0,0])
cube([81.5,10,10]);
*/