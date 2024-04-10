$fn = 30;

module LED(){
    intersection(){
    translate([0,0,0])
        cylinder(r=5.8/2,h=1);
    translate([-2.5,-3,0])
        cube([6,6,6]);
    }
    cylinder(r=5/2,6.2);
    translate([0,0,6.2]) sphere(r=2.5);
    
    translate([1.25*1.5-0.25,-0.25,-25])
        cube([0.5,0.5,25]);
    translate([1.25*0.5-0.25,-0.25,-26])
        cube([0.5,0.5,26]);
    translate([-1.25*0.5-0.25,-0.25,-27])
        cube([0.5,0.5,27]);
    translate([-1.25*1.5-0.25,-0.25,-26])
        cube([0.5,0.5,26]);
    
}

module Jig(){
    
pin4StrPos = [0, 1.25*1.5];
pin3StrPos = [0, 1.25*0.5];
pin1StrPos = [0,-1.25*1.5];
    
pin4EndPos = [-5.2,3.5];
pin3EndPos = [5.2,3.5];
pin1EndPos = [-5.2,-3.5];
    
difference(){
    union(){
        translate([-5,-10,0])
            cube([10,20,20.5]);
        
        difference(){
        union(){
        translate([-5.5,0,0])
            cylinder(r=5.6/2,h=20.5);
        translate([-5.5,0,20.5])
            cylinder(r1=6/2,r2=1.5,h=1.7);
        }
        translate([-105.1,-50,-1]) cube([100,100,100]);
        }
        
        difference(){
        union(){
        translate([ 5.5,0,0])
            cylinder(r=5.6/2,h=20.5);
        translate([ 5.5,0,20.5])
            cylinder(r1=5.6/2,r2=1.5,h=1.7);
        }
        translate([5.01,-50,-1]) cube([100,100,100]);
        }
        
        translate([-15,-20,0]) cube([30,40,10]);
        translate([0,-10.1,0]) cube([10,5,25]);
        translate([0,-20,0]) cube([10,5.7,25]);
        
        translate([0,-18.5,0]) cube([10,3,30]);
        
        translate([-10,5.1,0]) cube([10,5,25]);
        translate([-10,14.3,0]) cube([10,5.7,25]);
        
        translate([-10,15.8,0]) cube([10,3,30]);
        
    }
    union(){
        tol = 0.15;
        translate([0,0,-1]) cylinder(r=5/2+tol,h=100);
        translate([0,0,19]) intersection(){
            cylinder(r=5.8/2+tol,h=3.1);
            translate([0,0.4,0])
                cube([5.8+tol*2,5.8+tol*2,6],center=true);
        }
        
        translate([0,0,20]) hull(){
            translate(pin4StrPos) cylinder(r=0.5,h=5);
            translate(pin4EndPos) cylinder(r=0.5,h=5);
        }
        translate([0,0,20]) hull(){
            translate(pin3StrPos) cylinder(r=0.5,h=5);
            translate(pin3EndPos) cylinder(r=0.5,h=5);
        }
        translate([0,0,20]) hull(){
            translate(pin1StrPos) cylinder(r=0.5,h=5);
            translate(pin1EndPos) cylinder(r=0.5,h=5);
        }
        
        translate([0,0,12]) union(){
        translate(pin4EndPos)
            cylinder(r=0.5,h=50);
        translate(pin3EndPos)
            cylinder(r=0.5,h=50);
        translate(pin1EndPos)
            cylinder(r=0.5,h=50);
        }
        
        //Over bend Region
        hull(){
            translate([-13,-10.1,10])
                cube([10,15.1,1]);
            translate([-15,-10.1,18])
                cube([10,15.1,1]);
        }
        rotate([0,0,180])hull(){
            translate([-13,-10.1,10])
                cube([10,15.1,1]);
            translate([-15,-10.1,18])
                cube([10,15.1,1]);
        }
        
    }
    
    //Axis
    translate([5,-4.5,20]) rotate([90,0,0])
        cylinder(r=1.6,h=50);
    translate([-5,4.5,20]) rotate([-90,0,0])
        cylinder(r=1.6,h=50);
    
    }
}

module FormingTool(){
difference(){
    union(){
        translate([1,-1.25,0])
        rotate([90,135,0])
        cube([5,5,1.25+2]);
        
        translate([-1,1.25,0])
        rotate([90,135,0])
        cube([5,5,1.25]);
        
        translate([1,2.5+2,0])
        rotate([90,135,0])
        cube([5,5,1.25+2]);
    }
    union(){
        
    }
}
}

module RotatingForm2(){
    translate([-5,0,20])
    rotate([0,180,0])
    translate([5,0,0])
    union(){
    difference(){
        union(){
    translate([5.5,0,-5]) cube([5,14.2,5]);
        }
    
    hull(){
    translate([5.5,3.5,-5.1])
        cylinder(r1=3.5,r=0.1,h=4.5);
    translate([10.5,3.5,-5.1])
        cylinder(r1=0.5,r=0.1,h=1.5);
    }
    }
    difference(){
    intersection(){
    union(){
    hull(){
        translate([5.5,10.2,-5]) cube([5,4,5]);
        translate([-5,14.2,0])
        rotate([90,0,0])
            cylinder(r=5,h=4);
    }
    translate([5.5,10.2,-5]) cube([30,4,5]);
    }
    translate([-12,12.2,-6]) rotate([45,0,0])
        cube([50,20,20]);
    }
    //Axis
    translate([-5,0,0]) rotate([-90,0,0])
        cylinder(r=1.6,h=50);
}
    }
}

module RotatingForm1(){
    translate([5,0,20])
    rotate([0,85,0])
    translate([-5,0,0]) union(){
    difference(){
    union(){
    translate([-12.5,-14.2,-5]) cube([7,18.7,10]);
    *translate([-12.5,-5,2]) cube([20,10,3]);
    }
    
    hull(){
    translate([-5.5,3.5,-5.1])
        cylinder(r1=2.2,r=0.1,h=4.5);
    translate([-10,3.5,-5.1])
        cylinder(r1=2.1,r=0.1,h=4.5);
    }
    
   hull(){ 
    translate([-5.5,-3.5,-5.1])
        cylinder(r1=2.2,r=0.1,h=4.5);
    translate([-10,-3.5,-5.1])
        cylinder(r1=2.1,r=0.1,h=4.5);
    }
    
    translate([-6.5,0,1.5])
        cylinder(r1=1,r2=3,h=4);
    
    translate([0,-1.3,0]) rotate([-90,0,0]) 
        cylinder(r=10,h=2.6);
    }
    difference(){
    intersection(){
    union(){
    hull(){
        translate([-10.5,-14.2,-5]) cube([5,4,10]);
        translate([5,-10.2,0])
        rotate([90,0,0])
            cylinder(r=5,h=4);
    }
    translate([-35,-14.2,0]) cube([30,4,5]);
    }
    translate([-38,-12.2,-22.5]) rotate([45,0,0])
        cube([50,20,20]);
    }
    //Axis
    translate([5,0,0]) rotate([90,0,0])
        cylinder(r=1.6,h=50);
    }
}
}

Jig();

*translate([0,0,54]) FormingTool();

RotatingForm1();
RotatingForm2();

color("red")
translate([0,0,20.1])
rotate([180,0,90])
difference(){
    LED();
    translate([0,0,-30]) cylinder(r=10,19);
}

