
// Base properties (mm)
r_base = 220;
h_base = 10;

// Wheel properties (mm)
r_wheel = 78;
h_wheel = 27;
wheel_offset_z = 28;
wheel_offset_y = 190;

// Caster position (mm)
caster_offset_x = 180;
caster_offset_z = h_base/2;

// Main holes (need to be check) (mm)
holes_inside_x = 91+92;
holes_inside_y = 208/2;
holes_out_x = 91;
holes_out_y = 208/2+55;

// Electronics tower properties (mm)
electronics_tower_x = 200;
electronics_tower_y = 100;
electronics_tower_z = 180;
electronics_tower_offset_x = 0;
electronics_tower_offset_y = 0;
electronics_tower_offset_z = h_base/2+electronics_tower_z/2;

// Shell properties (mm)
shell_width = 5;
skirt_width = 3;
skirt_length = 20;

shell_skirt_diff = 1.1;

_fa = 0.25;
_fa2 = 20;

// Include papa model for design purposes only
papa();
// Generate shells 
back_shell(side = 1); // Select side, 1 is left, -1 is right
back_shell(side = -1);

module back_shell(side) {
    difference() {
        union() {
            // Generate top and bottom skirt
            difference() {
                cylinder(h=h_base + skirt_width *2, r=r_base + shell_width, $fa=_fa2, center=true);
                // Remove base
                cylinder(h=h_base, r=r_base, $fa= _fa, center=true);
                cylinder(h=h_base + skirt_width *2, r=r_base + shell_width*2 - skirt_length, $fa= _fa, center=true);
            }
            // Generate shell
            translate([0, 0, skirt_width])
            scale([1,1,0.5]) {
                difference() {
                    sphere(r=r_base + shell_width+ shell_skirt_diff, $fa=_fa2, center=true);
                    sphere(r=r_base, $fa=_fa2, center=true);
                    // Remove back bottom quarter of the semi-sphere
                    translate([r_base/2, 0, -r_base/2 + h_base/2])
                        cube([r_base + shell_width*4, r_base*2, r_base], center=true);
                }
            }
        }  
        // Remove right side of the spherer
        translate([0, side * r_base, 0])
          cube([r_base*2 + shell_width*4, r_base*2, r_base*2], center=true);
        // Remove front side of the semi-sphere
        translate([-r_base/2, 0, 0])
          cube([r_base + shell_width*4, r_base*2 + shell_width*4, r_base*2], center=true);
        // Greate space for the electronics tower
        electronics_space = electronics_tower_x/2 + electronics_tower_offset_x;
        translate([electronics_space/2, 0, 0])
          cube([electronics_space, r_base*2  + shell_width*4, r_base*2], center=true);
    }
}

module papa() {
    // Electronics box
    translate([electronics_tower_offset_x, electronics_tower_offset_y, electronics_tower_offset_z])
        cube([electronics_tower_x, electronics_tower_y, electronics_tower_z], center=true);
    
    // caster front
    translate([caster_offset_x, 0, -caster_offset_z])
        rotate([-90,0,90])
            scale([1000,1000,1000])
            import("caster-wheel.stl", convexity=3);
    // caster back
    translate([-caster_offset_x, 0, -caster_offset_z])
       rotate([-90,0,-90])
    scale([1000,1000,1000])
           import("caster-wheel.stl", convexity=3);
    // wheel left
    translate([0, wheel_offset_y, -wheel_offset_z]) 
        rotate([90,0,0])
            cylinder(h=h_wheel, r=r_wheel, $fa=_fa, center=true);
    // wheel right
    translate([0, -wheel_offset_y, -wheel_offset_z]) 
        rotate([90,0,0])
            cylinder(h=h_wheel, r=r_wheel, $fa=_fa, center=true);
    // base
    cylinder(h=h_base, r=r_base, $fa=_fa, center=true);
}