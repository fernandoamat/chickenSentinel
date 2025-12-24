// --- MEASURE YOUR BRASS FITTING ---
nozzle_barrel_dia = 16.5;    // Diameter of the round threaded part

// --- MOUNTING SETTINGS ---
// 2.8mm is tight for M3, allowing the screw to cut its own threads into the plastic.
// If it's too hard to screw in, drill it out slightly to 3.0mm.
mount_screw_dia = 2.8;     
hole_spacing = 14;         // Distance between the two mounting holes
whole_offset = 2.5;      // Offset to the wall

$fn=100;

difference() {
    // 1. The Main Block
    cube([nozzle_barrel_dia + 3, 30, nozzle_barrel_dia + 1.5], center=true);


    // 3. The Barrel Exit (Where water comes out)
    rotate([90, 0, 0])
        translate([0, 0, -10])
            cylinder(h=20, d=nozzle_barrel_dia, center=true);
            
    // 4. The Hose Entrance (Rear)
    rotate([90, 0, 0])
        translate([0, 0, 10])
            cylinder(h=30, d=nozzle_barrel_dia, center=true);

    // 5. Fixed Mounting Holes (13mm spacing)
    // Top Hole
    translate([0, hole_spacing/2, whole_offset]) 
        rotate([0, 90, 0]) 
            cylinder(h=30, d=mount_screw_dia, center=true);
            
    // Bottom Hole
    translate([0, -hole_spacing/2, whole_offset]) 
        rotate([0, 90, 0]) 
            cylinder(h=30, d=mount_screw_dia, center=true);
}