// --- Parameters ---

// Inner dimensions of the sleeve (Battery size + tolerance)
// I have set these to match the standard YB1208300-USB (approx 80mm x 39mm)
inner_width = 82;  // Long side (approx 8cm + clearance)
inner_depth = 41;  // Short side (approx 4cm + clearance)

// Height of the sleeve walls
total_height = 50;

// Thickness of the walls and base floor
wall_thickness = 3;
floor_thickness = 2; 

// Width of the mounting tabs (ears) for the screws
tab_width = 12;

// Screw hole diameter (standard M3 or wood screw is ~3-4mm)
screw_hole_diameter = 4;

// Smoothness of circles
$fn = 60;

// --- Main Render ---

difference() {
    // 1. The main solid body (Base + Walls)
    union() {
        // The Sleeve Walls (Outer Block)
        translate([0, 0, total_height/2])
            cube([inner_width + 2*wall_thickness, inner_depth + 2*wall_thickness, total_height], center=true);
        
        // The Mounting Flanges (Base ears at corners)
        // We create a larger flat plate at the bottom for the screws
        translate([0, 0, floor_thickness/2])
            cube([inner_width + 2*wall_thickness + 2*tab_width, inner_depth + 2*wall_thickness + 2*tab_width, floor_thickness], center=true);
    }

    // 2. Subtract the "Hole" for the battery
    // We shift it up by 'floor_thickness' so the battery sits on a floor, not the wood.
    translate([0, 0, floor_thickness + (total_height/2)])
        cube([inner_width, inner_depth, total_height + 1], center=true);

    // 3. Subtract the Screw Holes (4 Corners)
    // We calculate the positions based on the total size
    
    // X and Y positions for the holes
    hole_x = (inner_width/2) + wall_thickness + (tab_width/2) - 1; 
    hole_y = (inner_depth/2) + wall_thickness + (tab_width/2) - 1;

    // Front-Right
    translate([hole_x, hole_y, -1])
        cylinder(h = total_height + 2, d = screw_hole_diameter);
    
    // Front-Left
    translate([-hole_x, hole_y, -1])
        cylinder(h = total_height + 2, d = screw_hole_diameter);
        
    // Back-Right
    translate([hole_x, -hole_y, -1])
        cylinder(h = total_height + 2, d = screw_hole_diameter);
        
    // Back-Left
    translate([-hole_x, -hole_y, -1])
        cylinder(h = total_height + 2, d = screw_hole_diameter);
}