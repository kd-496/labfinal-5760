module SpaceShooter(
    input wire clk,                 // 50 MHz system clock
    input wire reset,               // Reset signal (active low)
    input wire [3:0] keys,          // Input keys for control (up, down, shoot)
    output wire [7:0] VGA_R,        // VGA red channel
    output wire [7:0] VGA_G,        // VGA green channel
    output wire [7:0] VGA_B,        // VGA blue channel
    output wire VGA_HS,             // VGA horizontal sync
    output wire VGA_VS              // VGA vertical sync
);

    // Screen dimensions
    localparam SCREEN_WIDTH = 640;
    localparam SCREEN_HEIGHT = 480;

    // Physics constants
    localparam integer G = 10;       // Simplified gravitational constant
    localparam integer MASS_EARTH = 10000; // Large mass representing a central gravitational body

    // Initial ship position and velocity
    reg [15:0] ship_x = SCREEN_WIDTH / 2;   // Horizontal center
    reg [15:0] ship_y = SCREEN_HEIGHT / 2;  // Vertical center
    reg signed [15:0] velocity_x = 0;       // Initial horizontal velocity
    reg signed [15:0] velocity_y = 0;       // Initial vertical velocity

    // Time step for simulation
    localparam integer dt = 1;      // Time step for physics calculations

    // Simulation of orbital mechanics
    always @(posedge clk) begin
        if (reset) begin
            // Reset ship position and velocity
            ship_x <= SCREEN_WIDTH / 2;
            ship_y <= SCREEN_HEIGHT / 2;
            velocity_x <= 0;
            velocity_y <= 0;
        end else begin
            // Calculate radial forces
            integer dx = SCREEN_WIDTH / 2 - ship_x;
            integer dy = SCREEN_HEIGHT / 2 - ship_y;
            integer r_squared = dx*dx + dy*dy; // Distance squared from center
            integer force_mag = (G * MASS_EARTH) / r_squared; // Magnitude of gravitational force
            
            // Update velocities based on gravitational pull
            velocity_x <= velocity_x + (force_mag * dx) / r_squared;
            velocity_y <= velocity_y + (force_mag * dy) / r_squared;
            
            // Update ship position
            ship_x <= ship_x + velocity_x * dt;
            ship_y <= ship_y + velocity_y * dt;
        end
    end

    // VGA controller instantiation for rendering
    VGA_Controller vga_ctrl (
        .clk(clk),
        .reset(reset),
        .ship_x(ship_x),
        .ship_y(ship_y),
        .VGA_R(VGA_R),
        .VGA_G(VGA_G),
        .VGA_B(VGA_B),
        .VGA_HS(VGA_HS),
        .VGA_VS(VGA_VS)
    );
    
endmodule
