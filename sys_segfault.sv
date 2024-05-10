module DE1_SoC_Computer (
    ////////////////////////////////////
    // FPGA Pins
    ////////////////////////////////////

    // Clock pins
    CLOCK_50,
    CLOCK2_50,
    CLOCK3_50,
    CLOCK4_50,

    // ADC
    ADC_CS_N,
    ADC_DIN,
    ADC_DOUT,
    ADC_SCLK,

    // Audio
    AUD_ADCDAT,
    AUD_ADCLRCK,
    AUD_BCLK,
    AUD_DACDAT,
    AUD_DACLRCK,
    AUD_XCK,

    // SDRAM
    DRAM_ADDR,
    DRAM_BA,
    DRAM_CAS_N,
    DRAM_CKE,
    DRAM_CLK,
    DRAM_CS_N,
    DRAM_DQ,
    DRAM_LDQM,
    DRAM_RAS_N,
    DRAM_UDQM,
    DRAM_WE_N,

    // I2C Bus for Configuration of the Audio and Video-In Chips
    FPGA_I2C_SCLK,
    FPGA_I2C_SDAT,

    // 40-Pin Headers
    GPIO_0,
    GPIO_1,
    
    // Seven Segment Displays
    HEX0,
    HEX1,
    HEX2,
    HEX3,
    HEX4,
    HEX5,

    // IR
    IRDA_RXD,
    IRDA_TXD,

    // Pushbuttons
    KEY,

    // LEDs
    LEDR,

    // PS2 Ports
    PS2_CLK,
    PS2_DAT,
    
    PS2_CLK2,
    PS2_DAT2,

    // Slider Switches
    SW,

    // Video-In
    TD_CLK27,
    TD_DATA,
    TD_HS,
    TD_RESET_N,
    TD_VS,

    // VGA
    VGA_B,
    VGA_BLANK_N,
    VGA_CLK,
    VGA_G,
    VGA_HS,
    VGA_R,
    VGA_SYNC_N,
    VGA_VS,

    ////////////////////////////////////
    // HPS Pins
    ////////////////////////////////////
    
    // DDR3 SDRAM
    HPS_DDR3_ADDR,
    HPS_DDR3_BA,
    HPS_DDR3_CAS_N,
    HPS_DDR3_CKE,
    HPS_DDR3_CK_N,
    HPS_DDR3_CK_P,
    HPS_DDR3_CS_N,
    HPS_DDR3_DM,
    HPS_DDR3_DQ,
    HPS_DDR3_DQS_N,
    HPS_DDR3_DQS_P,
    HPS_DDR3_ODT,
    HPS_DDR3_RAS_N,
    HPS_DDR3_RESET_N,
    HPS_DDR3_RZQ,
    HPS_DDR3_WE_N,

    // Ethernet
    HPS_ENET_GTX_CLK,
    HPS_ENET_INT_N,
    HPS_ENET_MDC,
    HPS_ENET_MDIO,
    HPS_ENET_RX_CLK,
    HPS_ENET_RX_DATA,
    HPS_ENET_RX_DV,
    HPS_ENET_TX_DATA,
    HPS_ENET_TX_EN,

    // Flash
    HPS_FLASH_DATA,
    HPS_FLASH_DCLK,
    HPS_FLASH_NCSO,

    // Accelerometer
    HPS_GSENSOR_INT,
        
    // General Purpose I/O
    HPS_GPIO,
        
    // I2C
    HPS_I2C_CONTROL,
    HPS_I2C1_SCLK,
    HPS_I2C1_SDAT,
    HPS_I2C2_SCLK,
    HPS_I2C2_SDAT,

    // Pushbutton
    HPS_KEY,

    // LED
    HPS_LED,
        
    // SD Card
    HPS_SD_CLK,
    HPS_SD_CMD,
    HPS_SD_DATA,

    // SPI
    HPS_SPIM_CLK,
    HPS_SPIM_MISO,
    HPS_SPIM_MOSI,
    HPS_SPIM_SS,

    // UART
    HPS_UART_RX,
    HPS_UART_TX,

    // USB
    HPS_CONV_USB_N,
    HPS_USB_CLKOUT,
    HPS_USB_DATA,
    HPS_USB_DIR,
    HPS_USB_NXT,
    HPS_USB_STP
);

//=======================================================
//  PARAMETER declarations
//=======================================================

//=======================================================
//  PORT declarations
//=======================================================

////////////////////////////////////
// FPGA Pins
////////////////////////////////////

// Clock pins
input CLOCK_50;
input CLOCK2_50;
input CLOCK3_50;
input CLOCK4_50;

// ADC
inout ADC_CS_N;
output ADC_DIN;
input ADC_DOUT;
output ADC_SCLK;

// Audio
input AUD_ADCDAT;
inout AUD_ADCLRCK;
inout AUD_BCLK;
output AUD_DACDAT;
inout AUD_DACLRCK;
output AUD_XCK;

// SDRAM
output [12:0] DRAM_ADDR;
output [1:0] DRAM_BA;
output DRAM_CAS_N;
output DRAM_CKE;
output DRAM_CLK;
output DRAM_CS_N;
inout [15:0] DRAM_DQ;
output DRAM_LDQM;
output DRAM_RAS_N;
output DRAM_UDQM;
output DRAM_WE_N;

// I2C Bus for Configuration of the Audio and Video-In Chips
output FPGA_I2C_SCLK;
inout FPGA_I2C_SDAT;

// 40-pin headers
inout [35:0] GPIO_0;
inout [35:0] GPIO_1;

// Seven Segment Displays
output [6:0] HEX0;
output [6:0] HEX1;
output [6:0] HEX2;
output [6:0] HEX3;
output [6:0] HEX4;
output [6:0] HEX5;

// IR
input IRDA_RXD;
output IRDA_TXD;

// Pushbuttons
input [3:0] KEY;

// LEDs
output [9:0] LEDR;

// PS2 Ports
inout PS2_CLK;
inout PS2_DAT;

inout PS2_CLK2;
inout PS2_DAT2;

// Slider Switches
input [9:0] SW;

// Video-In
input TD_CLK27;
input [7:0] TD_DATA;
input TD_HS;
output TD_RESET_N;
input TD_VS;

// VGA
output [7:0] VGA_B;
output VGA_BLANK_N;
output VGA_CLK;
output [7:0] VGA_G;
output VGA_HS;
output [7:0] VGA_R;
output VGA_SYNC_N;
output VGA_VS;

////////////////////////////////////
// HPS Pins
////////////////////////////////////
    
// DDR3 SDRAM
output [14:0] HPS_DDR3_ADDR;
output [2:0] HPS_DDR3_BA;
output HPS_DDR3_CAS_N;
output HPS_DDR3_CKE;
output HPS_DDR3_CK_N;
output HPS_DDR3_CK_P;
output HPS_DDR3_CS_N;
output [3:0] HPS_DDR3_DM;
inout [31:0] HPS_DDR3_DQ;
inout [3:0] HPS_DDR3_DQS_N;
inout [3:0] HPS_DDR3_DQS_P;
output HPS_DDR3_ODT;
output HPS_DDR3_RAS_N;
output HPS_DDR3_RESET_N;
input HPS_DDR3_RZQ;
output HPS_DDR3_WE_N;

// Ethernet
output HPS_ENET_GTX_CLK;
inout HPS_ENET_INT_N;
output HPS_ENET_MDC;
inout HPS_ENET_MDIO;
input HPS_ENET_RX_CLK;
input [3:0] HPS_ENET_RX_DATA;
input HPS_ENET_RX_DV;
output [3:0] HPS_ENET_TX_DATA;
output HPS_ENET_TX_EN;

// Flash
inout [3:0] HPS_FLASH_DATA;
output HPS_FLASH_DCLK;
output HPS_FLASH_NCSO;

// Accelerometer
inout HPS_GSENSOR_INT;

// General Purpose I/O
inout [1:0] HPS_GPIO;

// I2C
inout HPS_I2C_CONTROL;
inout HPS_I2C1_SCLK;
inout HPS_I2C1_SDAT;
inout HPS_I2C2_SCLK;
inout HPS_I2C2_SDAT;

// Pushbutton
inout HPS_KEY;

// LED
inout HPS_LED;

// SD Card
output HPS_SD_CLK;
inout HPS_SD_CMD;
inout [3:0] HPS_SD_DATA;

// SPI
output HPS_SPIM_CLK;
input HPS_SPIM_MISO;
output HPS_SPIM_MOSI;
inout HPS_SPIM_SS;

// UART
input HPS_UART_RX;
output HPS_UART_TX;

// USB
inout HPS_CONV_USB_N;
input HPS_USB_CLKOUT;
inout [7:0] HPS_USB_DATA;
input HPS_USB_DIR;
input HPS_USB_NXT;
output HPS_USB_STP;

//=======================================================
//  REG/WIRE declarations
//=======================================================

wire [15:0] hex3_hex0;

assign HEX4 = 7'b1111111;
assign HEX5 = 7'b1111111;

HexDigit Digit0(HEX0, hex3_hex0[3:0]);
HexDigit Digit1(HEX1, hex3_hex0[7:4]);
HexDigit Digit2(HEX2, hex3_hex0[11:8]);
HexDigit Digit3(HEX3, hex3_hex0[15:12]);

//=======================================================
//  Structural coding
//=======================================================

// Declare wires for enemy positions
wire [31:0] enemyPosX, enemyPosY;

Computer_System The_System (
    ////////////////////////////////////
    // FPGA Side
    ////////////////////////////////////

    // Global signals
    .system_pll_ref_clk_clk             (CLOCK_50),
    .system_pll_ref_reset_reset         (1'b0),

    // AV Config
    .av_config_SCLK                     (FPGA_I2C_SCLK),
    .av_config_SDAT                     (FPGA_I2C_SDAT),

    // VGA Subsystem
    .vga_pll_ref_clk_clk                (CLOCK2_50),
    .vga_pll_ref_reset_reset            (1'b0),
    .vga_CLK                            (VGA_CLK),
    .vga_BLANK                          (VGA_BLANK_N),
    .vga_SYNC                           (VGA_SYNC_N),
    .vga_HS                             (VGA_HS),
    .vga_VS                             (VGA_VS),
    .vga_R                              (VGA_R),
    .vga_G                              (VGA_G),
    .vga_B                              (VGA_B),
    
    // SDRAM
    .sdram_clk_clk                      (DRAM_CLK),
   .sdram_addr                          (DRAM_ADDR),
    .sdram_ba                           (DRAM_BA),
    .sdram_cas_n                        (DRAM_CAS_N),
    .sdram_cke                          (DRAM_CKE),
    .sdram_cs_n                         (DRAM_CS_N),
    .sdram_dq                           (DRAM_DQ),
    .sdram_dqm                          ({DRAM_UDQM,DRAM_LDQM}),
    .sdram_ras_n                        (DRAM_RAS_N),
    .sdram_we_n                         (DRAM_WE_N),
    
    // Parallel I/O ports for enemy positions
    .enemyposx_external_connection_export                   (enemyPosX),
    .enemyposy_external_connection_export                 (enemyPosY),
    
    ////////////////////////////////////
    // HPS Side
    ////////////////////////////////////
    // DDR3 SDRAM
    .memory_mem_a           (HPS_DDR3_ADDR),
    .memory_mem_ba          (HPS_DDR3_BA),
    .memory_mem_ck          (HPS_DDR3_CK_P),
    .memory_mem_ck_n        (HPS_DDR3_CK_N),
    .memory_mem_cke         (HPS_DDR3_CKE),
    .memory_mem_cs_n        (HPS_DDR3_CS_N),
    .memory_mem_ras_n       (HPS_DDR3_RAS_N),
    .memory_mem_cas_n       (HPS_DDR3_CAS_N),
    .memory_mem_we_n        (HPS_DDR3_WE_N),
    .memory_mem_reset_n     (HPS_DDR3_RESET_N),
    .memory_mem_dq          (HPS_DDR3_DQ),
    .memory_mem_dqs         (HPS_DDR3_DQS_P),
    .memory_mem_dqs_n       (HPS_DDR3_DQS_N),
    .memory_mem_odt         (HPS_DDR3_ODT),
    .memory_mem_dm          (HPS_DDR3_DM),
    .memory_oct_rzqin       (HPS_DDR3_RZQ),
          
    // Ethernet
    .hps_io_hps_io_gpio_inst_GPIO35     (HPS_ENET_INT_N),
    .hps_io_hps_io_emac1_inst_TX_CLK    (HPS_ENET_GTX_CLK),
    .hps_io_hps_io_emac1_inst_TXD0      (HPS_ENET_TX_DATA[0]),
    .hps_io_hps_io_emac1_inst_TXD1      (HPS_ENET_TX_DATA[1]),
    .hps_io_hps_io_emac1_inst_TXD2      (HPS_ENET_TX_DATA[2]),
    .hps_io_hps_io_emac1_inst_TXD3      (HPS_ENET_TX_DATA[3]),
    .hps_io_hps_io_emac1_inst_RXD0      (HPS_ENET_RX_DATA[0]),
    .hps_io_hps_io_emac1_inst_MDIO      (HPS_ENET_MDIO),
    .hps_io_hps_io_emac1_inst_MDC       (HPS_ENET_MDC),
    .hps_io_hps_io_emac1_inst_RX_CTL    (HPS_ENET_RX_DV),
    .hps_io_hps_io_emac1_inst_TX_CTL    (HPS_ENET_TX_EN),
    .hps_io_hps_io_emac1_inst_RX_CLK    (HPS_ENET_RX_CLK),
    .hps_io_hps_io_emac1_inst_RXD1      (HPS_ENET_RX_DATA[1]),
    .hps_io_hps_io_emac1_inst_RXD2      (HPS_ENET_RX_DATA[2]),
    .hps_io_hps_io_emac1_inst_RXD3      (HPS_ENET_RX_DATA[3]),

    // Flash
    .hps_io_hps_io_qspi_inst_IO0        (HPS_FLASH_DATA[0]),
    .hps_io_hps_io_qspi_inst_IO1        (HPS_FLASH_DATA[1]),
    .hps_io_hps_io_qspi_inst_IO2        (HPS_FLASH_DATA[2]),
    .hps_io_hps_io_qspi_inst_IO3        (HPS_FLASH_DATA[3]),
    .hps_io_hps_io_qspi_inst_SS0        (HPS_FLASH_NCSO),
    .hps_io_hps_io_qspi_inst_CLK        (HPS_FLASH_DCLK),

    // Accelerometer
    .hps_io_hps_io_gpio_inst_GPIO61     (HPS_GSENSOR_INT),

    // General Purpose I/O
    .hps_io_hps_io_gpio_inst_GPIO40     (HPS_GPIO[0]),
    .hps_io_hps_io_gpio_inst_GPIO41     (HPS_GPIO[1]),

    // I2C
    .hps_io_hps_io_gpio_inst_GPIO48     (HPS_I2C_CONTROL),
    .hps_io_hps_io_i2c0_inst_SDA        (HPS_I2C1_SDAT),
    .hps_io_hps_io_i2c0_inst_SCL        (HPS_I2C1_SCLK),
    .hps_io_hps_io_i2c1_inst_SDA        (HPS_I2C2_SDAT),
    .hps_io_hps_io_i2c1_inst_SCL        (HPS_I2C2_SCLK),

    // Pushbutton
    .hps_io_hps_io_gpio_inst_GPIO54     (HPS_KEY),

    // LED
    .hps_io_hps_io_gpio_inst_GPIO53     (HPS_LED),

    // SD Card
    .hps_io_hps_io_sdio_inst_CMD        (HPS_SD_CMD),
    .hps_io_hps_io_sdio_inst_D0         (HPS_SD_DATA[0]),
    .hps_io_hps_io_sdio_inst_D1         (HPS_SD_DATA[1]),
    .hps_io_hps_io_sdio_inst_CLK        (HPS_SD_CLK),
    .hps_io_hps_io_sdio_inst_D2         (HPS_SD_DATA[2]),
    .hps_io_hps_io_sdio_inst_D3         (HPS_SD_DATA[3]),

    // SPI
    .hps_io_hps_io_spim1_inst_CLK       (HPS_SPIM_CLK),
    .hps_io_hps_io_spim1_inst_MOSI      (HPS_SPIM_MOSI),
    .hps_io_hps_io_spim1_inst_MISO      (HPS_SPIM_MISO),
    .hps_io_hps_io_spim1_inst_SS0       (HPS_SPIM_SS),

    // UART
    .hps_io_hps_io_uart0_inst_RX        (HPS_UART_RX),
    .hps_io_hps_io_uart0_inst_TX        (HPS_UART_TX),

    // USB
    .hps_io_hps_io_gpio_inst_GPIO09     (HPS_CONV_USB_N),
    .hps_io_hps_io_usb1_inst_D0         (HPS_USB_DATA[0]),
    .hps_io_hps_io_usb1_inst_D1         (HPS_USB_DATA[1]),
    .hps_io_hps_io_usb1_inst_D2         (HPS_USB_DATA[2]),
    .hps_io_hps_io_usb1_inst_D3         (HPS_USB_DATA[3]),
    .hps_io_hps_io_usb1_inst_D4         (HPS_USB_DATA[4]),
    .hps_io_hps_io_usb1_inst_D5         (HPS_USB_DATA[5]),
    .hps_io_hps_io_usb1_inst_D6         (HPS_USB_DATA[6]),
    .hps_io_hps_io_usb1_inst_D7         (HPS_USB_DATA[7]),
    .hps_io_hps_io_usb1_inst_CLK        (HPS_USB_CLKOUT),
    .hps_io_hps_io_usb1_inst_STP        (HPS_USB_STP),
    .hps_io_hps_io_usb1_inst_DIR        (HPS_USB_DIR),
    .hps_io_hps_io_usb1_inst_NXT        (HPS_USB_NXT)
);

wire [31:0] radius;
wire [7:0] angle;

Orbit_Path u_1 (.clk(CLOCK_50),	.start(1'b0), 	.angle(angle), 	.r(radius) );

// Enemy orbital motion module instance
EnemyOrbitalMotion u_enemy_orbital_motion (
    .clk (CLOCK_50),
    .reset (1'b0),
	 .radius_(radius),
	 .angle_(angle),
    .posX (enemyPosX),
    .posY (enemyPosY)
);

endmodule

module EnemyOrbitalMotion(
    input clk,                          // Clock signal
    input reset,                        // Reset signal
	 input [31:0] radius_,
	 input [7:0] angle_,
    output reg [31:0] posX,             // X position of the enemy
    output reg [31:0] posY              // Y position of the enemy
);

// Parameters
parameter CENTER_X = 320;  // Center of orbit X
parameter CENTER_Y = 240;  // Center of orbit Y
parameter SIZE_X = 640;
parameter SIZE_Y = 480;
parameter RADIUS = 100;    // Radius of orbit
parameter SPEED = 1;             // Speed of orbit (1 degree per clock)
//parameter SCALE = ;     // Scaling factor for LUT output

// Internal signals
//wire [31:0] radius;
reg [31:0] radius; 
reg [7:0] angle;                // Angle in degrees
wire signed [15:0] sin_val;         // Sine value from LUT
wire signed [15:0] cos_val;         // Cosine value from LUT
reg signed [31:0] x, y;             // Intermediate X and Y values
reg [19:0] cnt; 
wire signed [26:0] orbit_x, orbit_y;

// Instantiate sine and cosine LUTs
sineTable u_sineTable (
    .angle(angle),
    .sine(sin_val)
);

cosineTable u_cosineTable (
    .angle(angle),
    .cosine(cos_val)
);


always @(posedge clk or posedge reset) begin
    if (reset) begin
        angle <= 0;
		  radius <= radius_;
        posX <= CENTER_X + RADIUS;
        posY <= CENTER_Y;
		  cnt <= 0;
    end else begin
	     if (cnt < 20'b110000110101000000) begin
		     cnt <= cnt + 20'd1;
		  end
		  else begin
         // Increment angle for orbit
         angle <= angle_;
			radius <= radius_;
			cnt <= 0;
		  end	

        // Calculate positions using fixed-point multiplication
        x <= cos_val * radius >>> 13 ;
        y <= sin_val * radius >>> 13 ;

        // Update positions relative to the center
        posX <= CENTER_X + x[31:0];
        posY <= CENTER_Y + y[31:0];
		  //posX <= CENTER_X + ({orbit_x, 5'b00000}  >>> 13);
		  //posY <= CENTER_Y + ({orbit_y, 5'b00000}  >>> 13);
    end
end

endmodule

module Orbit_Path //orbit_solver_polar //angle & sine bits match?
# (parameter G = 5,
   parameter r_init = 100,
   //parameter step_len = 27'd1,//or = 2
   parameter va_init = 0,
   parameter v_ang_init = 10,// not by linear speed
   parameter ang_init = 0,
   parameter aa_init = 0,
   parameter M = 3000
)
(
    	input clk,
    	input start,// acts like reset
    //input clk2,
    //input [15:0] r_init,
    //input [7:0] ang_init,
    //input [15:0] va_init,
    //input [15:0] time_,
    //input [15:0] steps,
    //input start,
    	//output signed [31:0] x,
	   //output signed [31:0] y,
		output [8:0] angle,
		output [31:0] r
	 //output [26:0] r,
    //output reg [26:0] ang,
        //output reg [7:0] color
);
    //wire [26:0] step_len;

	reg signed [31:0] aa, r_t, va, cnt;//_t =  true value; va, aa = axial velocity & acceleration; vr, ar = tangent velocity & acceleration
   reg signed [7:0] ang, ang_1, ang_2, v_ang, ar;
    always @(posedge clk) begin
        if (start) begin
            r_t <= r_init;
            ang <= ang_init;
	         ang_1 <= ang_init - v_ang_init;
	         ang_2 <= ang_init - (v_ang_init <<< 1);
	         v_ang <= v_ang_init;

            aa <= (G * M) / (r_init * r_init);

            va <= va_init;
	    //color <= 8'b0000_0000;
        end
        else begin
		  if (cnt < 32'b110000110101000000) begin
		     cnt <= cnt + 32'd1;
		  end
		  else begin
		      cnt <= 32'd0;
            r_t <= r_t + va;
            ang <= ang + v_ang;
	         ang_1 <= ang;
	         ang_2 <= ang_1;

            aa <= (G * M) / (r_t * r_t);
	         ar <= (ang - ang_1) - (ang_1 - ang_2); //beta_theta = (d^2 theta / dt^2) 
		
            va <= va - aa;
	         v_ang <= v_ang + ar;
            //vr <= vr;
        end
	    //color <= 8'b1111_1111;
        end
    end
    
    //sineTable sin(ang,sin_);
    //sineTable cos((ang - 64),cos_);
	
    assign r = r_t;
	 assign angle = ang;
    //assign y = sin_ * r_t;
    //assign x = cos_ * r_t;
endmodule

module sineTable (
    input [7:0] angle,          // 8-bit angle input (0-255, 0 to 360 degrees)
    output reg signed [15:0] sine  // 16-bit signed output value (scaled by 8192)
);

always @(*) begin
    case(angle)
        8'h00: sine = 16'h0000 ;
        8'h01: sine = 16'h0192 ;
        8'h02: sine = 16'h0323 ;
        8'h03: sine = 16'h04b5 ;
        8'h04: sine = 16'h0645 ;
        8'h05: sine = 16'h07d5 ;
        8'h06: sine = 16'h0963 ;
        8'h07: sine = 16'h0af0 ;
        8'h08: sine = 16'h0c7c ;
        8'h09: sine = 16'h0e05 ;
        8'h0a: sine = 16'h0f8c ;
        8'h0b: sine = 16'h1111 ;
        8'h0c: sine = 16'h1293 ;
        8'h0d: sine = 16'h1413 ;
        8'h0e: sine = 16'h158f ;
        8'h0f: sine = 16'h1708 ;
        8'h10: sine = 16'h187d ;
        8'h11: sine = 16'h19ef ;
        8'h12: sine = 16'h1b5c ;
        8'h13: sine = 16'h1cc5 ;
        8'h14: sine = 16'h1e2a ;
        8'h15: sine = 16'h1f8b ;
        8'h16: sine = 16'h20e6 ;
        8'h17: sine = 16'h223c ;
        8'h18: sine = 16'h238d ;
        8'h19: sine = 16'h24d9 ;
        8'h1a: sine = 16'h261f ;
        8'h1b: sine = 16'h275f ;
        8'h1c: sine = 16'h2899 ;
        8'h1d: sine = 16'h29cc ;
        8'h1e: sine = 16'h2afa ;
        8'h1f: sine = 16'h2c20 ;
        8'h20: sine = 16'h2d40 ;
        8'h21: sine = 16'h2e59 ;
        8'h22: sine = 16'h2f6b ;
        8'h23: sine = 16'h3075 ;
        8'h24: sine = 16'h3178 ;
        8'h25: sine = 16'h3273 ;
        8'h26: sine = 16'h3366 ;
        8'h27: sine = 16'h3452 ;
        8'h28: sine = 16'h3535 ;
        8'h29: sine = 16'h3611 ;
        8'h2a: sine = 16'h36e4 ;
        8'h2b: sine = 16'h37ae ;
        8'h2c: sine = 16'h3870 ;
        8'h2d: sine = 16'h3929 ;
        8'h2e: sine = 16'h39da ;
        8'h2f: sine = 16'h3a81 ;
        8'h30: sine = 16'h3b1f ;
        8'h31: sine = 16'h3bb5 ;
        8'h32: sine = 16'h3c41 ;
        8'h33: sine = 16'h3cc4 ;
        8'h34: sine = 16'h3d3d ;
        8'h35: sine = 16'h3dad ;
        8'h36: sine = 16'h3e14 ;
        8'h37: sine = 16'h3e70 ;
        8'h38: sine = 16'h3ec4 ;
        8'h39: sine = 16'h3f0d ;
        8'h3a: sine = 16'h3f4d ;
        8'h3b: sine = 16'h3f83 ;
        8'h3c: sine = 16'h3fb0 ;
        8'h3d: sine = 16'h3fd2 ;
        8'h3e: sine = 16'h3feb ;
        8'h3f: sine = 16'h3ffa ;
        8'h40: sine = 16'h3fff ;
        8'h41: sine = 16'h3ffa ;
        8'h42: sine = 16'h3feb ;
        8'h43: sine = 16'h3fd2 ;
        8'h44: sine = 16'h3fb0 ;
        8'h45: sine = 16'h3f83 ;
        8'h46: sine = 16'h3f4d ;
        8'h47: sine = 16'h3f0d ;
        8'h48: sine = 16'h3ec4 ;
        8'h49: sine = 16'h3e70 ;
        8'h4a: sine = 16'h3e14 ;
        8'h4b: sine = 16'h3dad ;
        8'h4c: sine = 16'h3d3d ;
        8'h4d: sine = 16'h3cc4 ;
        8'h4e: sine = 16'h3c41 ;
        8'h4f: sine = 16'h3bb5 ;
        8'h50: sine = 16'h3b1f ;
        8'h51: sine = 16'h3a81 ;
        8'h52: sine = 16'h39da ;
        8'h53: sine = 16'h3929 ;
        8'h54: sine = 16'h3870 ;
        8'h55: sine = 16'h37ae ;
        8'h56: sine = 16'h36e4 ;
        8'h57: sine = 16'h3611 ;
        8'h58: sine = 16'h3535 ;
        8'h59: sine = 16'h3452 ;
        8'h5a: sine = 16'h3366 ;
        8'h5b: sine = 16'h3273 ;
        8'h5c: sine = 16'h3178 ;
        8'h5d: sine = 16'h3075 ;
        8'h5e: sine = 16'h2f6b ;
        8'h5f: sine = 16'h2e59 ;
        8'h60: sine = 16'h2d40 ;
        8'h61: sine = 16'h2c20 ;
        8'h62: sine = 16'h2afa ;
        8'h63: sine = 16'h29cc ;
        8'h64: sine = 16'h2899 ;
        8'h65: sine = 16'h275f ;
        8'h66: sine = 16'h261f ;
        8'h67: sine = 16'h24d9 ;
        8'h68: sine = 16'h238d ;
        8'h69: sine = 16'h223c ;
        8'h6a: sine = 16'h20e6 ;
        8'h6b: sine = 16'h1f8b ;
        8'h6c: sine = 16'h1e2a ;
        8'h6d: sine = 16'h1cc5 ;
        8'h6e: sine = 16'h1b5c ;
        8'h6f: sine = 16'h19ef ;
        8'h70: sine = 16'h187d ;
        8'h71: sine = 16'h1708 ;
        8'h72: sine = 16'h158f ;
        8'h73: sine = 16'h1413 ;
        8'h74: sine = 16'h1293 ;
        8'h75: sine = 16'h1111 ;
        8'h76: sine = 16'h0f8c ;
        8'h77: sine = 16'h0e05 ;
        8'h78: sine = 16'h0c7c ;
        8'h79: sine = 16'h0af0 ;
        8'h7a: sine = 16'h0963 ;
        8'h7b: sine = 16'h07d5 ;
        8'h7c: sine = 16'h0645 ;
        8'h7d: sine = 16'h04b5 ;
        8'h7e: sine = 16'h0323 ;
        8'h7f: sine = 16'h0192 ;
        8'h80: sine = 16'h0000 ;
        8'h81: sine = 16'hfe6e ;
        8'h82: sine = 16'hfcdd ;
        8'h83: sine = 16'hfb4b ;
        8'h84: sine = 16'hf9bb ;
        8'h85: sine = 16'hf82b ;
        8'h86: sine = 16'hf69d ;
        8'h87: sine = 16'hf510 ;
        8'h88: sine = 16'hf384 ;
        8'h89: sine = 16'hf1fb ;
        8'h8a: sine = 16'hf074 ;
        8'h8b: sine = 16'heeef ;
        8'h8c: sine = 16'hed6d ;
        8'h8d: sine = 16'hebed ;
        8'h8e: sine = 16'hea71 ;
        8'h8f: sine = 16'he8f8 ;
        8'h90: sine = 16'he783 ;
        8'h91: sine = 16'he611 ;
        8'h92: sine = 16'he4a4 ;
        8'h93: sine = 16'he33b ;
        8'h94: sine = 16'he1d6 ;
        8'h95: sine = 16'he075 ;
        8'h96: sine = 16'hdf1a ;
        8'h97: sine = 16'hddc4 ;
        8'h98: sine = 16'hdc73 ;
        8'h99: sine = 16'hdb27 ;
        8'h9a: sine = 16'hd9e1 ;
        8'h9b: sine = 16'hd8a1 ;
        8'h9c: sine = 16'hd767 ;
        8'h9d: sine = 16'hd634 ;
        8'h9e: sine = 16'hd506 ;
        8'h9f: sine = 16'hd3e0 ;
        8'ha0: sine = 16'hd2c0 ;
        8'ha1: sine = 16'hd1a7 ;
        8'ha2: sine = 16'hd095 ;
        8'ha3: sine = 16'hcf8b ;
        8'ha4: sine = 16'hce88 ;
        8'ha5: sine = 16'hcd8d ;
        8'ha6: sine = 16'hcc9a ;
        8'ha7: sine = 16'hcbae ;
        8'ha8: sine = 16'hcacb ;
        8'ha9: sine = 16'hc9ef ;
        8'haa: sine = 16'hc91c ;
        8'hab: sine = 16'hc852 ;
        8'hac: sine = 16'hc790 ;
        8'had: sine = 16'hc6d7 ;
        8'hae: sine = 16'hc626 ;
        8'haf: sine = 16'hc57f ;
        8'hb0: sine = 16'hc4e1 ;
        8'hb1: sine = 16'hc44b ;
        8'hb2: sine = 16'hc3bf ;
        8'hb3: sine = 16'hc33c ;
        8'hb4: sine = 16'hc2c3 ;
        8'hb5: sine = 16'hc253 ;
        8'hb6: sine = 16'hc1ec ;
        8'hb7: sine = 16'hc190 ;
        8'hb8: sine = 16'hc13c ;
        8'hb9: sine = 16'hc0f3 ;
        8'hba: sine = 16'hc0b3 ;
        8'hbb: sine = 16'hc07d ;
        8'hbc: sine = 16'hc050 ;
        8'hbd: sine = 16'hc02e ;
        8'hbe: sine = 16'hc015 ;
        8'hbf: sine = 16'hc006 ;
        8'hc0: sine = 16'hc001 ;
        8'hc1: sine = 16'hc006 ;
        8'hc2: sine = 16'hc015 ;
        8'hc3: sine = 16'hc02e ;
        8'hc4: sine = 16'hc050 ;
        8'hc5: sine = 16'hc07d ;
        8'hc6: sine = 16'hc0b3 ;
        8'hc7: sine = 16'hc0f3 ;
        8'hc8: sine = 16'hc13c ;
        8'hc9: sine = 16'hc190 ;
        8'hca: sine = 16'hc1ec ;
        8'hcb: sine = 16'hc253 ;
        8'hcc: sine = 16'hc2c3 ;
        8'hcd: sine = 16'hc33c ;
        8'hce: sine = 16'hc3bf ;
        8'hcf: sine = 16'hc44b ;
        8'hd0: sine = 16'hc4e1 ;
        8'hd1: sine = 16'hc57f ;
        8'hd2: sine = 16'hc626 ;
        8'hd3: sine = 16'hc6d7 ;
        8'hd4: sine = 16'hc790 ;
        8'hd5: sine = 16'hc852 ;
        8'hd6: sine = 16'hc91c ;
        8'hd7: sine = 16'hc9ef ;
        8'hd8: sine = 16'hcacb ;
        8'hd9: sine = 16'hcbae ;
        8'hda: sine = 16'hcc9a ;
        8'hdb: sine = 16'hcd8d ;
        8'hdc: sine = 16'hce88 ;
        8'hdd: sine = 16'hcf8b ;
        8'hde: sine = 16'hd095 ;
        8'hdf: sine = 16'hd1a7 ;
        8'he0: sine = 16'hd2c0 ;
        8'he1: sine = 16'hd3e0 ;
        8'he2: sine = 16'hd506 ;
        8'he3: sine = 16'hd634 ;
        8'he4: sine = 16'hd767 ;
        8'he5: sine = 16'hd8a1 ;
        8'he6: sine = 16'hd9e1 ;
        8'he7: sine = 16'hdb27 ;
        8'he8: sine = 16'hdc73 ;
        8'he9: sine = 16'hddc4 ;
        8'hea: sine = 16'hdf1a ;
        8'heb: sine = 16'he075 ;
        8'hec: sine = 16'he1d6 ;
        8'hed: sine = 16'he33b ;
        8'hee: sine = 16'he4a4 ;
        8'hef: sine = 16'he611 ;
        8'hf0: sine = 16'he783 ;
        8'hf1: sine = 16'he8f8 ;
        8'hf2: sine = 16'hea71 ;
        8'hf3: sine = 16'hebed ;
        8'hf4: sine = 16'hed6d ;
        8'hf5: sine = 16'heeef ;
        8'hf6: sine = 16'hf074 ;
        8'hf7: sine = 16'hf1fb ;
        8'hf8: sine = 16'hf384 ;
        8'hf9: sine = 16'hf510 ;
        8'hfa: sine = 16'hf69d ;
        8'hfb: sine = 16'hf82b ;
        8'hfc: sine = 16'hf9bb ;
        8'hfd: sine = 16'hfb4b ;
        8'hfe: sine = 16'hfcdd ;
        8'hff: sine = 16'hfe6e ;
    endcase
end
   
endmodule



module cosineTable (
    input [7:0] angle,             // 8-bit angle input (0-255, 0 to 360 degrees)
    output reg signed [15:0] cosine  // 16-bit signed output value (scaled by 8192)
);

always @(*) begin
    case(angle)
        8'h00: cosine = 16'h3fff ;
        8'h01: cosine = 16'h3ffa ;
        8'h02: cosine = 16'h3feb ;
        8'h03: cosine = 16'h3fd2 ;
        8'h04: cosine = 16'h3fb0 ;
        8'h05: cosine = 16'h3f83 ;
        8'h06: cosine = 16'h3f4d ;
        8'h07: cosine = 16'h3f0d ;
        8'h08: cosine = 16'h3ec4 ;
        8'h09: cosine = 16'h3e70 ;
        8'h0a: cosine = 16'h3e14 ;
        8'h0b: cosine = 16'h3dad ;
        8'h0c: cosine = 16'h3d3d ;
        8'h0d: cosine = 16'h3cc4 ;
        8'h0e: cosine = 16'h3c41 ;
        8'h0f: cosine = 16'h3bb5 ;
        8'h10: cosine = 16'h3b1f ;
        8'h11: cosine = 16'h3a81 ;
        8'h12: cosine = 16'h39da ;
        8'h13: cosine = 16'h3929 ;
        8'h14: cosine = 16'h3870 ;
        8'h15: cosine = 16'h37ae ;
        8'h16: cosine = 16'h36e4 ;
        8'h17: cosine = 16'h3611 ;
        8'h18: cosine = 16'h3535 ;
        8'h19: cosine = 16'h3452 ;
        8'h1a: cosine = 16'h3366 ;
        8'h1b: cosine = 16'h3273 ;
        8'h1c: cosine = 16'h3178 ;
        8'h1d: cosine = 16'h3075 ;
        8'h1e: cosine = 16'h2f6b ;
        8'h1f: cosine = 16'h2e59 ;
        8'h20: cosine = 16'h2d40 ;
        8'h21: cosine = 16'h2c20 ;
        8'h22: cosine = 16'h2afa ;
        8'h23: cosine = 16'h29cc ;
        8'h24: cosine = 16'h2899 ;
        8'h25: cosine = 16'h275f ;
        8'h26: cosine = 16'h261f ;
        8'h27: cosine = 16'h24d9 ;
        8'h28: cosine = 16'h238d ;
        8'h29: cosine = 16'h223c ;
        8'h2a: cosine = 16'h20e6 ;
        8'h2b: cosine = 16'h1f8b ;
        8'h2c: cosine = 16'h1e2a ;
        8'h2d: cosine = 16'h1cc5 ;
        8'h2e: cosine = 16'h1b5c ;
        8'h2f: cosine = 16'h19ef ;
        8'h30: cosine = 16'h187d ;
        8'h31: cosine = 16'h1708 ;
        8'h32: cosine = 16'h158f ;
        8'h33: cosine = 16'h1413 ;
        8'h34: cosine = 16'h1293 ;
        8'h35: cosine = 16'h1111 ;
        8'h36: cosine = 16'h0f8c ;
        8'h37: cosine = 16'h0e05 ;
        8'h38: cosine = 16'h0c7c ;
        8'h39: cosine = 16'h0af0 ;
        8'h3a: cosine = 16'h0963 ;
        8'h3b: cosine = 16'h07d5 ;
        8'h3c: cosine = 16'h0645 ;
        8'h3d: cosine = 16'h04b5 ;
        8'h3e: cosine = 16'h0323 ;
        8'h3f: cosine = 16'h0192 ;
        8'h40: cosine = 16'h0000 ;
        8'h41: cosine = 16'hfe6e ;
        8'h42: cosine = 16'hfcdd ;
        8'h43: cosine = 16'hfb4b ;
        8'h44: cosine = 16'hf9bb ;
        8'h45: cosine = 16'hf82b ;
        8'h46: cosine = 16'hf69d ;
        8'h47: cosine = 16'hf510 ;
        8'h48: cosine = 16'hf384 ;
        8'h49: cosine = 16'hf1fb ;
        8'h4a: cosine = 16'hf074 ;
        8'h4b: cosine = 16'heeef ;
        8'h4c: cosine = 16'hed6d ;
        8'h4d: cosine = 16'hebed ;
        8'h4e: cosine = 16'hea71 ;
        8'h4f: cosine = 16'he8f8 ;
        8'h50: cosine = 16'he783 ;
        8'h51: cosine = 16'he611 ;
        8'h52: cosine = 16'he4a4 ;
        8'h53: cosine = 16'he33b ;
        8'h54: cosine = 16'he1d6 ;
        8'h55: cosine = 16'he075 ;
        8'h56: cosine = 16'hdf1a ;
        8'h57: cosine = 16'hddc4 ;
        8'h58: cosine = 16'hdc73 ;
        8'h59: cosine = 16'hdb27 ;
        8'h5a: cosine = 16'hd9e1 ;
        8'h5b: cosine = 16'hd8a1 ;
        8'h5c: cosine = 16'hd767 ;
        8'h5d: cosine = 16'hd634 ;
        8'h5e: cosine = 16'hd506 ;
        8'h5f: cosine = 16'hd3e0 ;
        8'h60: cosine = 16'hd2c0 ;
        8'h61: cosine = 16'hd1a7 ;
        8'h62: cosine = 16'hd095 ;
        8'h63: cosine = 16'hcf8b ;
        8'h64: cosine = 16'hce88 ;
        8'h65: cosine = 16'hcd8d ;
        8'h66: cosine = 16'hcc9a ;
        8'h67: cosine = 16'hcbae ;
        8'h68: cosine = 16'hcacb ;
        8'h69: cosine = 16'hc9ef ;
        8'h6a: cosine = 16'hc91c ;
        8'h6b: cosine = 16'hc852 ;
        8'h6c: cosine = 16'hc790 ;
        8'h6d: cosine = 16'hc6d7 ;
        8'h6e: cosine = 16'hc626 ;
        8'h6f: cosine = 16'hc57f ;
        8'h70: cosine = 16'hc4e1 ;
        8'h71: cosine = 16'hc44b ;
        8'h72: cosine = 16'hc3bf ;
        8'h73: cosine = 16'hc33c ;
        8'h74: cosine = 16'hc2c3 ;
        8'h75: cosine = 16'hc253 ;
        8'h76: cosine = 16'hc1ec ;
        8'h77: cosine = 16'hc190 ;
        8'h78: cosine = 16'hc13c ;
        8'h79: cosine = 16'hc0f3 ;
        8'h7a: cosine = 16'hc0b3 ;
        8'h7b: cosine = 16'hc07d ;
        8'h7c: cosine = 16'hc050 ;
        8'h7d: cosine = 16'hc02e ;
        8'h7e: cosine = 16'hc015 ;
        8'h7f: cosine = 16'hc006 ;
        8'h80: cosine = 16'hc001 ;
        8'h81: cosine = 16'hc006 ;
        8'h82: cosine = 16'hc015 ;
        8'h83: cosine = 16'hc02e ;
        8'h84: cosine = 16'hc050 ;
        8'h85: cosine = 16'hc07d ;
        8'h86: cosine = 16'hc0b3 ;
        8'h87: cosine = 16'hc0f3 ;
        8'h88: cosine = 16'hc13c ;
        8'h89: cosine = 16'hc190 ;
        8'h8a: cosine = 16'hc1ec ;
        8'h8b: cosine = 16'hc253 ;
        8'h8c: cosine = 16'hc2c3 ;
        8'h8d: cosine = 16'hc33c ;
        8'h8e: cosine = 16'hc3bf ;
        8'h8f: cosine = 16'hc44b ;
        8'h90: cosine = 16'hc4e1 ;
        8'h91: cosine = 16'hc57f ;
        8'h92: cosine = 16'hc626 ;
        8'h93: cosine = 16'hc6d7 ;
        8'h94: cosine = 16'hc790 ;
        8'h95: cosine = 16'hc852 ;
        8'h96: cosine = 16'hc91c ;
        8'h97: cosine = 16'hc9ef ;
        8'h98: cosine = 16'hcacb ;
        8'h99: cosine = 16'hcbae ;
        8'h9a: cosine = 16'hcc9a ;
        8'h9b: cosine = 16'hcd8d ;
        8'h9c: cosine = 16'hce88 ;
        8'h9d: cosine = 16'hcf8b ;
        8'h9e: cosine = 16'hd095 ;
        8'h9f: cosine = 16'hd1a7 ;
        8'ha0: cosine = 16'hd2c0 ;
        8'ha1: cosine = 16'hd3e0 ;
        8'ha2: cosine = 16'hd506 ;
        8'ha3: cosine = 16'hd634 ;
        8'ha4: cosine = 16'hd767 ;
        8'ha5: cosine = 16'hd8a1 ;
        8'ha6: cosine = 16'hd9e1 ;
        8'ha7: cosine = 16'hdb27 ;
        8'ha8: cosine = 16'hdc73 ;
        8'ha9: cosine = 16'hddc4 ;
        8'haa: cosine = 16'hdf1a ;
        8'hab: cosine = 16'he075 ;
        8'hac: cosine = 16'he1d6 ;
        8'had: cosine = 16'he33b ;
        8'hae: cosine = 16'he4a4 ;
        8'haf: cosine = 16'he611 ;
        8'hb0: cosine = 16'he783 ;
        8'hb1: cosine = 16'he8f8 ;
        8'hb2: cosine = 16'hea71 ;
        8'hb3: cosine = 16'hebed ;
        8'hb4: cosine = 16'hed6d ;
        8'hb5: cosine = 16'heeef ;
        8'hb6: cosine = 16'hf074 ;
        8'hb7: cosine = 16'hf1fb ;
        8'hb8: cosine = 16'hf384 ;
        8'hb9: cosine = 16'hf510 ;
        8'hba: cosine = 16'hf69d ;
        8'hbb: cosine = 16'hf82b ;
        8'hbc: cosine = 16'hf9bb ;
        8'hbd: cosine = 16'hfb4b ;
        8'hbe: cosine = 16'hfcdd ;
        8'hbf: cosine = 16'hfe6e ;
        8'hc0: cosine = 16'h0000 ;
        8'hc1: cosine = 16'h0192 ;
        8'hc2: cosine = 16'h0323 ;
        8'hc3: cosine = 16'h04b5 ;
        8'hc4: cosine = 16'h0645 ;
        8'hc5: cosine = 16'h07d5 ;
        8'hc6: cosine = 16'h0963 ;
        8'hc7: cosine = 16'h0af0 ;
        8'hc8: cosine = 16'h0c7c ;
        8'hc9: cosine = 16'h0e05 ;
        8'hca: cosine = 16'h0f8c ;
        8'hcb: cosine = 16'h1111 ;
        8'hcc: cosine = 16'h1293 ;
        8'hcd: cosine = 16'h1413 ;
        8'hce: cosine = 16'h158f ;
        8'hcf: cosine = 16'h1708 ;
        8'hd0: cosine = 16'h187d ;
        8'hd1: cosine = 16'h19ef ;
        8'hd2: cosine = 16'h1b5c ;
        8'hd3: cosine = 16'h1cc5 ;
        8'hd4: cosine = 16'h1e2a ;
        8'hd5: cosine = 16'h1f8b ;
        8'hd6: cosine = 16'h20e6 ;
        8'hd7: cosine = 16'h223c ;
        8'hd8: cosine = 16'h238d ;
        8'hd9: cosine = 16'h24d9 ;
        8'hda: cosine = 16'h261f ;
        8'hdb: cosine = 16'h275f ;
        8'hdc: cosine = 16'h2899 ;
        8'hdd: cosine = 16'h29cc ;
        8'hde: cosine = 16'h2afa ;
        8'hdf: cosine = 16'h2c20 ;
        8'he0: cosine = 16'h2d40 ;
        8'he1: cosine = 16'h2e59 ;
        8'he2: cosine = 16'h2f6b ;
        8'he3: cosine = 16'h3075 ;
        8'he4: cosine = 16'h3178 ;
        8'he5: cosine = 16'h3273 ;
        8'he6: cosine = 16'h3366 ;
        8'he7: cosine = 16'h3452 ;
        8'he8: cosine = 16'h3535 ;
        8'he9: cosine = 16'h3611 ;
        8'hea: cosine = 16'h36e4 ;
        8'heb: cosine = 16'h37ae ;
        8'hec: cosine = 16'h3870 ;
        8'hed: cosine = 16'h3929 ;
        8'hee: cosine = 16'h39da ;
        8'hef: cosine = 16'h3a81 ;
        8'hf0: cosine = 16'h3b1f ;
        8'hf1: cosine = 16'h3bb5 ;
        8'hf2: cosine = 16'h3c41 ;
        8'hf3: cosine = 16'h3cc4 ;
        8'hf4: cosine = 16'h3d3d ;
        8'hf5: cosine = 16'h3dad ;
        8'hf6: cosine = 16'h3e14 ;
        8'hf7: cosine = 16'h3e70 ;
        8'hf8: cosine = 16'h3ec4 ;
        8'hf9: cosine = 16'h3f0d ;
        8'hfa: cosine = 16'h3f4d ;
        8'hfb: cosine = 16'h3f83 ;
        8'hfc: cosine = 16'h3fb0 ;
        8'hfd: cosine = 16'h3fd2 ;
        8'hfe: cosine = 16'h3feb ;
        8'hff: cosine = 16'h3ffa ;
    endcase
end
   
endmodule

