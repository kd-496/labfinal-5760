module DE1_SoC_Computer (
    // FPGA Pins
    input  wire CLOCK_50,
    input  wire CLOCK2_50,
    input  wire CLOCK3_50,
    input  wire CLOCK4_50,
    output wire ADC_DIN,
    input  wire ADC_DOUT,
    inout  wire ADC_CS_N,
    output wire ADC_SCLK,
    input  wire AUD_ADCDAT,
    inout  wire AUD_ADCLRCK,
    inout  wire AUD_BCLK,
    output wire AUD_DACDAT,
    inout  wire AUD_DACLRCK,
    output wire AUD_XCK,
    output wire [12:0] DRAM_ADDR,
    output wire [1:0] DRAM_BA,
    output wire DRAM_CAS_N,
    output wire DRAM_CKE,
    output wire DRAM_CLK,
    output wire DRAM_CS_N,
    inout  wire [15:0] DRAM_DQ,
    output wire DRAM_LDQM,
    output wire DRAM_RAS_N,
    output wire DRAM_UDQM,
    output wire DRAM_WE_N,
    output wire FPGA_I2C_SCLK,
    inout  wire FPGA_I2C_SDAT,
    inout  wire [35:0] GPIO_0,
    inout  wire [35:0] GPIO_1,
    output wire [6:0] HEX0,
    output wire [6:0] HEX1,
    output wire [6:0] HEX2,
    output wire [6:0] HEX3,
    output wire [6:0] HEX4,
    output wire [6:0] HEX5,
    input  wire IRDA_RXD,
    output wire IRDA_TXD,
    input  wire [3:0] KEY,
    output wire [9:0] LEDR,
    inout  wire PS2_CLK,
    inout  wire PS2_DAT,
    inout  wire PS2_CLK2,
    inout  wire PS2_DAT2,
    input  wire TD_CLK27,
    input  wire [7:0] TD_DATA,
    input  wire TD_HS,
    output wire TD_RESET_N,
    input  wire TD_VS,
    output wire [7:0] VGA_B,
    output wire VGA_BLANK_N,
    output wire VGA_CLK,
    output wire [7:0] VGA_G,
    output wire VGA_HS,
    output wire [7:0] VGA_R,
    output wire VGA_SYNC_N,
    output wire VGA_VS
);

//=======================================================
//  PARAMETER declarations
//=======================================================

parameter ENEMY_POSITION_WIDTH = 16;

//=======================================================
//  PORT declarations
//=======================================================

output wire [ENEMY_POSITION_WIDTH-1:0] enemy_x;
output wire [ENEMY_POSITION_WIDTH-1:0] enemy_y;

//=======================================================
//  REG/WIRE declarations
//=======================================================

reg [ENEMY_POSITION_WIDTH-1:0] enemy_x_reg;
reg [ENEMY_POSITION_WIDTH-1:0] enemy_y_reg;

//=======================================================
//  Enemy Orbital Motion Calculation
//=======================================================

// Constants for fixed-point arithmetic
localparam INTEGER_BITS = 8;
localparam FRACTIONAL_BITS = ENEMY_POSITION_WIDTH - INTEGER_BITS;

// Constants for enemy orbit parameters
localparam ORBIT_RADIUS = 200;
localparam ANGULAR_SPEED = 0.01;

// Clock divider for updating enemy position
reg [31:0] clk_divider = 0;

always @(posedge CLOCK_50) begin
    if (clk_divider == 0) begin
        // Calculate new enemy position
        enemy_x_reg <= ORBIT_RADIUS * $cos(32'h0 + clk_divider * ANGULAR_SPEED);
        enemy_y_reg <= ORBIT_RADIUS * $sin(32'h0 + clk_divider * ANGULAR_SPEED);
    end
    // Increment clock divider
    clk_divider <= clk_divider + 1;
end

// Output enemy position
assign enemy_x = enemy_x_reg;
assign enemy_y = enemy_y_reg;

//=======================================================
//  Structural coding
//=======================================================

Computer_System The_System (
    // FPGA Side
    .system_pll_ref_clk_clk (CLOCK_50),
    .system_pll_ref_reset_reset (1'b0),
    .av_config_SCLK (FPGA_I2C_SCLK),
    .av_config_SDAT (FPGA_I2C_SDAT),
    .vga_pll_ref_clk_clk (CLOCK2_50),
    .vga_pll_ref_reset_reset (1'b0),
    .vga_CLK (VGA_CLK),
    .vga_BLANK (VGA_BLANK_N),
    .vga_SYNC (VGA_SYNC_N),
    .vga_HS (VGA_HS),
    .vga_VS (VGA_VS),
    .vga_R (VGA_R),
    .vga_G (VGA_G),
    .vga_B (VGA_B),
    .sdram_clk_clk (DRAM_CLK),
    .sdram_addr (DRAM_ADDR),
    .sdram_ba (DRAM_BA),
    .sdram_cas_n (DRAM_CAS_N),
    .sdram_cke (DRAM_CKE),
    .sdram_cs_n (DRAM_CS_N),
    .sdram_dq (DRAM_DQ),
    .sdram_dqm ({DRAM_UDQM, DRAM_LDQM}),
    .sdram_ras_n (DRAM_RAS_N),
    .sdram_we_n (DRAM_WE_N),
    // HPS Side
    .memory_mem_a (HPS_DDR3_ADDR),
    .memory_mem_ba (HPS_DDR3_BA),
    .memory_mem_ck (HPS_DDR3_CK_P),
    .memory_mem_ck_n (HPS_DDR3_CK_N),
    .memory_mem_cke (HPS_DDR3_CKE),
    .memory_mem_cs_n (HPS_DDR3_CS_N),
    .memory_mem_ras_n (HPS_DDR3_RAS_N),
    .memory_mem_cas_n (HPS_DDR3_CAS_N),
    .memory_mem_we_n (HPS_DDR3_WE_N),
    .memory_mem_reset_n (HPS_DDR3_RESET_N),
    .memory_mem_dq (HPS_DDR3_DQ),
    .memory_mem_dqs (HPS_DDR3_DQS_P),
    .memory_mem_dqs_n (HPS_DDR3_DQS_N),
    .memory_mem_odt (HPS_DDR3_ODT),
    .memory_mem_dm (HPS_DDR3_DM),
    .memory_oct_rzqin (HPS_DDR3_RZQ),
    .hps_io_hps_io_gpio_inst_GPIO35 (HPS_ENET_INT_N),
    .hps_io_hps_io_emac1_inst_TX_CLK (HPS_ENET_GTX_CLK),
    .hps_io_hps_io_emac1_inst_TXD0 (HPS_ENET_TX_DATA[0]),
    .hps_io_hps_io_emac1_inst_TXD1 (HPS_ENET_TX_DATA[1]),
    .hps_io_hps_io_emac1_inst_TXD2 (HPS_ENET_TX_DATA[2]),
    .hps_io_hps_io_emac1_inst_TXD3 (HPS_ENET_TX_DATA[3]),
    .hps_io_hps_io_emac1_inst_RXD0 (HPS_ENET_RX_DATA[0]),
    .hps_io_hps_io_emac1_inst_MDIO (HPS_ENET_MDIO),
    .hps_io_hps_io_emac1_inst_MDC (HPS_ENET_MDC),
    .hps_io_hps_io_emac1_inst_RX_CTL (HPS_ENET_RX_DV),
    .hps_io_hps_io_emac1_inst_TX_CTL (HPS_ENET_TX_EN),
    .hps_io_hps_io_emac1_inst_RX_CLK (HPS_ENET_RX_CLK),
    .hps_io_hps_io_emac1_inst_RXD1 (HPS_ENET_RX_DATA[1]),
    .hps_io_hps_io_emac1_inst_RXD2 (HPS_ENET_RX_DATA[2]),
    .hps_io_hps_io_emac1_inst_RXD3 (HPS_ENET_RX_DATA[3]),
    .hps_io_hps_io_qspi_inst_IO0 (HPS_FLASH_DATA[0]),
    .hps_io_hps_io_qspi_inst_IO1 (HPS_FLASH_DATA[1]),
    .hps_io_hps_io_qspi_inst_IO2 (HPS_FLASH_DATA[2]),
    .hps_io_hps_io_qspi_inst_IO3 (HPS_FLASH_DATA[3]),
    .hps_io_hps_io_qspi_inst_SS0 (HPS_FLASH_NCSO),
    .hps_io_hps_io_qspi_inst_CLK (HPS_FLASH_DCLK),
    .hps_io_hps_io_gpio_inst_GPIO61 (HPS_GSENSOR_INT),
    .hps_io_hps_io_gpio_inst_GPIO40 (HPS_GPIO[0]),
    .hps_io_hps_io_gpio_inst_GPIO41 (HPS_GPIO[1]),
    .hps_io_hps_io_gpio_inst_GPIO48 (HPS_I2C_CONTROL),
    .hps_io_hps_io_i2c0_inst_SDA (HPS_I2C1_SDAT),
    .hps_io_hps_io_i2c0_inst_SCL (HPS_I2C1_SCLK),
    .hps_io_hps_io_i2c1_inst_SDA (HPS_I2C2_SDAT),
    .hps_io_hps_io_i2c1_inst_SCL (HPS_I2C2_SCLK),
    .hps_io_hps_io_gpio_inst_GPIO54 (HPS_KEY),
    .hps_io_hps_io_gpio_inst_GPIO53 (HPS_LED),
    .hps_io_hps_io_sdio_inst_CMD (HPS_SD_CMD),
    .hps_io_hps_io_sdio_inst_D0 (HPS_SD_DATA[0]),
    .hps_io_hps_io_sdio_inst_D1 (HPS_SD_DATA[1]),
    .hps_io_hps_io_sdio_inst_CLK (HPS_SD_CLK),
    .hps_io_hps_io_sdio_inst_D2 (HPS_SD_DATA[2]),
    .hps_io_hps_io_sdio_inst_D3 (HPS_SD_DATA[3]),
    .hps_io_hps_io_spim1_inst_CLK (HPS_SPIM_CLK),
    .hps_io_hps_io_spim1_inst_MOSI (HPS_SPIM_MOSI),
    .hps_io_hps_io_spim1_inst_MISO (HPS_SPIM_MISO),
    .hps_io_hps_io_spim1_inst_SS0 (HPS_SPIM_SS),
    .hps_io_hps_io_uart0_inst_RX (HPS_UART_RX),
    .hps_io_hps_io_uart0_inst_TX (HPS_UART_TX),
    .hps_io_hps_io_gpio_inst_GPIO09 (HPS_CONV_USB_N),
    .hps_io_hps_io_usb1_inst_D0 (HPS_USB_DATA[0]),
    .hps_io_hps_io_usb1_inst_D1 (HPS_USB_DATA[1]),
    .hps_io_hps_io_usb1_inst_D2 (HPS_USB_DATA[2]),
    .hps_io_hps_io_usb1_inst_D3 (HPS_USB_DATA[3]),
    .hps_io_hps_io_usb1_inst_D4 (HPS_USB_DATA[4]),
    .hps_io_hps_io_usb1_inst_D5 (HPS_USB_DATA[5]),
    .hps_io_hps_io_usb1_inst_D6 (HPS_USB_DATA[6]),
    .hps_io_hps_io_usb1_inst_D7 (HPS_USB_DATA[7]),
    .hps_io_hps_io_usb1_inst_CLK (HPS_USB_CLKOUT),
    .hps_io_hps_io_usb1_inst_STP (HPS_USB_STP),
    .hps_io_hps_io_usb1_inst_DIR (HPS_USB_DIR),
    .hps_io_hps_io_usb1_inst_NXT (HPS_USB_NXT)
);

endmodule
