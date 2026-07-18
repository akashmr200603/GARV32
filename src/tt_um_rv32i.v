// SPDX-License-Identifier: MIT
// Copyright (c) 2020-2025 RV5 Project Contributors
//
// Tiny Tapeout Top-Level Wrapper for RV5 RISC-V SoC
//
// Pin Mapping:
// ============================================================================
// Signal         | TT Pin         | Direction | Description
// --------------------------------------------------------------------------
// pin_in[7:0]    | ui_in[7:0]     | Input     | Dynamic Pin Mux Inputs
// --------------------------------------------------------------------------
// pin_out[0]     | uo_out[0]      | Output    | Dynamic Pin Mux Output 0
// qspi_sck       | uo_out[1]      | Output    | QSPI Serial Clock (Dedicated)
// qspi_cs_n      | uo_out[2]      | Output    | QSPI Flash CS (Dedicated)
// qspi_cs_ram_n  | uo_out[3]      | Output    | QSPI PSRAM CS (Dedicated)
// pin_out[4:1]   | uo_out[7:4]    | Output    | Dynamic Pin Mux Output 4:1
// --------------------------------------------------------------------------
// qspi_io[3:0]   | uio[3:0]       | Bidir     | QSPI Data Lines (Dedicated)
// pin_uio[3:0]   | uio[7:4]       | Bidir     | Dynamic Pin Mux Bidir 3:0
// ============================================================================

module tt_um_rv32i (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (1=output, 0=input)
    input  wire       ena,      // Always 1 when design is powered
    input  wire       clk,      // Clock
    input  wire       rst_n     // Reset (active low)
);

  // -------------------------------------------------------------------------
  // Reset Synchronizer (Power-On-Reset)
  // -------------------------------------------------------------------------
  wire reset_sync;

  rv5_por por_inst (
    .clock       (clk),
    .ext_reset_n (rst_n),
    .sys_reset   (reset_sync)
  );

  // -------------------------------------------------------------------------
  // QSPI (Dedicated Pins)
  // -------------------------------------------------------------------------
  wire       qspi_sck;
  wire       qspi_cs_n;
  wire       qspi_cs_ram_n;
  wire [3:0] qspi_io_in;
  wire [3:0] qspi_io_out;
  wire       qspi_io_oe;

  assign uo_out[1] = qspi_sck;
  assign uo_out[2] = qspi_cs_n;
  assign uo_out[3] = qspi_cs_ram_n;

  assign uio_out[3:0] = qspi_io_out;
  assign uio_oe[3:0]  = {4{qspi_io_oe}};
  assign qspi_io_in   = uio_in[3:0];

  // -------------------------------------------------------------------------
  // Hardwired Peripherals
  // -------------------------------------------------------------------------

  // UART0
  wire uart0_rx = ui_in[0];
  wire uart0_tx;
  assign uo_out[0] = uart0_tx;

  // UART1
  wire uart1_rx = ui_in[1];
  wire uart1_tx;
  assign uo_out[4] = uart1_tx;

  // SPI0
  wire spi0_miso = ui_in[2];
  wire spi0_mosi;
  wire spi0_sck;
  wire spi0_cs;
  assign uo_out[5] = spi0_sck;
  assign uo_out[6] = spi0_mosi;
  assign uo_out[7] = spi0_cs;

  // I2C0
  wire i2c0_sda_i = uio_in[4];
  wire i2c0_sda_o;
  wire i2c0_sda_oe;
  assign uio_out[4] = i2c0_sda_o;
  assign uio_oe[4]  = i2c0_sda_oe;

  wire i2c0_scl_i = uio_in[5];
  wire i2c0_scl_o;
  wire i2c0_scl_oe;
  assign uio_out[5] = i2c0_scl_o;
  assign uio_oe[5]  = i2c0_scl_oe;

  // PWM
  wire [7:0] pwm_out;
  assign uio_out[6] = pwm_out[0];
  assign uio_oe[6]  = 1'b1; // Force bidir as output

  // GPIO
  wire [3:0] gpio_in;
  wire [3:0] gpio_out;
  wire [3:0] gpio_oe;

  assign gpio_in[0] = ui_in[3];
  assign gpio_in[1] = ui_in[4];
  assign gpio_in[2] = ui_in[5];
  assign gpio_in[3] = uio_in[7];
  
  assign uio_out[7] = gpio_out[3];
  assign uio_oe[7]  = gpio_oe[3];

  // -------------------------------------------------------------------------
  // RV5 SoC Instance
  // -------------------------------------------------------------------------
  rv5 #(
    .CLOCK_FREQUENCY     (50000000),
    .UART_BAUD_RATE      (9600),
    .MEMORY_SIZE         (64),
    .BOOT_ADDRESS        (32'h00000000),
    .GPIO_WIDTH          (4),        // Match the 4 internal GPIO lines in Pin Mux
    .SPI_NUM_CHIP_SELECT (1),
    .UART1_BAUD_RATE     (9600)
  ) rv5_inst (
    .clock       (clk),
    .reset       (reset_sync),
    .halt        (1'b0),

    // Hardwired Interface
    .uart_rx        (uart0_rx),
    .uart_tx        (uart0_tx),
    .uart1_rx       (uart1_rx),
    .uart1_tx       (uart1_tx),
    .poci           (spi0_miso),
    .pico           (spi0_mosi),
    .sclk           (spi0_sck),
    .cs             (spi0_cs),
    .gpio_input_internal (gpio_in),
    .gpio_output_internal(gpio_out),
    .gpio_oe_internal    (gpio_oe),
    .i2c0_sda_i     (i2c0_sda_i),
    .i2c0_sda_o     (i2c0_sda_o),
    .i2c0_sda_oe    (i2c0_sda_oe),
    .i2c0_scl_i     (i2c0_scl_i),
    .i2c0_scl_o     (i2c0_scl_o),
    .i2c0_scl_oe    (i2c0_scl_oe),
    .pwm_out        (pwm_out),

    // QSPI (directly routed to pads)
    .qspi_sck      (qspi_sck),
    .qspi_cs_n     (qspi_cs_n),
    .qspi_cs_ram_n (qspi_cs_ram_n),
    .qspi_io_in    (qspi_io_in),
    .qspi_io_out   (qspi_io_out),
    .qspi_io_oe    (qspi_io_oe)
  );

  // Avoid warnings about unused signals
  wire unused_ok = &{1'b0, ena, 1'b0};

endmodule
