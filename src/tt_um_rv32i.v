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
  // Pin Mux (Dynamic Pins)
  // -------------------------------------------------------------------------
  wire [7:0] pin_in  = ui_in[7:0];
  wire [4:0] pin_out;
  
  wire [3:0] pin_uio_in = uio_in[7:4];
  wire [3:0] pin_uio_out;
  wire [3:0] pin_uio_oe;

  assign uo_out[0]   = pin_out[0];
  assign uo_out[7:4] = pin_out[4:1];
  
  assign uio_out[7:4] = pin_uio_out;
  assign uio_oe[7:4]  = pin_uio_oe;

  // -------------------------------------------------------------------------
  // RV5 SoC Instance
  // -------------------------------------------------------------------------
  rv5 #(
    .CLOCK_FREQUENCY     (50000000),
    .UART_BAUD_RATE      (9600),
    .MEMORY_SIZE         (64),
    .BOOT_ADDRESS        (32'h00000000),
    .GPIO_WIDTH          (8),
    .SPI_NUM_CHIP_SELECT (1),
    .UART1_BAUD_RATE     (9600)
  ) rv5_inst (
    .clock       (clk),
    .reset       (reset_sync),
    .halt        (1'b0),

    // Pin Mux Interface
    .pin_in      (pin_in),
    .pin_out     (pin_out),
    .pin_uio_in  (pin_uio_in),
    .pin_uio_out (pin_uio_out),
    .pin_uio_oe  (pin_uio_oe),

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
