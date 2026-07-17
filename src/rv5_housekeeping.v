// SPDX-License-Identifier: MIT
// Housekeeping Peripheral for RV5 SoC
//
// Provides read-only identification and status registers:
//   0x00 : CHIP_ID     - ASCII "RV50" (0x52563530)
//   0x04 : VERSION     - Firmware/RTL version (Major.Minor.Patch)
//   0x08 : RESET_CAUSE - Source of last reset (0=POR, 1=External, 2=Watchdog)
//   0x0C : CLK_STATUS  - Clock/PLL status flags
//   0x10 : TILE_INFO   - Tiny Tapeout tile configuration

module rv5_housekeeping (
  input  wire        clock,
  input  wire        reset,

  // Bus interface
  input  wire [4:0]  rw_address,
  output reg  [31:0] read_data,
  input  wire        read_request,
  output reg         read_response,
  input  wire [31:0] write_data,
  input  wire [3:0]  write_strobe,
  input  wire        write_request,
  output reg         write_response
);

  // Register addresses
  localparam REG_CHIP_ID     = 5'h00;
  localparam REG_VERSION     = 5'h04;
  localparam REG_RESET_CAUSE = 5'h08;
  localparam REG_CLK_STATUS  = 5'h0C;
  localparam REG_TILE_INFO   = 5'h10;

  // Writable scratch register (for software self-test)
  reg [31:0] scratch_reg;

  always @(posedge clock) begin
    if (reset) begin
      read_response  <= 1'b0;
      write_response <= 1'b0;
      read_data      <= 32'h0;
      scratch_reg    <= 32'h0;
    end else begin
      read_response  <= read_request;
      write_response <= write_request;

      // Write path (only scratch register is writable)
      if (write_request && rw_address == 5'h14 && &write_strobe)
        scratch_reg <= write_data;

      // Read path
      if (read_request) begin
        case (rw_address)
          REG_CHIP_ID:     read_data <= 32'h52563530; // "RV50"
          REG_VERSION:     read_data <= 32'h00_01_00_00; // v1.0.0
          REG_RESET_CAUSE: read_data <= 32'h0000_0000; // POR
          REG_CLK_STATUS:  read_data <= 32'h0000_0001; // Clock stable
          REG_TILE_INFO:   read_data <= 32'h0000_0033; // 3x3 TT tile
          5'h14:           read_data <= scratch_reg;
          default:         read_data <= 32'hDEAD_BEEF;
        endcase
      end
    end
  end

  // Avoid warnings about unused signals
  wire unused_ok = &{1'b0, write_data, write_strobe, 1'b0};

endmodule
