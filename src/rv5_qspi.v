// SPDX-License-Identifier: MIT
// QSPI XIP Controller for external Flash (4MB) and PSRAM (8MB)
//
// Supports:
//   - Quad Read  (0xEB) for Flash and PSRAM reads
//   - Quad Write (0x38) for PSRAM writes
//   - 24-bit addressing (up to 16MB per device)
//   - Arbitration: Data memory port has priority over Instruction Fetch
//
// Pin Interface:
//   qspi_sck       - Serial Clock
//   qspi_cs_n      - Chip Select for Flash (active low)
//   qspi_cs_ram_n   - Chip Select for PSRAM (active low)
//   qspi_io[3:0]   - Quad data lines (directly directly directly directly directly directly directly shared between Flash and PSRAM)
//   qspi_io_oe     - Output Enable for qspi_io (1 = driving, 0 = receiving)

module rv5_qspi (
  input  wire        clock,
  input  wire        reset,

  // Port A: Instruction Fetch (Read-Only)
  input  wire [31:0] if_address,
  output reg  [31:0] if_read_data,
  input  wire        if_request,
  output reg         if_response,

  // Port B: Data Memory (Read/Write)
  input  wire [31:0] mem_address,
  output reg  [31:0] mem_read_data,
  input  wire        mem_read_request,
  output reg         mem_read_response,
  input  wire [31:0] mem_write_data,
  input  wire [3:0]  mem_write_strobe,
  input  wire        mem_write_request,
  output reg         mem_write_response,

  // QSPI Pad Interface
  output reg         qspi_sck,
  output reg         qspi_cs_n,
  output reg         qspi_cs_ram_n,
  input  wire [3:0]  qspi_io_in,
  output reg  [3:0]  qspi_io_out,
  output reg         qspi_io_oe
);

  // -------------------------------------------------------------------------
  // State Machine
  // -------------------------------------------------------------------------
  localparam [2:0] S_IDLE  = 3'd0,
                   S_CMD   = 3'd1,
                   S_ADDR  = 3'd2,
                   S_DUMMY = 3'd3,
                   S_DATA  = 3'd4,
                   S_DONE  = 3'd5;

  reg [2:0]  state;
  reg [3:0]  bit_cnt;
  reg [31:0] shift_out;
  reg [31:0] shift_in;
  reg        serving_mem;     // 1 = serving data port, 0 = serving IF port
  reg        serving_write;   // 1 = write transaction
  reg        target_is_ram;   // 1 = PSRAM, 0 = Flash
  reg        sck_en;          // Gate the SCK output

  // Arbitration: data port has priority
  wire mem_req      = mem_read_request | mem_write_request;
  wire active_if    = if_request & ~mem_req;
  wire any_request  = mem_req | active_if;

  // Address selection
  wire [31:0] sel_addr = mem_req ? mem_address : if_address;

  // PSRAM lives at 0x1000_0000; Flash at 0x0000_0000
  wire addr_is_ram = (sel_addr[31:28] == 4'h1);

  // Command bytes
  wire [7:0] cmd_byte = serving_write ? 8'h38 : 8'hEB;

  always @(posedge clock) begin
    if (reset) begin
      state            <= S_IDLE;
      qspi_cs_n        <= 1'b1;
      qspi_cs_ram_n    <= 1'b1;
      qspi_sck         <= 1'b0;
      qspi_io_oe       <= 1'b0;
      qspi_io_out      <= 4'h0;
      sck_en           <= 1'b0;
      if_response      <= 1'b0;
      mem_read_response  <= 1'b0;
      mem_write_response <= 1'b0;
      serving_mem      <= 1'b0;
      serving_write    <= 1'b0;
      target_is_ram    <= 1'b0;
      bit_cnt          <= 4'd0;
      shift_out        <= 32'd0;
      shift_in         <= 32'd0;
    end else begin
      // Default: clear single-cycle response pulses
      if_response        <= 1'b0;
      mem_read_response  <= 1'b0;
      mem_write_response <= 1'b0;

      case (state)
        // -----------------------------------------------------------------
        S_IDLE: begin
          qspi_sck  <= 1'b0;
          sck_en    <= 1'b0;
          if (any_request) begin
            serving_mem   <= mem_req;
            serving_write <= mem_write_request;
            target_is_ram <= addr_is_ram;

            // Assert the correct chip select
            if (addr_is_ram)
              qspi_cs_ram_n <= 1'b0;
            else
              qspi_cs_n <= 1'b0;

            // Load command byte (sent 1-bit SPI on IO[0])
            shift_out <= {(serving_write ? 8'h38 : 8'hEB), sel_addr[23:0]};
            bit_cnt   <= 4'd7; // 8 bits of command
            qspi_io_oe <= 1'b1;
            sck_en     <= 1'b1;
            state      <= S_CMD;
          end
        end

        // -----------------------------------------------------------------
        // Command phase: send 8-bit command on IO[0] (standard SPI mode)
        S_CMD: begin
          qspi_sck <= ~qspi_sck;
          if (qspi_sck) begin // Falling edge = setup next bit
            qspi_io_out <= {3'b0, shift_out[31]};
            shift_out   <= {shift_out[30:0], 1'b0};
            if (bit_cnt == 4'd0) begin
              // Reload shift_out with the 24-bit address
              shift_out <= {sel_addr[23:0], 8'h0};
              bit_cnt   <= 4'd5; // 6 nibbles = 24 bits in Quad mode
              state     <= S_ADDR;
            end else begin
              bit_cnt <= bit_cnt - 4'd1;
            end
          end
        end

        // -----------------------------------------------------------------
        // Address phase: send 24-bit address in Quad mode (4 bits/cycle)
        S_ADDR: begin
          qspi_sck <= ~qspi_sck;
          if (qspi_sck) begin
            qspi_io_out <= shift_out[31:28];
            shift_out   <= {shift_out[27:0], 4'h0};
            if (bit_cnt == 4'd0) begin
              bit_cnt <= 4'd5; // 6 dummy cycles
              state   <= S_DUMMY;
              // During dummy cycles: tri-state for reads, keep driving for writes
              qspi_io_oe <= serving_write;
            end else begin
              bit_cnt <= bit_cnt - 4'd1;
            end
          end
        end

        // -----------------------------------------------------------------
        // Dummy cycles (required by Quad Read commands)
        S_DUMMY: begin
          qspi_sck <= ~qspi_sck;
          if (qspi_sck) begin
            if (bit_cnt == 4'd0) begin
              bit_cnt   <= 4'd7; // 8 nibbles = 32 bits of data
              shift_out <= mem_write_data;
              shift_in  <= 32'd0;
              state     <= S_DATA;
              qspi_io_oe <= serving_write;
            end else begin
              bit_cnt <= bit_cnt - 4'd1;
            end
          end
        end

        // -----------------------------------------------------------------
        // Data phase: read or write 32 bits in Quad mode
        S_DATA: begin
          qspi_sck <= ~qspi_sck;
          if (serving_write) begin
            // Write: drive data on rising edge
            if (qspi_sck) begin
              qspi_io_out <= shift_out[31:28];
              shift_out   <= {shift_out[27:0], 4'h0};
              if (bit_cnt == 4'd0)
                state <= S_DONE;
              else
                bit_cnt <= bit_cnt - 4'd1;
            end
          end else begin
            // Read: sample data on falling edge
            if (!qspi_sck) begin
              shift_in <= {shift_in[27:0], qspi_io_in};
              if (bit_cnt == 4'd0)
                state <= S_DONE;
              else
                bit_cnt <= bit_cnt - 4'd1;
            end
          end
        end

        // -----------------------------------------------------------------
        // Complete: deassert CS, send response
        S_DONE: begin
          qspi_cs_n     <= 1'b1;
          qspi_cs_ram_n <= 1'b1;
          qspi_sck      <= 1'b0;
          qspi_io_oe    <= 1'b0;
          sck_en        <= 1'b0;

          if (serving_mem) begin
            if (serving_write)
              mem_write_response <= 1'b1;
            else begin
              mem_read_data      <= shift_in;
              mem_read_response  <= 1'b1;
            end
          end else begin
            if_read_data <= shift_in;
            if_response  <= 1'b1;
          end

          state <= S_IDLE;
        end

        default: state <= S_IDLE;
      endcase
    end
  end

  // Avoid warnings about unused signals
  wire unused_ok = &{1'b0, ena_unused, 1'b0};
  wire ena_unused = &{1'b0, mem_write_strobe, if_address[31:24], mem_address[31:24], 1'b0};

endmodule
