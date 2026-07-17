// SPDX-License-Identifier: MIT
// Copyright (c) 2020-2025 RV5 Project Contributors
// 64B Register-based SRAM for Tiny Tapeout (no OpenRAM macro needed)

module rv5_ram #(
  parameter MEMORY_SIZE = 64 // 64B default
) (
  input   wire          clock,
  input   wire          reset,

  // Port A: Instruction Fetch (Read-Only)
  input  wire   [31:0]  if_address,
  output reg    [31:0]  if_read_data,
  input  wire           if_request,
  output reg            if_response,

  // Port B: Data Memory (Read/Write)
  input  wire   [31:0]  mem_address,
  output reg    [31:0]  mem_read_data,
  input  wire           mem_read_request,
  output reg            mem_read_response,
  input  wire   [31:0]  mem_write_data,
  input  wire   [3:0 ]  mem_write_strobe,
  input  wire           mem_write_request,
  output reg            mem_write_response
);

  localparam NUM_WORDS  = MEMORY_SIZE / 4;
  localparam ADDR_WIDTH = $clog2(NUM_WORDS);

  // Synthesizable register-based memory array.
  // Yosys will map this to standard-cell Flip-Flops.
  // 64B = 16 words x 32 bits = 512 Flip-Flops.
  reg [31:0] memory [0:NUM_WORDS-1];

  wire [ADDR_WIDTH-1:0] if_word_addr  = if_address[ADDR_WIDTH+1:2];
  wire [ADDR_WIDTH-1:0] mem_word_addr = mem_address[ADDR_WIDTH+1:2];

  always @(posedge clock) begin
    if (reset) begin
      if_response        <= 1'b0;
      mem_read_response  <= 1'b0;
      mem_write_response <= 1'b0;
    end else begin
      // Port A: Instruction Fetch (true dual-port, no arbitration needed)
      if_response <= if_request;
      if (if_request)
        if_read_data <= memory[if_word_addr];

      // Port B: Data Memory
      mem_read_response  <= mem_read_request;
      mem_write_response <= mem_write_request;

      if (mem_read_request)
        mem_read_data <= memory[mem_word_addr];

      if (mem_write_request) begin
        if (mem_write_strobe[0]) memory[mem_word_addr][ 7: 0] <= mem_write_data[ 7: 0];
        if (mem_write_strobe[1]) memory[mem_word_addr][15: 8] <= mem_write_data[15: 8];
        if (mem_write_strobe[2]) memory[mem_word_addr][23:16] <= mem_write_data[23:16];
        if (mem_write_strobe[3]) memory[mem_word_addr][31:24] <= mem_write_data[31:24];
      end
    end
  end

  // Avoid warnings about intentionally unused address bits
  wire unused_ok =
    &{1'b0,
    if_address[31:ADDR_WIDTH+2],
    if_address[1:0],
    mem_address[31:ADDR_WIDTH+2],
    mem_address[1:0],
    1'b0};

endmodule
