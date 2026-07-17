// SPDX-License-Identifier: MIT

module rv5_pwm #(
  parameter CHANNELS = 8
) (
  input  wire                 clock,
  input  wire                 reset,

  input  wire [7:0]           rw_address,
  output reg  [31:0]          read_data,
  input  wire                 read_request,
  output reg                  read_response,
  input  wire [31:0]          write_data,
  input  wire [3:0]           write_strobe,
  input  wire                 write_request,
  output reg                  write_response,

  output wire [CHANNELS-1:0]  pwm_out
);

  // Address map:
  // 0x00: Prescaler
  // 0x04: Period
  // 0x10 to 0x2C: Duty cycles for channels 0 to 7

  reg [31:0] prescaler;
  reg [31:0] period;
  reg [31:0] duty [0:CHANNELS-1];
  
  reg [31:0] prescaler_count;
  reg [31:0] period_count;

  wire valid_write = write_request && &write_strobe;

  genvar ch;
  generate
    for (ch = 0; ch < CHANNELS; ch = ch + 1) begin : pwm_gen
      assign pwm_out[ch] = (period_count < duty[ch]) ? 1'b1 : 1'b0;
    end
  endgenerate

  // Single always block for all registers to avoid multi-driven nets
  integer i;
  always @(posedge clock) begin
    if (reset) begin
      prescaler <= 0;
      period <= 32'hFFFF;
      prescaler_count <= 0;
      period_count <= 0;
      read_response <= 0;
      write_response <= 0;
      read_data <= 0;
      for (i = 0; i < CHANNELS; i = i + 1) begin
        duty[i] <= 0;
      end
    end else begin
      // PWM counter logic
      if (prescaler_count >= prescaler) begin
        prescaler_count <= 0;
        if (period_count >= period) begin
          period_count <= 0;
        end else begin
          period_count <= period_count + 1;
        end
      end else begin
        prescaler_count <= prescaler_count + 1;
      end

      // Bus interface
      read_response <= read_request;
      write_response <= write_request;
      
      if (valid_write) begin
        case (rw_address)
          8'h00: prescaler <= write_data;
          8'h04: period <= write_data;
          8'h10: duty[0] <= write_data;
          8'h14: duty[1] <= write_data;
          8'h18: duty[2] <= write_data;
          8'h1C: duty[3] <= write_data;
          8'h20: duty[4] <= write_data;
          8'h24: duty[5] <= write_data;
          8'h28: duty[6] <= write_data;
          8'h2C: duty[7] <= write_data;
        endcase
      end
      
      if (read_request) begin
        case (rw_address)
          8'h00: read_data <= prescaler;
          8'h04: read_data <= period;
          8'h10: read_data <= duty[0];
          8'h14: read_data <= duty[1];
          8'h18: read_data <= duty[2];
          8'h1C: read_data <= duty[3];
          8'h20: read_data <= duty[4];
          8'h24: read_data <= duty[5];
          8'h28: read_data <= duty[6];
          8'h2C: read_data <= duty[7];
          default: read_data <= 0;
        endcase
      end
    end
  end

endmodule
