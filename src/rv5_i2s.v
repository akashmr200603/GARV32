// SPDX-License-Identifier: MIT

module rv5_i2s (
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

  output wire                 i2s_bclk,
  output wire                 i2s_lrclk,
  output reg                  i2s_sdata_out,
  input  wire                 i2s_sdata_in
);

  reg [31:0] control;
  reg [31:0] clk_div;
  
  // TX Data
  reg [15:0] tx_left_data;
  reg [15:0] tx_right_data;
  reg [15:0] tx_left_shift;
  reg [15:0] tx_right_shift;

  // RX Data
  reg [15:0] rx_left_data;
  reg [15:0] rx_right_data;
  reg [15:0] rx_left_shift;
  reg [15:0] rx_right_shift;
  reg        rx_data_ready;

  reg [31:0] bclk_counter;
  reg        bclk_reg;
  reg [5:0]  bit_counter; // 0 to 31
  reg        lrclk_reg;
  
  assign i2s_bclk = bclk_reg;
  assign i2s_lrclk = lrclk_reg;

  wire valid_write = write_request && &write_strobe;

  always @(posedge clock) begin
    if (reset) begin
      bclk_counter <= 0;
      bclk_reg <= 0;
      bit_counter <= 0;
      lrclk_reg <= 0; 
      i2s_sdata_out <= 0;
      tx_left_data <= 0;
      tx_right_data <= 0;
      tx_left_shift <= 0;
      tx_right_shift <= 0;
      rx_left_data <= 0;
      rx_right_data <= 0;
      rx_left_shift <= 0;
      rx_right_shift <= 0;
      rx_data_ready <= 0;
      control <= 0;
      clk_div <= 0;
      read_response <= 0;
      write_response <= 0;
      read_data <= 0;
    end else begin
      // 1. Bus Responses
      read_response <= read_request;
      write_response <= write_request;
      
      // 2. Bus Write
      if (valid_write) begin
        case (rw_address)
          8'h00: control[0] <= write_data[0];
          8'h04: clk_div <= write_data;
          8'h08: tx_left_data <= write_data[15:0];
          8'h0C: tx_right_data <= write_data[15:0];
        endcase
      end
      
      // 3. Bus Read
      if (read_request) begin
        case (rw_address)
          8'h00: read_data <= {30'b0, rx_data_ready, control[0]};
          8'h04: read_data <= clk_div;
          8'h08: read_data <= {16'b0, tx_left_data};
          8'h0C: read_data <= {16'b0, tx_right_data};
          8'h10: read_data <= {16'b0, rx_left_data};
          8'h14: read_data <= {16'b0, rx_right_data};
          default: read_data <= 0;
        endcase
      end
      
      // 4. I2S State Machine & RX Data Ready flag
      if (control[0]) begin
        if (bclk_counter >= clk_div) begin
          bclk_counter <= 0;
          bclk_reg <= ~bclk_reg;
          
          if (bclk_reg == 1) begin // Falling edge of BCLK: output TX data, update LRCLK
            if (bit_counter == 31) begin
               lrclk_reg <= 1; // Transition to Right Channel
            end else if (bit_counter == 63) begin
               lrclk_reg <= 0; // Transition to Left Channel
            end
            
            if (bit_counter == 63) begin
              bit_counter <= 0;
              // Load new samples for TX
              tx_left_shift <= tx_left_data;
              tx_right_shift <= tx_right_data;
            end else begin
              bit_counter <= bit_counter + 1;
            end
            
            // Data is shifted out MSB first
            if (bit_counter < 16) begin
               i2s_sdata_out <= tx_left_shift[15];
               tx_left_shift <= {tx_left_shift[14:0], 1'b0};
            end else if (bit_counter >= 32 && bit_counter < 48) begin
               i2s_sdata_out <= tx_right_shift[15];
               tx_right_shift <= {tx_right_shift[14:0], 1'b0};
            end else begin
               i2s_sdata_out <= 0; // Pad with zeros for the rest of the 32-bit channel
            end
            
            // Handle rx_data_ready clear here so it doesn't conflict
            if (read_request && (rw_address == 8'h10 || rw_address == 8'h14)) begin
              rx_data_ready <= 0;
            end
            
          end else begin // Rising edge of BCLK: sample RX data
            if (bit_counter < 16) begin
               rx_left_shift <= {rx_left_shift[14:0], i2s_sdata_in};
            end else if (bit_counter >= 32 && bit_counter < 48) begin
               rx_right_shift <= {rx_right_shift[14:0], i2s_sdata_in};
            end
            
            if (bit_counter == 63) begin
               rx_left_data <= rx_left_shift;
               rx_right_data <= rx_right_shift;
               rx_data_ready <= 1; // Set data ready
            end else if (read_request && (rw_address == 8'h10 || rw_address == 8'h14)) begin
               rx_data_ready <= 0; // Clear on read
            end
          end
          
        end else begin
          bclk_counter <= bclk_counter + 1;
          // Clear rx_data_ready when read
          if (read_request && (rw_address == 8'h10 || rw_address == 8'h14)) begin
            rx_data_ready <= 0;
          end
        end
      end else begin
        bclk_counter <= 0;
        bclk_reg <= 0;
        bit_counter <= 0;
        lrclk_reg <= 0;
        i2s_sdata_out <= 0;
        if (read_request && (rw_address == 8'h10 || rw_address == 8'h14)) begin
          rx_data_ready <= 0;
        end
      end
    end
  end

endmodule
