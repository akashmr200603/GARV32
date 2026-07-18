// SPDX-License-Identifier: MIT
module rv5_icache #(
  parameter CACHE_LINES = 4 // 4 words * 4 bytes = 16 bytes I-Cache
)(
  input  wire        clock,
  input  wire        reset,
  
  // Core IF Interface
  input  wire [31:0] core_if_address,
  output reg  [31:0] core_if_read_data,
  input  wire        core_if_request,
  output reg         core_if_response,
  
  // Downstream Memory IF Interface
  output reg  [31:0] mem_if_address,
  input  wire [31:0] mem_if_read_data,
  output reg         mem_if_request,
  input  wire        mem_if_response
);

  localparam INDEX_BITS = $clog2(CACHE_LINES);
  localparam TAG_BITS   = 32 - 2 - INDEX_BITS;
  
  reg [31:0] cache_data [0:CACHE_LINES-1];
  reg [TAG_BITS-1:0] cache_tag [0:CACHE_LINES-1];
  reg valid_bit [0:CACHE_LINES-1];
  
  wire [INDEX_BITS-1:0] index = core_if_address[INDEX_BITS+1:2];
  wire [TAG_BITS-1:0]   tag   = core_if_address[31:INDEX_BITS+2];
  
  wire cache_hit = valid_bit[index] && (cache_tag[index] == tag);
  
  localparam STATE_IDLE = 1'b0;
  localparam STATE_MISS = 1'b1;
  reg state;
  
  integer i;
  
  always @(posedge clock) begin
    if (reset) begin
      state <= STATE_IDLE;
      core_if_response <= 1'b0;
      mem_if_request <= 1'b0;
      for (i = 0; i < CACHE_LINES; i = i + 1) begin
        valid_bit[i] <= 1'b0;
      end
    end else begin
      core_if_response <= 1'b0;
      
      case (state)
        STATE_IDLE: begin
          if (core_if_request) begin
            if (cache_hit) begin
              core_if_read_data <= cache_data[index];
              core_if_response  <= 1'b1;
            end else begin
              mem_if_address <= core_if_address;
              mem_if_request <= 1'b1;
              state          <= STATE_MISS;
            end
          end
        end
        
        STATE_MISS: begin
          if (mem_if_response) begin
            // Update cache
            cache_data[index] <= mem_if_read_data;
            cache_tag[index]  <= tag;
            valid_bit[index]  <= 1'b1;
            
            // Forward to core
            core_if_read_data <= mem_if_read_data;
            core_if_response  <= 1'b1;
            
            mem_if_request <= 1'b0;
            state          <= STATE_IDLE;
          end
        end
      endcase
    end
  end

endmodule
