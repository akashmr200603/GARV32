// SPDX-License-Identifier: MIT
module rv5_por (
  input  wire clock,
  input  wire ext_reset_n, // External active-low reset
  output wire sys_reset    // Internal active-high synchronized reset
);

  // 4-stage synchronizer for robust Power-On-Reset
  reg [3:0] reset_sync;

  always @(posedge clock or negedge ext_reset_n) begin
    if (!ext_reset_n) begin
      reset_sync <= 4'b1111;
    end else begin
      reset_sync <= {reset_sync[2:0], 1'b0};
    end
  end

  // Internal reset is active high, de-asserts synchronously with clock edge
  assign sys_reset = reset_sync[3];

endmodule
