// SPDX-License-Identifier: MIT
// Copyright (c) 2020-2025 RV5 Project Contributors

module rv5_bus #(
  parameter NUM_DEVICES               = 15

  )(

  // Global signals

  input   wire                        clock                 ,
  input   wire                        reset                 ,

  // Interface with the manager device (Processor Core IP)

  input   wire  [31:0]                manager_rw_address    ,
  output  wire  [31:0]                manager_read_data     ,
  input   wire                        manager_read_request  ,
  output  wire                        manager_read_response ,
  input   wire  [31:0]                manager_write_data    ,
  input   wire  [3:0 ]                manager_write_strobe  ,
  input   wire                        manager_write_request ,
  output  wire                        manager_write_response,

  // Interface with the managed devices

  output  wire  [31:0]                device_rw_address     ,
  input   wire  [NUM_DEVICES*32-1:0]  device_read_data      ,
  output  wire  [NUM_DEVICES-1:0]     device_read_request   ,
  input   wire  [NUM_DEVICES-1:0]     device_read_response  ,
  output  wire  [31:0]                device_write_data     ,
  output  wire  [3:0 ]                device_write_strobe   ,
  output  wire  [NUM_DEVICES-1:0]     device_write_request  ,
  input   wire  [NUM_DEVICES-1:0]     device_write_response ,

  // Base addresses and masks of the managed devices

  input   wire  [NUM_DEVICES*32-1:0]  device_start_address  ,
  input   wire  [NUM_DEVICES*32-1:0]  device_mask_address

  );

  integer i;
  
  wire _unused = &{1'b0, device_start_address, device_mask_address, 1'b0};

  reg [NUM_DEVICES-1:0]         device_sel;
  reg [NUM_DEVICES-1:0]         device_sel_save;

  // Manager request

  assign device_rw_address      = manager_rw_address;
  assign device_read_request    = device_sel & {NUM_DEVICES{manager_read_request}};
  assign device_write_data      = manager_write_data;
  assign device_write_strobe    = manager_write_strobe;
  assign device_write_request   = device_sel & {NUM_DEVICES{manager_write_request}};

  // Optimized address decode: 2-level hierarchical selection
  // Level 1: bit[31] distinguishes RAM (0) from peripherals (1)
  // Level 2: bits[19:16] select the specific peripheral (0x0-0x9)
  //
  // This replaces the generic (addr & mask) == start comparison
  // which synthesizes into deep CARRY4 chains, with simple LUT decoding.
  
  wire is_peripheral = manager_rw_address[31];
  wire is_ram = (manager_rw_address[31:28] == 4'h2);
  wire is_qram = (manager_rw_address[31:28] == 4'h1);
  wire [3:0] periph_id = manager_rw_address[19:16];

  always @(*) begin
    device_sel = {NUM_DEVICES{1'b0}};
    if (!is_peripheral) begin
      if (is_ram)
        device_sel[1] = 1'b1; // RAM
      else if (is_qram)
        device_sel[10] = 1'b1; // QSPI RAM
      else
        device_sel[0] = 1'b1; // ROM
    end else begin
      case (periph_id)
        4'h0: device_sel[2]  = 1'b1; // UART0   @ 0x8000_0000
        4'h1: device_sel[3]  = 1'b1; // MTIMER  @ 0x8001_0000
        4'h2: device_sel[4]  = 1'b1; // GPIO    @ 0x8002_0000
        4'h3: device_sel[5]  = 1'b1; // SPI0    @ 0x8003_0000
        4'h4: device_sel[6]  = 1'b1; // UART1   @ 0x8004_0000
        4'h6: device_sel[7]  = 1'b1; // I2C0    @ 0x8006_0000
        4'h8: device_sel[8]  = 1'b1; // PWM     @ 0x8008_0000
        4'hA: device_sel[10] = 1'b1; // HK      @ 0x800A_0000
        default: ; // no device
      endcase
    end
  end

  // Save which device was selected (registered)

  always @(posedge clock) begin
    if (reset)
      device_sel_save <= {NUM_DEVICES{1'b0}};
    else if ((manager_read_request || manager_write_request) && (|device_sel))
      device_sel_save <= device_sel;
    else
      device_sel_save <= {NUM_DEVICES{1'b0}};
  end

  // One-hot OR-reduction mux for device responses (combinational).

  reg [31:0] read_data_mux;
  reg        read_response_mux;
  reg        write_response_mux;

  always @(*) begin
    read_data_mux           = 32'b0;
    read_response_mux       = 1'b1;
    write_response_mux      = 1'b1;
    for (i = 0; i < NUM_DEVICES; i = i + 1) begin
      read_data_mux       = read_data_mux | ({32{device_sel_save[i]}} & device_read_data[i*32 +: 32]);
      read_response_mux   = read_response_mux & (device_read_response[i] | ~device_sel_save[i]);
      write_response_mux  = write_response_mux & (device_write_response[i] | ~device_sel_save[i]);
    end
  end

  // Removed pipeline register on the response path.
  // The extra cycle of latency caused the CPU to receive duplicated responses
  // and double-execute instructions because the CPU's memory interface 
  // expects exactly 1 cycle latency and does not track inflight requests.
  
  assign manager_read_data      = read_data_mux;
  assign manager_read_response  = read_response_mux;
  assign manager_write_response = write_response_mux;

endmodule
