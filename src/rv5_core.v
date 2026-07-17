// SPDX-License-Identifier: MIT
// Copyright (c) 2020-2025 RV5 Project Contributors

module rv5_core #(
  parameter     [31:0]  BOOT_ADDRESS = 32'h00000000
) (
  // Global signals
  input  wire           clock,
  input  wire           reset,
  input  wire           halt,

  // IF interface (Instruction Fetch)
  output wire   [31:0]  if_address,
  input  wire   [31:0]  if_read_data,
  output wire           if_request,
  input  wire           if_response,

  // MEM interface (Data Memory)
  output wire   [31:0]  mem_address,
  input  wire   [31:0]  mem_read_data,
  output wire           mem_read_request,
  input  wire           mem_read_response,
  output wire   [31:0]  mem_write_data,
  output wire   [3:0 ]  mem_write_strobe,
  output wire           mem_write_request,
  input  wire           mem_write_response,

  // Interrupt signals
  input  wire           irq_external,
  output wire           irq_external_response,
  input  wire           irq_timer,
  output wire           irq_timer_response,
  input  wire           irq_software,
  output wire           irq_software_response,
  input  wire   [15:0]  irq_fast,
  output wire   [15:0]  irq_fast_response,

  // Real Time Clock
  input  wire   [63:0]  real_time_clock
);

  // =========================================================================
  //  5-STAGE PIPELINED RV32I CORE - LATENCY-AWARE IMPLEMENTATION
  // =========================================================================
  //
  //  RAM Port A (IF):  1-cycle read latency (registered output)
  //  RAM Port B (MEM): 1-cycle read latency (registered output)
  //  Bus peripherals:  1-cycle response latency (registered response)
  //
  //  Pipeline stages:
  //    IF0  - Present address to RAM
  //    IF1  - RAM data arrives; latch into IF/ID register
  //    ID   - Decode + register read
  //    EX   - ALU + branch resolution
  //    MEM  - Data memory access (request issued)
  //    WB   - Data memory response arrives; write-back to register file
  //
  //  Because RAM has 1-cycle latency, the "IF" is effectively split into
  //  two sub-stages: IF0 (address out) and IF1 (data back). The PC that
  //  corresponds to the data arriving on if_read_data is the PC from the
  //  PREVIOUS cycle. We track this with `if1_pc`.
  // =========================================================================

  reg reset_reg;
  always @(posedge clock) reset_reg <= reset;
  wire reset_internal = reset | reset_reg;

  wire clock_enable = !halt;

  // Forward declarations
  wire        stall_pipeline;
  wire        flush_pipeline;
  wire        ex_branch_taken;
  wire [31:0] ex_branch_target;
  wire        wb_trap_taken;
  wire [31:0] wb_trap_target;
  wire        wb_mret;
  wire [31:0] wb_mepc;
  wire        mem_stall;       // stall while waiting for memory response

  wire [4:0]  id_ex_rd;
  wire        id_ex_valid;
  wire        id_ex_mem_read;
  wire        id_ex_reg_write;

  wire [4:0]  ex_mem_rd;
  wire        ex_mem_valid;
  wire        ex_mem_mem_read;
  wire        ex_mem_reg_write;
  wire [31:0] ex_mem_alu_result;

  assign flush_pipeline = ex_branch_taken || wb_trap_taken || wb_mret;

  // =========================================================================
  //  IF0 STAGE - Drive address to RAM
  // =========================================================================
  reg [31:0] pc_reg;

  wire [31:0] next_pc = wb_trap_taken   ? wb_trap_target :
                        wb_mret         ? wb_mepc :
                        ex_branch_taken ? ex_branch_target :
                        (pc_reg + 4);

  always @(posedge clock) begin
    if (reset_internal)
      pc_reg <= BOOT_ADDRESS;
    else if (clock_enable) begin
      if (flush_pipeline)
        pc_reg <= next_pc;
      else if (!stall_pipeline && !mem_stall)
        pc_reg <= next_pc;
    end
  end

  assign if_address = pc_reg;
  assign if_request = clock_enable && !reset_internal;

  // =========================================================================
  //  IF1 STAGE - RAM data arrives this cycle for address presented last cycle
  //  We need to track which PC corresponds to the arriving instruction.
  // =========================================================================
  reg [31:0] if1_pc;
  reg        if1_valid;

  always @(posedge clock) begin
    if (reset_internal) begin
      if1_valid <= 1'b0;
    end else if (clock_enable) begin
      if (flush_pipeline) begin
        if1_valid <= 1'b0;
      end else if (!stall_pipeline && !mem_stall) begin
        if1_pc    <= pc_reg;
        if1_valid <= 1'b1;
      end
    end
  end

  // When the pipeline stalls, the instruction that was arriving from RAM
  // during that exact cycle will be lost because the pipeline registers 
  // are frozen, and the RAM will output the *next* instruction on the next cycle.
  // We must hold it in a temporary register.
  reg [31:0] if_read_data_r;
  reg        use_held_data;

  always @(posedge clock) begin
    if (reset_internal) begin
      use_held_data <= 1'b0;
    end else if (clock_enable) begin
      if (stall_pipeline || mem_stall) begin
        if (!use_held_data) begin
          if_read_data_r <= if_read_data;
          use_held_data  <= 1'b1;
        end
      end else begin
        use_held_data <= 1'b0;
      end
    end
  end

  wire [31:0] current_if_inst = use_held_data ? if_read_data_r : if_read_data;

  // =========================================================================
  //  IF/ID REGISTER - Latch instruction from RAM
  // =========================================================================
  reg [31:0] if_id_pc;
  reg [31:0] if_id_inst;
  reg        if_id_valid;

  always @(posedge clock) begin
    if (reset_internal) begin
      if_id_valid <= 1'b0;
    end else if (clock_enable) begin
      if (flush_pipeline) begin
        if_id_valid <= 1'b0;
      end else if (!mem_stall && !stall_pipeline) begin
        if_id_valid <= if1_valid;
      end
      
      // Decouple datapath from flush_pipeline to improve timing
      if (!mem_stall && !stall_pipeline) begin
        if_id_pc    <= if1_pc;
        if_id_inst  <= current_if_inst;
      end
    end
  end

  // =========================================================================
  //  ID STAGE - Decode + Register Read
  // =========================================================================
  wire [6:0] id_opcode = if_id_inst[6:0];
  wire [2:0] id_funct3 = if_id_inst[14:12];
  wire [6:0] id_funct7 = if_id_inst[31:25];
  wire [4:0] id_rs1    = if_id_inst[19:15];
  wire [4:0] id_rs2    = if_id_inst[24:20];
  wire [4:0] id_rd     = if_id_inst[11:7];

  // Register file
  reg [31:0] reg_file [0:31];
  wire        wb_reg_write;
  wire [4:0]  wb_rd;
  wire [31:0] wb_write_data;

  // Read with EX/MEM and WB bypass in ID stage
  // EX/MEM forwarding here handles non-load results (load-use is stalled)
  // WB forwarding handles results that just finished writeback
  wire [31:0] id_rs1_data = (id_rs1 == 5'b0) ? 32'b0 :
                            (ex_mem_reg_write && ex_mem_rd != 0 && ex_mem_rd == id_rs1) ? ex_mem_alu_result :
                            (wb_reg_write && wb_rd != 0 && wb_rd == id_rs1) ? wb_write_data :
                            reg_file[id_rs1];
  wire [31:0] id_rs2_data = (id_rs2 == 5'b0) ? 32'b0 :
                            (ex_mem_reg_write && ex_mem_rd != 0 && ex_mem_rd == id_rs2) ? ex_mem_alu_result :
                            (wb_reg_write && wb_rd != 0 && wb_rd == id_rs2) ? wb_write_data :
                            reg_file[id_rs2];

  integer i;
  always @(posedge clock) begin
    if (reset_internal) begin
      for (i = 0; i < 32; i = i + 1) reg_file[i] <= 32'b0;
    end else if (clock_enable && wb_reg_write && wb_rd != 5'b0) begin
      reg_file[wb_rd] <= wb_write_data;
    end
  end

  // Instruction type decode
  wire id_is_branch  = (id_opcode == 7'b1100011);
  wire id_is_jal     = (id_opcode == 7'b1101111);
  wire id_is_jalr    = (id_opcode == 7'b1100111);
  wire id_is_load    = (id_opcode == 7'b0000011);
  wire id_is_store   = (id_opcode == 7'b0100011);
  wire id_is_alu_imm = (id_opcode == 7'b0010011);
  wire id_is_alu_reg = (id_opcode == 7'b0110011);
  wire id_is_lui     = (id_opcode == 7'b0110111);
  wire id_is_auipc   = (id_opcode == 7'b0010111);
  wire id_is_system  = (id_opcode == 7'b1110011);
  wire id_is_fence   = (id_opcode == 7'b0001111);

  // Immediate generation
  wire [31:0] id_imm_i = {{20{if_id_inst[31]}}, if_id_inst[31:20]};
  wire [31:0] id_imm_s = {{20{if_id_inst[31]}}, if_id_inst[31:25], if_id_inst[11:7]};
  wire [31:0] id_imm_b = {{20{if_id_inst[31]}}, if_id_inst[7], if_id_inst[30:25], if_id_inst[11:8], 1'b0};
  wire [31:0] id_imm_u = {if_id_inst[31:12], 12'b0};
  wire [31:0] id_imm_j = {{12{if_id_inst[31]}}, if_id_inst[19:12], if_id_inst[20], if_id_inst[30:21], 1'b0};

  reg [31:0] id_imm;
  always @(*) begin
    case (1'b1)
      id_is_store:                          id_imm = id_imm_s;
      id_is_branch:                         id_imm = id_imm_b;
      (id_is_lui | id_is_auipc):            id_imm = id_imm_u;
      id_is_jal:                            id_imm = id_imm_j;
      default:                              id_imm = id_imm_i;
    endcase
  end

  // Pre-compute in ID to reduce EX critical path
  wire [31:0] id_pc_plus_imm = if_id_pc + id_imm;
  wire [31:0] id_pc_plus_4   = if_id_pc + 32'd4;

  wire id_reg_write = if_id_valid && (id_is_alu_imm | id_is_alu_reg | id_is_load |
                      id_is_lui | id_is_auipc | id_is_jal | id_is_jalr | id_is_system);

  // =========================================================================
  //  HAZARD DETECTION - Stall on data dependencies
  // =========================================================================
  //
  //  We need to stall if the instruction in ID reads a register that:
  //   - Is being written by a LOAD in EX (classic load-use, 1 stall cycle)
  //   - Is being written by a LOAD in MEM (data not yet available, 1 stall)
  //
  //  For non-load instructions in EX, we can forward from EX→EX.
  //  For non-load instructions in MEM, we can forward via WB bypass.
  // =========================================================================

  wire uses_rs1 = if_id_valid && (id_is_branch | id_is_jalr | id_is_load | id_is_store |
                                  id_is_alu_imm | id_is_alu_reg | id_is_system);
  wire uses_rs2 = if_id_valid && (id_is_branch | id_is_store | id_is_alu_reg);

  wire ex_load_hazard  = id_ex_valid && id_ex_mem_read && id_ex_rd != 0 &&
                         ((uses_rs1 && id_ex_rd == id_rs1) ||
                          (uses_rs2 && id_ex_rd == id_rs2));

  wire mem_load_hazard = ex_mem_valid && ex_mem_mem_read && ex_mem_rd != 0 &&
                         ((uses_rs1 && ex_mem_rd == id_rs1) ||
                          (uses_rs2 && ex_mem_rd == id_rs2));

  assign stall_pipeline = (ex_load_hazard || mem_load_hazard) && !flush_pipeline;

  wire fwd_ex_rs1 = id_ex_valid && id_ex_reg_write && id_ex_rd != 5'b0 && (id_ex_rd == id_rs1);
  wire fwd_ex_rs2 = id_ex_valid && id_ex_reg_write && id_ex_rd != 5'b0 && (id_ex_rd == id_rs2);

  // =========================================================================
  //  ID/EX REGISTER
  // =========================================================================
  reg [31:0] id_ex_pc_r;
  reg [31:0] id_ex_inst_r;
  reg [31:0] id_ex_rs1_data;
  reg [31:0] id_ex_rs2_data;
  reg [31:0] id_ex_imm_r;
  reg [31:0] id_ex_pc_plus_imm;
  reg [31:0] id_ex_pc_plus_4;
  reg [4:0]  id_ex_rs1_r;
  reg [4:0]  id_ex_rs2_r;
  reg [4:0]  id_ex_rd_r;
  reg        id_ex_reg_write_r;
  reg        id_ex_mem_read_r;
  reg        id_ex_mem_write;
  reg        id_ex_is_branch;
  reg        id_ex_is_jal;
  reg        id_ex_is_jalr;
  reg        id_ex_is_lui;
  reg        id_ex_is_auipc;
  reg        id_ex_is_system;
  reg        id_ex_valid_r;
  reg        id_ex_fwd_rs1;
  reg        id_ex_fwd_rs2;

  assign id_ex_rd        = id_ex_rd_r;
  assign id_ex_valid     = id_ex_valid_r;
  assign id_ex_mem_read  = id_ex_mem_read_r;
  assign id_ex_reg_write = id_ex_reg_write_r;

  wire id_ex_flush = stall_pipeline || flush_pipeline;

  always @(posedge clock) begin
    if (reset_internal) begin
      id_ex_valid_r <= 1'b0;
    end else if (clock_enable) begin
      if (!mem_stall) begin
        if (id_ex_flush) begin
          id_ex_valid_r <= 1'b0;
        end else begin
          id_ex_valid_r <= if_id_valid;
        end
      end
      
      // Decouple datapath from id_ex_flush to improve timing
      if (!mem_stall) begin
        id_ex_pc_r         <= if_id_pc;
        id_ex_inst_r       <= if_id_inst;
        id_ex_rs1_data     <= id_rs1_data;
        id_ex_rs2_data     <= id_rs2_data;
        id_ex_imm_r        <= id_imm;
        id_ex_pc_plus_imm  <= id_pc_plus_imm;
        id_ex_pc_plus_4    <= id_pc_plus_4;
        id_ex_rs1_r        <= id_rs1;
        id_ex_rs2_r        <= id_rs2;
        id_ex_rd_r         <= id_rd;
        id_ex_reg_write_r  <= id_reg_write;
        id_ex_mem_read_r   <= id_is_load;
        id_ex_mem_write    <= id_is_store;
        id_ex_is_branch    <= id_is_branch;
        id_ex_is_jal       <= id_is_jal;
        id_ex_is_jalr      <= id_is_jalr;
        id_ex_is_lui       <= id_is_lui;
        id_ex_is_auipc     <= id_is_auipc;
        id_ex_is_system    <= id_is_system;
        id_ex_fwd_rs1      <= fwd_ex_rs1;
        id_ex_fwd_rs2      <= fwd_ex_rs2;
      end
    end
  end

  // =========================================================================
  //  EX STAGE - ALU + Branch Resolution
  // =========================================================================

  // Forwarding from EX/MEM stage only (short registered path)
  // WB forwarding is handled in ID stage before the pipeline register,
  // so it does NOT appear on the EX critical path.

  reg [31:0] ex_rs1_fwd;
  reg [31:0] ex_rs2_fwd;

  always @(*) begin
    if (id_ex_fwd_rs1)
      ex_rs1_fwd = ex_mem_alu_result;
    else
      ex_rs1_fwd = id_ex_rs1_data;
  end

  always @(*) begin
    if (id_ex_fwd_rs2)
      ex_rs2_fwd = ex_mem_alu_result;
    else
      ex_rs2_fwd = id_ex_rs2_data;
  end

  // ALU
  wire [31:0] alu_in1 = ex_rs1_fwd;
  wire [31:0] alu_in2 = (id_ex_inst_r[6:0] == 7'b0110011) ? ex_rs2_fwd : id_ex_imm_r;

  wire [2:0] ex_funct3 = id_ex_inst_r[14:12];
  wire [6:0] ex_funct7 = id_ex_inst_r[31:25];
  wire is_sub = (id_ex_inst_r[6:0] == 7'b0110011) && ex_funct7[5];
  wire is_sra = ex_funct7[5];

  wire [2:0] alu_op = (id_ex_mem_read_r || id_ex_mem_write) ? 3'b000 : ex_funct3;

  reg [31:0] alu_result;
  always @(*) begin
    case (alu_op)
      3'b000:  alu_result = is_sub ? (alu_in1 - alu_in2) : (alu_in1 + alu_in2);
      3'b001:  alu_result = alu_in1 << alu_in2[4:0];
      3'b010:  alu_result = {31'b0, $signed(alu_in1) < $signed(alu_in2)};
      3'b011:  alu_result = {31'b0, alu_in1 < alu_in2};
      3'b100:  alu_result = alu_in1 ^ alu_in2;
      3'b101:  alu_result = is_sra ? ($signed(alu_in1) >>> alu_in2[4:0]) : (alu_in1 >> alu_in2[4:0]);
      3'b110:  alu_result = alu_in1 | alu_in2;
      3'b111:  alu_result = alu_in1 & alu_in2;
      default: alu_result = 32'b0;
    endcase
  end

  // Final EX result mux
  wire [31:0] ex_result = id_ex_is_lui   ? id_ex_imm_r :
                          id_ex_is_auipc ? id_ex_pc_plus_imm :
                          (id_ex_is_jal || id_ex_is_jalr) ? id_ex_pc_plus_4 :
                          alu_result;

  // Branch logic
  wire beq  = (ex_rs1_fwd == ex_rs2_fwd);
  wire blt  = ($signed(ex_rs1_fwd) < $signed(ex_rs2_fwd));
  wire bltu = (ex_rs1_fwd < ex_rs2_fwd);

  reg branch_cond;
  always @(*) begin
    case (ex_funct3[2:1])
      2'b00:   branch_cond = ex_funct3[0] ? !beq  : beq;
      2'b10:   branch_cond = ex_funct3[0] ? !blt  : blt;
      2'b11:   branch_cond = ex_funct3[0] ? !bltu : bltu;
      default: branch_cond = 1'b0;
    endcase
  end

  assign ex_branch_taken  = id_ex_valid_r && (id_ex_is_jal || id_ex_is_jalr ||
                            (id_ex_is_branch && branch_cond));
  assign ex_branch_target = id_ex_is_jalr ? ((ex_rs1_fwd + id_ex_imm_r) & ~32'd1) :
                            id_ex_pc_plus_imm;

  // =========================================================================
  //  EX/MEM REGISTER
  // =========================================================================
  reg [31:0] ex_mem_pc;
  reg [31:0] ex_mem_inst;
  reg [31:0] ex_mem_alu_result_r;
  reg [31:0] ex_mem_rs2_data;
  reg [4:0]  ex_mem_rd_r;
  reg        ex_mem_reg_write_r;
  reg        ex_mem_mem_read_r;
  reg        ex_mem_mem_write;
  reg        ex_mem_is_system;
  reg        ex_mem_valid_r;

  assign ex_mem_alu_result = ex_mem_alu_result_r;
  assign ex_mem_rd         = ex_mem_rd_r;
  assign ex_mem_reg_write  = ex_mem_reg_write_r && ex_mem_valid_r;
  assign ex_mem_valid      = ex_mem_valid_r;
  assign ex_mem_mem_read   = ex_mem_mem_read_r;

  wire ex_mem_flush = wb_trap_taken || wb_mret;

  always @(posedge clock) begin
    if (reset_internal) begin
      ex_mem_valid_r <= 1'b0;
    end else if (clock_enable) begin
      if (!mem_stall) begin
        if (ex_mem_flush) begin
          ex_mem_valid_r <= 1'b0;
        end else begin
          ex_mem_valid_r <= id_ex_valid_r;
        end
      end
      
      if (!mem_stall) begin
        ex_mem_pc           <= id_ex_pc_r;
        ex_mem_inst         <= id_ex_inst_r;
        ex_mem_alu_result_r <= ex_result;
        ex_mem_rs2_data     <= ex_rs2_fwd;
        ex_mem_rd_r         <= id_ex_rd_r;
        ex_mem_reg_write_r  <= id_ex_reg_write_r;
        ex_mem_mem_read_r   <= id_ex_mem_read_r;
        ex_mem_mem_write    <= id_ex_mem_write;
        ex_mem_is_system    <= id_ex_is_system;
      end
    end
  end

  // =========================================================================
  //  MEM STAGE - Data memory access
  // =========================================================================
  wire [1:0] mem_byte_offset = ex_mem_alu_result_r[1:0];
  wire [2:0] mem_funct3      = ex_mem_inst[14:12];

  // Write data alignment
  reg [3:0]  mem_strobe_r;
  reg [31:0] mem_wdata_r;

  always @(*) begin
    mem_strobe_r = 4'b0000;
    mem_wdata_r  = ex_mem_rs2_data;
    case (mem_funct3)
      3'b000: begin // SB
        mem_strobe_r = 4'b0001 << mem_byte_offset;
        mem_wdata_r  = {4{ex_mem_rs2_data[7:0]}};
      end
      3'b001: begin // SH
        mem_strobe_r = 4'b0011 << mem_byte_offset;
        mem_wdata_r  = {2{ex_mem_rs2_data[15:0]}};
      end
      3'b010: begin // SW
        mem_strobe_r = 4'b1111;
        mem_wdata_r  = ex_mem_rs2_data;
      end
      default: begin
        mem_strobe_r = 4'b0000;
        mem_wdata_r  = ex_mem_rs2_data;
      end
    endcase
  end

  assign mem_address       = {ex_mem_alu_result_r[31:2], 2'b00};
  assign mem_write_data    = mem_wdata_r;
  assign mem_write_strobe  = mem_strobe_r;
  assign mem_read_request  = ex_mem_valid_r && ex_mem_mem_read_r;
  assign mem_write_request = ex_mem_valid_r && ex_mem_mem_write;

  // Memory stall: only LOADS need to wait 1 cycle for read data.
  // Stores are fire-and-forget (data is presented combinationally, 
  // peripheral latches on clock edge - no wait needed).
  reg mem_load_pending;
  always @(posedge clock) begin
    if (reset_internal)
      mem_load_pending <= 1'b0;
    else if (clock_enable)
      mem_load_pending <= mem_read_request && !mem_load_pending;
  end

  assign mem_stall = mem_read_request && !mem_load_pending;

  // =========================================================================
  //  MEM/WB REGISTER
  // =========================================================================
  reg [31:0] mem_wb_pc;
  reg [31:0] mem_wb_inst;
  reg [31:0] mem_wb_alu_result;
  reg [31:0] mem_wb_read_data;
  reg [4:0]  mem_wb_rd;
  reg        mem_wb_reg_write;
  reg        mem_wb_mem_read;
  reg        mem_wb_is_system;
  reg        mem_wb_valid;

  wire mem_wb_flush = wb_trap_taken || wb_mret;

  always @(posedge clock) begin
    if (reset_internal) begin
      mem_wb_valid <= 1'b0;
    end else if (clock_enable) begin
      if (mem_wb_flush) begin
        mem_wb_valid <= 1'b0;
      end else if (mem_stall) begin
        // Hold - don't advance to WB yet
        mem_wb_valid <= 1'b0;
      end else begin
        mem_wb_valid <= ex_mem_valid_r;
      end
      
      if (!mem_stall) begin
        mem_wb_pc         <= ex_mem_pc;
        mem_wb_inst       <= ex_mem_inst;
        mem_wb_alu_result <= ex_mem_alu_result_r;
        mem_wb_read_data  <= mem_read_data;
        mem_wb_rd         <= ex_mem_rd_r;
        mem_wb_reg_write  <= ex_mem_reg_write_r;
        mem_wb_mem_read   <= ex_mem_mem_read_r;
        mem_wb_is_system  <= ex_mem_is_system;
      end
    end
  end

  // =========================================================================
  //  WB STAGE - Write-back + CSR
  // =========================================================================
  wire [1:0] wb_byte_offset = mem_wb_alu_result[1:0];
  wire [2:0] wb_funct3      = mem_wb_inst[14:12];

  // Load data alignment
  reg [31:0] wb_load_data;
  always @(*) begin
    case (wb_funct3)
      3'b000:  wb_load_data = {{24{mem_wb_read_data[wb_byte_offset*8 +  7]}},
                                   mem_wb_read_data[wb_byte_offset*8 +: 8]};  // LB
      3'b100:  wb_load_data = {24'b0, mem_wb_read_data[wb_byte_offset*8 +: 8]};  // LBU
      3'b001:  wb_load_data = {{16{mem_wb_read_data[wb_byte_offset*8 + 15]}},
                                   mem_wb_read_data[wb_byte_offset*8 +: 16]}; // LH
      3'b101:  wb_load_data = {16'b0, mem_wb_read_data[wb_byte_offset*8 +: 16]}; // LHU
      3'b010:  wb_load_data = mem_wb_read_data;                                   // LW
      default: wb_load_data = mem_wb_read_data;
    endcase
  end

  // CSR logic
  wire wb_is_csr   = mem_wb_is_system && (wb_funct3 != 3'b0);
  wire wb_is_ecall = mem_wb_is_system && (wb_funct3 == 3'b0) && (mem_wb_inst[31:20] == 12'h000);
  wire wb_is_ebreak= mem_wb_is_system && (wb_funct3 == 3'b0) && (mem_wb_inst[31:20] == 12'h001);
  assign wb_mret   = mem_wb_valid && mem_wb_is_system && (wb_funct3 == 3'b0) && (mem_wb_inst[31:20] == 12'h302);

  wire [11:0] wb_csr_addr  = mem_wb_inst[31:20];
  wire [31:0] wb_csr_wdata = wb_funct3[2] ? {27'b0, mem_wb_inst[19:15]} : mem_wb_alu_result;

  reg [31:0] csr_mepc;
  reg [31:0] csr_mtvec;
  reg [31:0] csr_mscratch;
  reg [31:0] csr_mcause;
  reg [31:0] csr_mtval;
  reg [31:0] csr_mstatus;

  always @(posedge clock) begin
    if (reset_internal) begin
      csr_mepc     <= 32'b0;
      csr_mtvec    <= 32'b0;
      csr_mscratch <= 32'b0;
      csr_mcause   <= 32'b0;
      csr_mtval    <= 32'b0;
      csr_mstatus  <= 32'h00001800;
    end else if (clock_enable) begin
      if (wb_trap_taken) begin
        csr_mepc   <= mem_wb_pc;
        csr_mcause <= wb_is_ecall ? 32'd11 : 32'd3;
      end else if (mem_wb_valid && wb_is_csr) begin
        case (wb_csr_addr)
          12'h341: csr_mepc     <= wb_csr_wdata;
          12'h305: csr_mtvec    <= wb_csr_wdata;
          12'h340: csr_mscratch <= wb_csr_wdata;
          12'h342: csr_mcause   <= wb_csr_wdata;
          12'h343: csr_mtval    <= wb_csr_wdata;
          12'h300: csr_mstatus  <= wb_csr_wdata;
        endcase
      end
    end
  end

  reg [31:0] wb_csr_rdata;
  always @(*) begin
    case (wb_csr_addr)
      12'h341: wb_csr_rdata = csr_mepc;
      12'h305: wb_csr_rdata = csr_mtvec;
      12'h340: wb_csr_rdata = csr_mscratch;
      12'h342: wb_csr_rdata = csr_mcause;
      12'h343: wb_csr_rdata = csr_mtval;
      12'h300: wb_csr_rdata = csr_mstatus;
      12'hF14: wb_csr_rdata = 32'b0; // mhartid
      default: wb_csr_rdata = 32'b0;
    endcase
  end

  assign wb_mepc        = csr_mepc;
  assign wb_trap_taken  = mem_wb_valid && (wb_is_ecall || wb_is_ebreak);
  assign wb_trap_target = csr_mtvec;

  assign wb_write_data = wb_is_csr     ? wb_csr_rdata :
                         mem_wb_mem_read ? wb_load_data :
                         mem_wb_alu_result;
  assign wb_rd       = mem_wb_rd;
  assign wb_reg_write = mem_wb_reg_write && mem_wb_valid && !wb_trap_taken;

  // Unused interrupt outputs (directly mapped, no interrupt controller yet)
  assign irq_external_response = 1'b0;
  assign irq_timer_response    = 1'b0;
  assign irq_software_response = 1'b0;
  assign irq_fast_response     = 16'b0;

endmodule
