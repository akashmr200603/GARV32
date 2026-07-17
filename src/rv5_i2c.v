// SPDX-License-Identifier: MIT
// Copyright (c) 2020-2025 RV5 Project Contributors

module rv5_i2c (

  // Global signals

  input   wire          clock           ,
  input   wire          reset           ,

  // IO interface

  input  wire   [4:0 ]  rw_address      ,
  output reg    [31:0]  read_data       ,
  input  wire           read_request    ,
  output reg            read_response   ,
  input  wire   [7:0 ]  write_data      ,
  input  wire   [3:0 ]  write_strobe    ,
  input  wire           write_request   ,
  output reg            write_response  ,

  // I2C signals

  output reg            scl_o           ,
  output reg            scl_oe          ,
  input  wire           sda_i           ,
  output reg            sda_o           ,
  output reg            sda_oe

  );

  // --------------------------------------------------------------------------
  //  Register Map
  // --------------------------------------------------------------------------

  localparam REG_CTRL      = 5'h00;   // Control: [0]=START [1]=STOP [2]=RD [3]=WR [4]=ACK_EN
  localparam REG_STATUS    = 5'h04;   // Status:  [0]=BUSY [1]=ACK_RECV [2]=ARB_LOST
  localparam REG_WDATA     = 5'h08;   // Write data (8-bit)
  localparam REG_RDATA     = 5'h0C;   // Read data (8-bit)
  localparam REG_PRESCALE  = 5'h10;   // SCL prescaler (8-bit)

  // --------------------------------------------------------------------------
  //  I2C State Machine
  // --------------------------------------------------------------------------

  localparam ST_IDLE       = 4'd0;
  localparam ST_START_1    = 4'd1;    // SDA low, SCL high  (setup)
  localparam ST_START_2    = 4'd2;    // SDA low, SCL low   (hold)
  localparam ST_WRITE_BIT  = 4'd3;    // Shift out data bit (SCL low)
  localparam ST_WRITE_CLK  = 4'd4;    // Pulse SCL high for write
  localparam ST_READ_BIT   = 4'd5;    // Pulse SCL high for read (sample SDA)
  localparam ST_READ_CLK   = 4'd6;    // SCL low after read sample
  localparam ST_WRITE_ACK  = 4'd7;    // Send ACK/NACK after reading a byte
  localparam ST_WRITE_ACK2 = 4'd8;    // Clock the ACK/NACK bit
  localparam ST_READ_ACK   = 4'd9;    // Sample ACK after writing a byte
  localparam ST_READ_ACK2  = 4'd10;   // SCL low after ACK sample
  localparam ST_STOP_1     = 4'd11;   // SDA low, SCL high
  localparam ST_STOP_2     = 4'd12;   // SDA high, SCL high (stop condition)

  // --------------------------------------------------------------------------
  //  Internal Registers
  // --------------------------------------------------------------------------

  reg [3:0]  state;
  reg [7:0]  prescale;
  reg [7:0]  clk_cnt;
  reg [7:0]  tx_shift;
  reg [7:0]  rx_shift;
  reg [7:0]  rx_data;
  reg [2:0]  bit_cnt;
  reg        busy;
  reg        ack_recv;
  reg        arb_lost;
  reg        ack_en;         // 1 = send ACK after read, 0 = send NACK

  // Command registers (self-clearing)
  reg        cmd_start;
  reg        cmd_stop;
  reg        cmd_read;
  reg        cmd_write;

  wire valid_write = write_request == 1'b1 && &write_strobe == 1'b1;

  // Quarter-period tick: SCL freq = sys_clk / (4 * (prescale + 1))
  wire clk_tick = (clk_cnt == prescale);

  // --------------------------------------------------------------------------
  //  Bus Response (1-cycle latency, same as all other peripherals)
  // --------------------------------------------------------------------------

  always @(posedge clock) begin
    if (reset) begin
      read_response  <= 1'b0;
      write_response <= 1'b0;
    end
    else begin
      read_response  <= read_request;
      write_response <= write_request;
    end
  end

  // --------------------------------------------------------------------------
  //  Register Read
  // --------------------------------------------------------------------------

  always @(posedge clock) begin
    if (reset)
      read_data <= 32'h00000000;
    else if (read_request == 1'b1) begin
      case (rw_address)
        REG_CTRL:     read_data <= {27'b0, ack_en, cmd_write, cmd_read, cmd_stop, cmd_start};
        REG_STATUS:   read_data <= {29'b0, arb_lost, ack_recv, busy};
        REG_WDATA:    read_data <= {24'b0, tx_shift};
        REG_RDATA:    read_data <= {24'b0, rx_data};
        REG_PRESCALE: read_data <= {24'b0, prescale};
        default:      read_data <= 32'h00000000;
      endcase
    end
    else
      read_data <= 32'h00000000;
  end

  // --------------------------------------------------------------------------
  //  Register Write - Prescaler
  // --------------------------------------------------------------------------

  always @(posedge clock) begin
    if (reset)
      prescale <= 8'hFF;
    else if (rw_address == REG_PRESCALE && valid_write)
      prescale <= write_data[7:0];
  end

  // --------------------------------------------------------------------------
  //  Clock Counter (quarter-period tick generator)
  // --------------------------------------------------------------------------

  always @(posedge clock) begin
    if (reset)
      clk_cnt <= 8'd0;
    else if (state == ST_IDLE)
      clk_cnt <= 8'd0;
    else if (clk_tick)
      clk_cnt <= 8'd0;
    else
      clk_cnt <= clk_cnt + 8'd1;
  end

  // --------------------------------------------------------------------------
  //  Command & TX Data Register Writes
  // --------------------------------------------------------------------------

  always @(posedge clock) begin
    if (reset) begin
      cmd_start <= 1'b0;
      cmd_stop  <= 1'b0;
      cmd_read  <= 1'b0;
      cmd_write <= 1'b0;
      ack_en    <= 1'b0;
      tx_shift  <= 8'h00;
    end
    else if (rw_address == REG_WDATA && valid_write && !busy) begin
      tx_shift <= write_data[7:0];
    end
    else if (rw_address == REG_CTRL && valid_write) begin
      cmd_start <= write_data[0];
      cmd_stop  <= write_data[1];
      cmd_read  <= write_data[2];
      cmd_write <= write_data[3];
      ack_en    <= write_data[4];
    end
    else begin
      // Self-clear commands when the FSM picks them up
      if (state != ST_IDLE) begin
        cmd_start <= 1'b0;
        cmd_stop  <= 1'b0;
        cmd_read  <= 1'b0;
        cmd_write <= 1'b0;
      end
    end
  end

  // --------------------------------------------------------------------------
  //  I2C Master State Machine
  // --------------------------------------------------------------------------

  always @(posedge clock) begin
    if (reset) begin
      state    <= ST_IDLE;
      scl_o    <= 1'b1;
      scl_oe   <= 1'b1;
      sda_o    <= 1'b1;
      sda_oe   <= 1'b1;
      rx_shift <= 8'h00;
      rx_data  <= 8'h00;
      bit_cnt  <= 3'd7;
      busy     <= 1'b0;
      ack_recv <= 1'b0;
      arb_lost <= 1'b0;
    end
    else begin
      case (state)

        // ---- IDLE: Wait for a command ----
        ST_IDLE: begin
          busy <= 1'b0;
          if (cmd_start) begin
            // Generate START condition
            sda_o  <= 1'b1;
            sda_oe <= 1'b1;
            scl_o  <= 1'b1;
            scl_oe <= 1'b1;
            busy   <= 1'b1;
            state  <= ST_START_1;
          end
          else if (cmd_write) begin
            // Write a byte (tx_shift already loaded)
            bit_cnt  <= 3'd7;
            busy     <= 1'b1;
            sda_o    <= tx_shift[7];
            sda_oe   <= 1'b1;
            scl_o    <= 1'b0;
            scl_oe   <= 1'b1;
            state    <= ST_WRITE_BIT;
          end
          else if (cmd_read) begin
            // Read a byte
            bit_cnt  <= 3'd7;
            rx_shift <= 8'h00;
            busy     <= 1'b1;
            sda_oe   <= 1'b0;      // Release SDA for peripheral to drive
            scl_o    <= 1'b0;
            scl_oe   <= 1'b1;
            state    <= ST_READ_BIT;
          end
          else if (cmd_stop) begin
            sda_o  <= 1'b0;
            sda_oe <= 1'b1;
            scl_o  <= 1'b0;
            scl_oe <= 1'b1;
            busy   <= 1'b1;
            state  <= ST_STOP_1;
          end
        end

        // ---- START Condition ----
        // START = SDA falls while SCL is high
        ST_START_1: begin
          if (clk_tick) begin
            sda_o <= 1'b0;         // Pull SDA low while SCL is high
            state <= ST_START_2;
          end
        end

        ST_START_2: begin
          if (clk_tick) begin
            scl_o <= 1'b0;         // Pull SCL low (hold time)
            state <= ST_IDLE;
            busy  <= 1'b0;
          end
        end

        // ---- WRITE: Shift out 8 bits, MSB first ----
        ST_WRITE_BIT: begin
          // Data is set on SDA while SCL is low
          sda_o  <= tx_shift[bit_cnt];
          sda_oe <= 1'b1;
          scl_o  <= 1'b0;
          if (clk_tick) begin
            scl_o <= 1'b1;         // Raise SCL
            state <= ST_WRITE_CLK;
          end
        end

        ST_WRITE_CLK: begin
          scl_o <= 1'b1;
          if (clk_tick) begin
            // Check for arbitration loss: if we drive SDA high but read low
            if (sda_oe && sda_o && !sda_i) begin
              arb_lost <= 1'b1;
              sda_oe   <= 1'b0;
              scl_o    <= 1'b0;
              state    <= ST_IDLE;
              busy     <= 1'b0;
            end
            else begin
              scl_o <= 1'b0;       // Lower SCL
              if (bit_cnt == 3'd0) begin
                // All 8 bits sent, read ACK from peripheral
                sda_oe <= 1'b0;    // Release SDA
                state  <= ST_READ_ACK;
              end
              else begin
                bit_cnt <= bit_cnt - 3'd1;
                state   <= ST_WRITE_BIT;
              end
            end
          end
        end

        // ---- READ ACK (after write): Sample SDA on SCL high ----
        ST_READ_ACK: begin
          scl_o <= 1'b0;
          if (clk_tick) begin
            scl_o <= 1'b1;
            state <= ST_READ_ACK2;
          end
        end

        ST_READ_ACK2: begin
          scl_o <= 1'b1;
          if (clk_tick) begin
            ack_recv <= ~sda_i;    // ACK = SDA low
            scl_o    <= 1'b0;
            state    <= ST_IDLE;
            busy     <= 1'b0;
          end
        end

        // ---- READ: Shift in 8 bits, MSB first ----
        ST_READ_BIT: begin
          sda_oe <= 1'b0;          // Keep SDA released
          scl_o  <= 1'b0;
          if (clk_tick) begin
            scl_o <= 1'b1;         // Raise SCL to sample
            state <= ST_READ_CLK;
          end
        end

        ST_READ_CLK: begin
          scl_o <= 1'b1;
          if (clk_tick) begin
            rx_shift <= {rx_shift[6:0], sda_i};   // Sample SDA
            scl_o    <= 1'b0;
            if (bit_cnt == 3'd0) begin
              rx_data <= {rx_shift[6:0], sda_i};   // Capture full byte
              // Send ACK or NACK
              sda_o   <= ack_en ? 1'b0 : 1'b1;    // ACK=low, NACK=high
              sda_oe  <= 1'b1;
              state   <= ST_WRITE_ACK;
            end
            else begin
              bit_cnt <= bit_cnt - 3'd1;
              state   <= ST_READ_BIT;
            end
          end
        end

        // ---- WRITE ACK/NACK (after read) ----
        ST_WRITE_ACK: begin
          scl_o <= 1'b0;
          if (clk_tick) begin
            scl_o <= 1'b1;
            state <= ST_WRITE_ACK2;
          end
        end

        ST_WRITE_ACK2: begin
          scl_o <= 1'b1;
          if (clk_tick) begin
            scl_o  <= 1'b0;
            sda_oe <= 1'b0;       // Release SDA
            state  <= ST_IDLE;
            busy   <= 1'b0;
          end
        end

        // ---- STOP Condition ----
        // STOP = SDA rises while SCL is high
        ST_STOP_1: begin
          sda_o  <= 1'b0;
          sda_oe <= 1'b1;
          scl_o  <= 1'b0;
          if (clk_tick) begin
            scl_o <= 1'b1;         // Raise SCL first
            state <= ST_STOP_2;
          end
        end

        ST_STOP_2: begin
          scl_o <= 1'b1;
          if (clk_tick) begin
            sda_o  <= 1'b1;       // Release SDA while SCL is high
            sda_oe <= 1'b1;
            state  <= ST_IDLE;
            busy   <= 1'b0;
          end
        end

        default: begin
          state <= ST_IDLE;
          busy  <= 1'b0;
        end

      endcase
    end
  end

endmodule
