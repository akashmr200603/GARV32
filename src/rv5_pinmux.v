/* ==========================================================================
   rv5_pinmux.v - Universal Pin Multiplexer for Tiny Tapeout
   ========================================================================== */

module rv5_pinmux (
    input  wire        clk,
    input  wire        reset,
    
    // System Bus
    input  wire [31:0] bus_addr,
    input  wire [31:0] bus_wdata,
    input  wire        bus_we,
    input  wire        bus_req,
    output reg  [31:0] bus_rdata,
    output reg         bus_ack,

    // Physical Pins (Excludes dedicated QSPI pins)
    input  wire [7:0]  pin_in,       // ui_in[7:0]
    output reg  [4:0]  pin_out,      // uo_out[7:3]
    input  wire [3:0]  pin_uio_in,   // uio[7:4]
    output reg  [3:0]  pin_uio_out,  // uio[7:4]
    output reg  [3:0]  pin_uio_oe,   // uio[7:4]

    // Internal Peripheral Inputs (To Peripherals)
    output reg         uart0_rx,
    output reg         uart1_rx,
    output reg         spi0_miso,
    output reg         spi1_miso,
    output reg         i2s_sdata_in,
    output reg  [7:0]  gpio_in,

    // Internal Peripheral Outputs (From Peripherals)
    input  wire        uart0_tx,
    input  wire        uart1_tx,
    input  wire        spi0_mosi,
    input  wire        spi0_sck,
    input  wire        spi0_cs,
    input  wire        spi1_mosi,
    input  wire        spi1_sck,
    input  wire        spi1_cs,
    input  wire        i2s_sdata_out,
    input  wire        i2s_bclk,
    input  wire        i2s_lrclk,
    input  wire [7:0]  pwm_out,
    input  wire [7:0]  gpio_out,
    input  wire [7:0]  gpio_oe,
    input  wire        i2c0_sda_out,
    input  wire        i2c0_scl_out,
    input  wire        i2c0_sda_oe,
    input  wire        i2c0_scl_oe,
    input  wire        i2c1_sda_out,
    input  wire        i2c1_scl_out,
    input  wire        i2c1_sda_oe,
    input  wire        i2c1_scl_oe
);

    // -----------------------------------------------------------------------
    // Configuration Registers
    // -----------------------------------------------------------------------
    // 0x00: IN_CTRL    - [19:0] 4 bits per input (UART0, UART1, SPI0, SPI1, I2S)
    // 0x04: OUT_CTRL0  - [24:0] 5 bits per output pin (pin_out 0 to 4)
    // 0x08: OUT_CTRL1  - [19:0] 5 bits per bidir pin output (pin_uio_out 0 to 3)
    // 0x0C: OE_CTRL    - [15:0] 4 bits per bidir pin OE (pin_uio_oe 0 to 3)
    // 0x10: GPIO_IN    - [31:0] 4 bits per GPIO input (gpio_in 0 to 7)

    reg [31:0] reg_in_ctrl;
    reg [31:0] reg_out_ctrl0;
    reg [31:0] reg_out_ctrl1;
    reg [31:0] reg_oe_ctrl;
    reg [31:0] reg_gpio_in;

    always @(posedge clk) begin
        if (reset) begin
            reg_in_ctrl   <= 32'h00000000;
            reg_out_ctrl0 <= 32'h00000000;
            reg_out_ctrl1 <= 32'h00000000;
            reg_oe_ctrl   <= 32'h00000000;
            reg_gpio_in   <= 32'h00000000;
            bus_ack       <= 1'b0;
            bus_rdata     <= 32'h0;
        end else begin
            bus_ack <= 1'b0;
            if (bus_req && !bus_ack) begin
                bus_ack <= 1'b1;
                if (bus_we) begin
                    case (bus_addr[7:0])
                        8'h00: reg_in_ctrl   <= bus_wdata;
                        8'h04: reg_out_ctrl0 <= bus_wdata;
                        8'h08: reg_out_ctrl1 <= bus_wdata;
                        8'h0C: reg_oe_ctrl   <= bus_wdata;
                        8'h10: reg_gpio_in   <= bus_wdata;
                    endcase
                end else begin
                    case (bus_addr[7:0])
                        8'h00: bus_rdata <= reg_in_ctrl;
                        8'h04: bus_rdata <= reg_out_ctrl0;
                        8'h08: bus_rdata <= reg_out_ctrl1;
                        8'h0C: bus_rdata <= reg_oe_ctrl;
                        8'h10: bus_rdata <= reg_gpio_in;
                        default: bus_rdata <= 32'h0;
                    endcase
                end
            end
        end
    end

    // -----------------------------------------------------------------------
    // Input Multiplexer Function
    // -----------------------------------------------------------------------
    function automatic reg mux_in;
        input [3:0] sel;
        begin
            case (sel)
                4'd0:  mux_in = pin_in[0];
                4'd1:  mux_in = pin_in[1];
                4'd2:  mux_in = pin_in[2];
                4'd3:  mux_in = pin_in[3];
                4'd4:  mux_in = pin_in[4];
                4'd5:  mux_in = pin_in[5];
                4'd6:  mux_in = pin_in[6];
                4'd7:  mux_in = pin_in[7];
                4'd8:  mux_in = pin_uio_in[0];
                4'd9:  mux_in = pin_uio_in[1];
                4'd10: mux_in = pin_uio_in[2];
                4'd11: mux_in = pin_uio_in[3];
                4'd12: mux_in = 1'b1;
                4'd13: mux_in = 1'b0;
                default: mux_in = 1'b0;
            endcase
        end
    endfunction

    always @(*) begin
        uart0_rx     = mux_in(reg_in_ctrl[3:0]);
        uart1_rx     = mux_in(reg_in_ctrl[7:4]);
        spi0_miso    = mux_in(reg_in_ctrl[11:8]);
        spi1_miso    = mux_in(reg_in_ctrl[15:12]);
        i2s_sdata_in = mux_in(reg_in_ctrl[19:16]);
        
        gpio_in[0]   = mux_in(reg_gpio_in[3:0]);
        gpio_in[1]   = mux_in(reg_gpio_in[7:4]);
        gpio_in[2]   = mux_in(reg_gpio_in[11:8]);
        gpio_in[3]   = mux_in(reg_gpio_in[15:12]);
        gpio_in[4]   = mux_in(reg_gpio_in[19:16]);
        gpio_in[5]   = mux_in(reg_gpio_in[23:20]);
        gpio_in[6]   = mux_in(reg_gpio_in[27:24]);
        gpio_in[7]   = mux_in(reg_gpio_in[31:28]);
    end

    // -----------------------------------------------------------------------
    // Output Multiplexer Function
    // -----------------------------------------------------------------------
    function automatic reg mux_out;
        input [4:0] sel;
        begin
            case (sel)
                5'd0:  mux_out = 1'b0;
                5'd1:  mux_out = uart0_tx;
                5'd2:  mux_out = uart1_tx;
                5'd3:  mux_out = spi0_mosi;
                5'd4:  mux_out = spi0_sck;
                5'd5:  mux_out = spi0_cs;
                5'd6:  mux_out = spi1_mosi;
                5'd7:  mux_out = spi1_sck;
                5'd8:  mux_out = spi1_cs;
                5'd9:  mux_out = i2s_sdata_out;
                5'd10: mux_out = i2s_bclk;
                5'd11: mux_out = i2s_lrclk;
                5'd12: mux_out = pwm_out[0];
                5'd13: mux_out = pwm_out[1];
                5'd14: mux_out = pwm_out[2];
                5'd15: mux_out = pwm_out[3];
                5'd16: mux_out = pwm_out[4];
                5'd17: mux_out = pwm_out[5];
                5'd18: mux_out = pwm_out[6];
                5'd19: mux_out = pwm_out[7];
                5'd20: mux_out = gpio_out[0];
                5'd21: mux_out = gpio_out[1];
                5'd22: mux_out = gpio_out[2];
                5'd23: mux_out = gpio_out[3];
                5'd24: mux_out = gpio_out[4];
                5'd25: mux_out = gpio_out[5];
                5'd26: mux_out = gpio_out[6];
                5'd27: mux_out = gpio_out[7];
                5'd28: mux_out = i2c0_sda_out;
                5'd29: mux_out = i2c0_scl_out;
                5'd30: mux_out = i2c1_sda_out;
                5'd31: mux_out = i2c1_scl_out;
            endcase
        end
    endfunction

    always @(*) begin
        pin_out[0] = mux_out(reg_out_ctrl0[4:0]);
        pin_out[1] = mux_out(reg_out_ctrl0[9:5]);
        pin_out[2] = mux_out(reg_out_ctrl0[14:10]);
        pin_out[3] = mux_out(reg_out_ctrl0[19:15]);
        pin_out[4] = mux_out(reg_out_ctrl0[24:20]);

        pin_uio_out[0] = mux_out(reg_out_ctrl1[4:0]);
        pin_uio_out[1] = mux_out(reg_out_ctrl1[9:5]);
        pin_uio_out[2] = mux_out(reg_out_ctrl1[14:10]);
        pin_uio_out[3] = mux_out(reg_out_ctrl1[19:15]);
    end

    // -----------------------------------------------------------------------
    // Output Enable (OE) Multiplexer Function for Bidir pins
    // -----------------------------------------------------------------------
    function automatic reg mux_oe;
        input [3:0] sel;
        begin
            case (sel)
                4'd0:  mux_oe = 1'b1; // Always Output
                4'd1:  mux_oe = 1'b0; // Always Input
                4'd2:  mux_oe = gpio_oe[0];
                4'd3:  mux_oe = gpio_oe[1];
                4'd4:  mux_oe = gpio_oe[2];
                4'd5:  mux_oe = gpio_oe[3];
                4'd6:  mux_oe = gpio_oe[4];
                4'd7:  mux_oe = gpio_oe[5];
                4'd8:  mux_oe = gpio_oe[6];
                4'd9:  mux_oe = gpio_oe[7];
                4'd10: mux_oe = i2c0_sda_oe;
                4'd11: mux_oe = i2c0_scl_oe;
                4'd12: mux_oe = i2c1_sda_oe;
                4'd13: mux_oe = i2c1_scl_oe;
                default: mux_oe = 1'b0;
            endcase
        end
    endfunction

    always @(*) begin
        pin_uio_oe[0] = mux_oe(reg_oe_ctrl[3:0]);
        pin_uio_oe[1] = mux_oe(reg_oe_ctrl[7:4]);
        pin_uio_oe[2] = mux_oe(reg_oe_ctrl[11:8]);
        pin_uio_oe[3] = mux_oe(reg_oe_ctrl[15:12]);
    end

endmodule
