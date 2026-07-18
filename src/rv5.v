// SPDX-License-Identifier: MIT
// Copyright (c) 2020-2025 RV5 Project Contributors

module rv5 #(

  // Frequency of 'clock' signal
  parameter CLOCK_FREQUENCY = 50000000   ,
  // Desired baud rate for UART unit
  parameter UART_BAUD_RATE = 9600         ,
  // Internal SRAM size in bytes - must be a power of 2
  parameter MEMORY_SIZE = 64             ,
  // Address of the first instruction to fetch from memory
  parameter BOOT_ADDRESS = 32'h00000000 ,
  parameter GPIO_WIDTH = 8              ,
  // Number of CS (Chip Select) pins for the SPI controllers
  parameter SPI_NUM_CHIP_SELECT = 1     ,
  parameter UART1_BAUD_RATE = 9600

  ) (

  input   wire                             clock          ,
  input   wire                             reset          ,
  input   wire                             halt           ,
  // Pin Mux Interface
  input   wire  [7:0]                      pin_in         ,
  output  wire  [4:0]                      pin_out        ,
  input   wire  [3:0]                      pin_uio_in     ,
  output  wire  [3:0]                      pin_uio_out    ,
  output  wire  [3:0]                      pin_uio_oe     ,
  // QSPI
  output  wire                             qspi_sck       ,
  output  wire                             qspi_cs_n      ,
  output  wire                             qspi_cs_ram_n  ,
  input   wire  [3:0]                      qspi_io_in     ,
  output  wire  [3:0]                      qspi_io_out    ,
  output  wire                             qspi_io_oe


  );

  // System bus configuration

  localparam NUM_DEVICES    = 14;
  localparam D0_ROM         = 0;
  localparam D1_RAM         = 1;
  localparam D2_UART        = 2;
  localparam D3_MTIMER      = 3;
  localparam D4_GPIO        = 4;
  localparam D5_SPI         = 5;
  localparam D6_UART1       = 6;
  localparam D7_I2C0        = 7;
  localparam D8_PWM         = 8;
  localparam D9_I2S         = 9;
  localparam D10_QRAM       = 10;
  localparam D11_HK         = 11;
  localparam D12_PINMUX     = 12;
  localparam D13_UNUSED     = 13;

  wire  [NUM_DEVICES*32-1:0] device_start_address;
  wire  [NUM_DEVICES*32-1:0] device_mask_address;





  assign device_start_address [32*D0_ROM      +: 32]  = 32'h0000_0000;
  assign device_mask_address  [32*D0_ROM      +: 32]  = ~(32'h0040_0000 - 1); // 4MB Flash

  assign device_start_address [32*D1_RAM      +: 32]  = 32'h2000_0000;
  assign device_mask_address  [32*D1_RAM      +: 32]  = ~(MEMORY_SIZE - 1);

  assign device_start_address [32*D2_UART     +: 32]  = 32'h8000_0000;
  assign device_mask_address  [32*D2_UART     +: 32]  = ~(32'd16 - 1);

  assign device_start_address [32*D3_MTIMER   +: 32]  = 32'h8001_0000;
  assign device_mask_address  [32*D3_MTIMER   +: 32]  = ~(32'd32 - 1);

  assign device_start_address [32*D4_GPIO     +: 32]  = 32'h8002_0000;
  assign device_mask_address  [32*D4_GPIO     +: 32]  = ~(32'd32 - 1);

  assign device_start_address [32*D5_SPI      +: 32]  = 32'h8003_0000;
  assign device_mask_address  [32*D5_SPI      +: 32]  = ~(32'd32 - 1);

  assign device_start_address [32*D6_UART1    +: 32]  = 32'h8004_0000;
  assign device_mask_address  [32*D6_UART1    +: 32]  = ~(32'd16 - 1);

  assign device_start_address [32*D7_I2C0     +: 32]  = 32'h8006_0000;
  assign device_mask_address  [32*D7_I2C0     +: 32]  = ~(32'd32 - 1);

  assign device_start_address [32*D8_PWM      +: 32]  = 32'h8008_0000;
  assign device_mask_address  [32*D8_PWM      +: 32]  = ~(32'd64 - 1);

  assign device_start_address [32*D9_I2S      +: 32]  = 32'h8009_0000;
  assign device_mask_address  [32*D9_I2S      +: 32]  = ~(32'd32 - 1);

  assign device_start_address [32*D10_QRAM    +: 32]  = 32'h1000_0000;
  assign device_mask_address  [32*D10_QRAM    +: 32]  = ~(32'h0080_0000 - 1); // 8MB PSRAM

  assign device_start_address [32*D11_HK      +: 32]  = 32'h800A_0000;
  assign device_mask_address  [32*D11_HK      +: 32]  = ~(32'd32 - 1);

  assign device_start_address [32*D12_PINMUX  +: 32]  = 32'h800B_0000;
  assign device_mask_address  [32*D12_PINMUX  +: 32]  = ~(32'd32 - 1);

  assign device_start_address [32*D13_UNUSED  +: 32]  = 32'hFFFF_FFFF;
  assign device_mask_address  [32*D13_UNUSED  +: 32]  = 32'hFFFF_FFFF;

  // RV5 32-bit Processor (Manager Device) <=> System Bus

  // IF Interface (Instruction Fetch)
  wire  [31:0]                manager_if_address      ;
  wire                        manager_if_request      ;
  wire  [31:0]                manager_if_read_data    ;
  wire                        manager_if_response     ;

  // I-Cache wires
  wire [31:0]                 icache_mem_address;
  wire [31:0]                 icache_mem_read_data;
  wire                        icache_mem_request;
  wire                        icache_mem_response;

  // Multiplex icache mem request between QSPI (0x0/0x10) and RAM (0x20000000)
  wire                        icache_is_ram = (icache_mem_address[31:28] == 4'h2);
  
  wire                        qspi_if_request = icache_mem_request & ~icache_is_ram;
  wire [31:0]                 qspi_if_read_data;
  wire                        qspi_if_response;

  wire                        ram_if_request = icache_mem_request & icache_is_ram;
  wire [31:0]                 ram_if_read_data;
  wire                        ram_if_response;

  assign icache_mem_read_data = icache_is_ram ? ram_if_read_data : qspi_if_read_data;
  assign icache_mem_response  = icache_is_ram ? ram_if_response  : qspi_if_response;

  // QSPI memory data bus multiplexing
  wire qspi_mem_read_request = device_read_request[D0_ROM] | device_read_request[D10_QRAM];
  wire qspi_mem_write_request = device_write_request[D0_ROM] | device_write_request[D10_QRAM];
  wire [31:0] qspi_mem_read_data;
  wire qspi_mem_read_response;
  wire qspi_mem_write_response;

  assign device_read_data[32*D0_ROM +: 32] = qspi_mem_read_data;
  assign device_read_response[D0_ROM] = qspi_mem_read_response & device_read_request[D0_ROM];
  assign device_write_response[D0_ROM] = qspi_mem_write_response & device_write_request[D0_ROM];

  assign device_read_data[32*D10_QRAM +: 32] = qspi_mem_read_data;
  assign device_read_response[D10_QRAM] = qspi_mem_read_response & device_read_request[D10_QRAM];
  assign device_write_response[D10_QRAM] = qspi_mem_write_response & device_write_request[D10_QRAM];

  // Unused device slot - tie off
  assign device_read_data[32*D13_UNUSED +: 32] = 32'h0;
  assign device_read_response[D13_UNUSED] = device_read_request[D13_UNUSED];
  assign device_write_response[D13_UNUSED] = device_write_request[D13_UNUSED];

  // MEM Interface (Data Memory) -> to System Bus
  wire  [31:0]                manager_mem_address      ;
  wire                        manager_mem_read_request ;
  wire  [31:0]                manager_mem_read_data    ;
  wire                        manager_mem_read_response;
  wire  [31:0]                manager_mem_write_data   ;
  wire  [3:0 ]                manager_mem_write_strobe ;
  wire                        manager_mem_write_request;
  wire                        manager_mem_write_response;

  // System Bus <=> Managed Devices

  wire  [31:0]                device_rw_address       ;
  wire  [NUM_DEVICES*32-1:0]  device_read_data        ;
  wire  [NUM_DEVICES-1:0]     device_read_request     ;
  wire  [NUM_DEVICES-1:0]     device_read_response    ;
  wire  [31:0]                device_write_data       ;
  wire  [3:0]                 device_write_strobe     ;
  wire  [NUM_DEVICES-1:0]     device_write_request    ;
  wire  [NUM_DEVICES-1:0]     device_write_response   ;

  // Real-time clock (unused)

  wire  [63:0] real_time_clock;

  assign real_time_clock = 64'b0;

  // Interrupt signals

  wire  [15:0] irq_fast;
  wire         irq_external;
  wire         irq_timer;
  wire         irq_software;

  wire  [15:0] irq_fast_response;
  wire         irq_external_response;
  wire         irq_timer_response;
  wire         irq_software_response;

  wire         irq_uart;
  wire         irq_uart_response;

  wire         irq_uart1;
  wire         irq_uart1_response;

  // Interrupt signals map

  assign irq_fast               = {14'b0, irq_uart1, irq_uart}; // UART0=fast[0], UART1=fast[1]
  assign irq_uart_response      = irq_fast_response[0];
  assign irq_uart1_response     = irq_fast_response[1];

  assign irq_external           = 1'b0; // unused
  assign irq_software           = 1'b0; // unused


  rv5_core #(

    .BOOT_ADDRESS                   (BOOT_ADDRESS                       )

  ) rv5_core_instance (

    // Global signals

    .clock                          (clock                              ),
    .reset                          (reset                              ),
    .halt                           (halt                               ),

    // IF interface (Instruction Fetch)

    .if_address                     (manager_if_address                 ),
    .if_request                     (manager_if_request                 ),
    .if_read_data                   (manager_if_read_data               ),
    .if_response                    (manager_if_response                ),

    // MEM interface (Data Memory)

    .mem_address                    (manager_mem_address                ),
    .mem_read_request               (manager_mem_read_request           ),
    .mem_read_data                  (manager_mem_read_data              ),
    .mem_read_response              (manager_mem_read_response          ),
    .mem_write_data                 (manager_mem_write_data             ),
    .mem_write_strobe               (manager_mem_write_strobe           ),
    .mem_write_request              (manager_mem_write_request          ),
    .mem_write_response             (manager_mem_write_response         ),

    // Interrupt request signals

    .irq_fast                       (irq_fast                           ),
    .irq_external                   (irq_external                       ),
    .irq_timer                      (irq_timer                          ),
    .irq_software                   (irq_software                       ),

    // Interrupt response signals

    .irq_fast_response              (irq_fast_response                  ),
    .irq_external_response          (irq_external_response              ),
    .irq_timer_response             (irq_timer_response                 ),
    .irq_software_response          (irq_software_response              ),

    // Real Time Clock

    .real_time_clock                (real_time_clock                    )

  );

  rv5_bus #(

    .NUM_DEVICES(NUM_DEVICES)

  ) rv5_bus_instance (

    // Global signals

    .clock                          (clock                              ),
    .reset                          (reset                              ),

    // Interface with the manager device (Processor Core IP)

    .manager_rw_address             (manager_mem_address                ),
    .manager_read_request           (manager_mem_read_request           ),
    .manager_read_data              (manager_mem_read_data              ),
    .manager_read_response          (manager_mem_read_response          ),
    .manager_write_data             (manager_mem_write_data             ),
    .manager_write_strobe           (manager_mem_write_strobe           ),
    .manager_write_request          (manager_mem_write_request          ),
    .manager_write_response         (manager_mem_write_response         ),

    // Interface with the managed devices

    .device_rw_address              (device_rw_address                  ),
    .device_read_data               (device_read_data                   ),
    .device_read_request            (device_read_request                ),
    .device_read_response           (device_read_response               ),
    .device_write_data              (device_write_data                  ),
    .device_write_strobe            (device_write_strobe                ),
    .device_write_request           (device_write_request               ),
    .device_write_response          (device_write_response              ),

    // Base addresses and masks of the managed devices

    .device_start_address          (device_start_address                ),
    .device_mask_address           (device_mask_address                 )

  );

  rv5_icache #(
    .CACHE_LINES(16)
  ) rv5_icache_instance (
    .clock                          (clock                              ),
    .reset                          (reset                              ),
    .core_if_address                (manager_if_address                 ),
    .core_if_read_data              (manager_if_read_data               ),
    .core_if_request                (manager_if_request                 ),
    .core_if_response               (manager_if_response                ),
    .mem_if_address                 (icache_mem_address                 ),
    .mem_if_read_data               (icache_mem_read_data               ),
    .mem_if_request                 (icache_mem_request                 ),
    .mem_if_response                (icache_mem_response                )
  );

  rv5_qspi rv5_qspi_instance (
    .clock                          (clock                              ),
    .reset                          (reset                              ),
    .if_address                     (icache_mem_address                 ),
    .if_read_data                   (qspi_if_read_data                  ),
    .if_request                     (qspi_if_request                    ),
    .if_response                    (qspi_if_response                   ),
    .mem_address                    (device_rw_address                  ),
    .mem_read_data                  (qspi_mem_read_data                 ),
    .mem_read_request               (qspi_mem_read_request              ),
    .mem_read_response              (qspi_mem_read_response             ),
    .mem_write_data                 (device_write_data                  ),
    .mem_write_strobe               (device_write_strobe                ),
    .mem_write_request              (qspi_mem_write_request             ),
    .mem_write_response             (qspi_mem_write_response            ),
    .qspi_sck                       (qspi_sck                           ),
    .qspi_cs_n                      (qspi_cs_n                          ),
    .qspi_cs_ram_n                  (qspi_cs_ram_n                      ),
    .qspi_io_in                     (qspi_io_in                         ),
    .qspi_io_out                    (qspi_io_out                        ),
    .qspi_io_oe                     (qspi_io_oe                         )
  );

  rv5_housekeeping rv5_housekeeping_instance (
    .clock                          (clock                              ),
    .reset                          (reset                              ),
    .rw_address                     (device_rw_address[4:0]             ),
    .read_data                      (device_read_data[32*D11_HK +: 32]  ),
    .read_request                   (device_read_request[D11_HK]        ),
    .read_response                  (device_read_response[D11_HK]       ),
    .write_data                     (device_write_data                  ),
    .write_strobe                   (device_write_strobe                ),
    .write_request                  (device_write_request[D11_HK]       ),
    .write_response                 (device_write_response[D11_HK]      )
  );

  rv5_ram #(

    .MEMORY_SIZE                    (MEMORY_SIZE                        )

  ) rv5_ram_instance (

    // Global signals

    .clock                          (clock                              ),
    .reset                          (reset                              ),

    // Port A: Instruction Fetch

    .if_address                     (icache_mem_address                 ),
    .if_read_data                   (ram_if_read_data                   ),
    .if_request                     (ram_if_request                     ),
    .if_response                    (ram_if_response                    ),

    // Port B: Data Memory

    .mem_address                    (device_rw_address                  ),
    .mem_read_data                  (device_read_data[32*D1_RAM +: 32]  ),
    .mem_read_request               (device_read_request[D1_RAM]        ),
    .mem_read_response              (device_read_response[D1_RAM]       ),
    .mem_write_data                 (device_write_data                  ),
    .mem_write_strobe               (device_write_strobe                ),
    .mem_write_request              (device_write_request[D1_RAM]       ),
    .mem_write_response             (device_write_response[D1_RAM]      )

  );

  rv5_uart #(

    .CLOCK_FREQUENCY                (CLOCK_FREQUENCY                    ),
    .UART_BAUD_RATE                 (UART_BAUD_RATE                     )

  ) rv5_uart_instance (

    // Global signals

    .clock                          (clock                              ),
    .reset                          (reset                              ),

    // IO interface

    .rw_address                     (device_rw_address[4:0]             ),
    .read_data                      (device_read_data[32*D2_UART +: 32] ),
    .read_request                   (device_read_request[D2_UART]       ),
    .read_response                  (device_read_response[D2_UART]      ),
    .write_data                     (device_write_data[7:0]             ),
    .write_request                  (device_write_request[D2_UART]      ),
    .write_response                 (device_write_response[D2_UART]     ),

    // RX/TX signals

    .uart_tx                        (uart_tx                            ),
    .uart_rx                        (uart_rx                            ),

    // Interrupt signaling

    .uart_irq                       (irq_uart                           ),
    .uart_irq_response              (irq_uart_response                  )

  );

  rv5_mtimer
  rv5_mtimer_instance (

    // Global signals

    .clock                          (clock                                  ),
    .reset                          (reset                                  ),

    // IO interface

    .rw_address                     (device_rw_address[4:0]                 ),
    .read_data                      (device_read_data[32*D3_MTIMER +: 32]   ),
    .read_request                   (device_read_request[D3_MTIMER]         ),
    .read_response                  (device_read_response[D3_MTIMER]        ),
    .write_data                     (device_write_data                      ),
    .write_strobe                   (device_write_strobe                    ),
    .write_request                  (device_write_request[D3_MTIMER]        ),
    .write_response                 (device_write_response[D3_MTIMER]       ),

    // Interrupt signaling

    .irq                            (irq_timer                              )

  );

  rv5_gpio #(

    .GPIO_WIDTH                     (GPIO_WIDTH                             )

  ) rv5_gpio_instance (

    // Global signals

    .clock                          (clock                                  ),
    .reset                          (reset                                  ),

    // IO interface

    .rw_address                     (device_rw_address[4:0]                 ),
    .read_data                      (device_read_data[32*D4_GPIO +: 32]     ),
    .read_request                   (device_read_request[D4_GPIO]           ),
    .read_response                  (device_read_response[D4_GPIO]          ),
    .write_data                     (device_write_data[GPIO_WIDTH-1:0]      ),
    .write_strobe                   (device_write_strobe                    ),
    .write_request                  (device_write_request[D4_GPIO]          ),
    .write_response                 (device_write_response[D4_GPIO]         ),

    // I/O signals

    .gpio_input                     (gpio_input_internal                    ),
    .gpio_oe                        (gpio_oe_internal                       ),
    .gpio_output                    (gpio_output_internal                   )

  );

  // Internal peripheral wires for Pin Mux
  wire        uart_tx;
  wire        uart_rx;
  wire        uart1_tx;
  wire        uart1_rx;
  wire        pico;
  wire        sclk;
  wire        poci;
  wire [SPI_NUM_CHIP_SELECT-1:0] cs;
  wire [GPIO_WIDTH-1:0] gpio_input_internal;
  wire [GPIO_WIDTH-1:0] gpio_output_internal;
  wire [GPIO_WIDTH-1:0] gpio_oe_internal;
  wire        i2c0_sda_i;
  wire        i2c0_sda_o;
  wire        i2c0_sda_oe;
  wire        i2c0_scl_o;
  wire        i2c0_scl_oe;
  wire [7:0]  pwm_out;
  wire        i2s_bclk;
  wire        i2s_lrclk;
  wire        i2s_sdata_out;
  wire        i2s_sdata_in;

  rv5_spi #(

    .SPI_NUM_CHIP_SELECT            (SPI_NUM_CHIP_SELECT                    )

  ) rv5_spi_instance (

    // Global signals

    .clock                          (clock                              ),
    .reset                          (reset                              ),

    // IO interface

    .rw_address                     (device_rw_address[4:0]             ),
    .read_data                      (device_read_data[32*D5_SPI +: 32]  ),
    .read_request                   (device_read_request[D5_SPI]        ),
    .read_response                  (device_read_response[D5_SPI]       ),
    .write_data                     (device_write_data[7:0]             ),
    .write_strobe                   (device_write_strobe                ),
    .write_request                  (device_write_request[D5_SPI]       ),
    .write_response                 (device_write_response[D5_SPI]      ),

    // SPI signals

    .sclk                           (sclk                               ),
    .pico                           (pico                               ),
    .poci                           (poci                               ),
    .cs                             (cs                                 )

  );

  // --------------------------------------------------------------------------
  //  UART1 Instance
  // --------------------------------------------------------------------------

  rv5_uart #(

    .CLOCK_FREQUENCY                (CLOCK_FREQUENCY                    ),
    .UART_BAUD_RATE                 (UART1_BAUD_RATE                    )

  ) rv5_uart1_instance (

    // Global signals

    .clock                          (clock                              ),
    .reset                          (reset                              ),

    // IO interface

    .rw_address                     (device_rw_address[4:0]             ),
    .read_data                      (device_read_data[32*D6_UART1 +: 32]),
    .read_request                   (device_read_request[D6_UART1]      ),
    .read_response                  (device_read_response[D6_UART1]     ),
    .write_data                     (device_write_data[7:0]             ),
    .write_request                  (device_write_request[D6_UART1]     ),
    .write_response                 (device_write_response[D6_UART1]    ),

    // RX/TX signals

    .uart_tx                        (uart1_tx                           ),
    .uart_rx                        (uart1_rx                           ),

    // Interrupt signaling

    .uart_irq                       (irq_uart1                          ),
    .uart_irq_response              (irq_uart1_response                 )

  );


  // --------------------------------------------------------------------------
  //  I2C0 Instance
  // --------------------------------------------------------------------------

  rv5_i2c rv5_i2c0_instance (

    // Global signals

    .clock                          (clock                              ),
    .reset                          (reset                              ),

    // IO interface

    .rw_address                     (device_rw_address[4:0]             ),
    .read_data                      (device_read_data[32*D7_I2C0 +: 32]),
    .read_request                   (device_read_request[D7_I2C0]       ),
    .read_response                  (device_read_response[D7_I2C0]      ),
    .write_data                     (device_write_data[7:0]             ),
    .write_strobe                   (device_write_strobe                ),
    .write_request                  (device_write_request[D7_I2C0]      ),
    .write_response                 (device_write_response[D7_I2C0]     ),

    // I2C signals

    .scl_o                          (i2c0_scl_o                         ),
    .scl_oe                         (i2c0_scl_oe                        ),
    .sda_i                          (i2c0_sda_i                         ),
    .sda_o                          (i2c0_sda_o                         ),
    .sda_oe                         (i2c0_sda_oe                        )

  );


  // --------------------------------------------------------------------------
  //  PWM Instance
  // --------------------------------------------------------------------------

  rv5_pwm #(.CHANNELS(8)) rv5_pwm_instance (
    .clock                          (clock                              ),
    .reset                          (reset                              ),
    .rw_address                     (device_rw_address[7:0]             ),
    .read_data                      (device_read_data[32*D8_PWM +: 32]  ),
    .read_request                   (device_read_request[D8_PWM]        ),
    .read_response                  (device_read_response[D8_PWM]       ),
    .write_data                     (device_write_data                  ),
    .write_strobe                   (device_write_strobe                ),
    .write_request                  (device_write_request[D8_PWM]       ),
    .write_response                 (device_write_response[D8_PWM]      ),
    .pwm_out                        (pwm_out                            )
  );

  // --------------------------------------------------------------------------
  //  I2S Instance
  // --------------------------------------------------------------------------

  rv5_i2s rv5_i2s_instance (
    .clock                          (clock                              ),
    .reset                          (reset                              ),
    .rw_address                     (device_rw_address[7:0]             ),
    .read_data                      (device_read_data[32*D9_I2S +: 32]  ),
    .read_request                   (device_read_request[D9_I2S]        ),
    .read_response                  (device_read_response[D9_I2S]       ),
    .write_data                     (device_write_data                  ),
    .write_strobe                   (device_write_strobe                ),
    .write_request                  (device_write_request[D9_I2S]       ),
    .write_response                 (device_write_response[D9_I2S]      ),
    .i2s_bclk                       (i2s_bclk                           ),
    .i2s_lrclk                      (i2s_lrclk                          ),
    .i2s_sdata_out                  (i2s_sdata_out                      ),
    .i2s_sdata_in                   (i2s_sdata_in                       )
  );

  // --------------------------------------------------------------------------
  //  Pin Multiplexer Instance
  // --------------------------------------------------------------------------

  rv5_pinmux rv5_pinmux_instance (
    .clk                            (clock                              ),
    .reset                          (reset                              ),

    // System Bus
    .bus_addr                       (device_rw_address                  ),
    .bus_wdata                      (device_write_data                  ),
    .bus_we                         (&device_write_strobe               ),
    .bus_req                        (device_read_request[D12_PINMUX] | device_write_request[D12_PINMUX]),
    .bus_rdata                      (device_read_data[32*D12_PINMUX +: 32]),
    .bus_ack                        (device_read_response[D12_PINMUX]   ),

    // Physical Pins
    .pin_in                         (pin_in                             ),
    .pin_out                        (pin_out                            ),
    .pin_uio_in                     (pin_uio_in                         ),
    .pin_uio_out                    (pin_uio_out                        ),
    .pin_uio_oe                     (pin_uio_oe                         ),

    // Internal Peripheral Inputs
    .uart0_rx                       (uart_rx                            ),
    .uart1_rx                       (uart1_rx                           ),
    .spi0_miso                      (poci                               ),
    .spi1_miso                      (                                   ),
    .i2s_sdata_in                   (i2s_sdata_in                       ),
    .i2c0_sda_in                    (i2c0_sda_i                         ),
    .gpio_in                        (gpio_input_internal                ),

    // Internal Peripheral Outputs
    .uart0_tx                       (uart_tx                            ),
    .uart1_tx                       (uart1_tx                           ),
    .spi0_mosi                      (pico                               ),
    .spi0_sck                       (sclk                               ),
    .spi0_cs                        (cs[0]                              ),
    .spi1_mosi                      (1'b0                               ),
    .spi1_sck                       (1'b0                               ),
    .spi1_cs                        (1'b0                               ),
    .i2s_sdata_out                  (i2s_sdata_out                      ),
    .i2s_bclk                       (i2s_bclk                           ),
    .i2s_lrclk                      (i2s_lrclk                          ),
    .pwm_out                        (pwm_out                            ),
    .gpio_out                       (gpio_output_internal               ),
    .gpio_oe                        (gpio_oe_internal                   ),
    .i2c0_sda_out                   (i2c0_sda_o                         ),
    .i2c0_scl_out                   (i2c0_scl_o                         ),
    .i2c0_sda_oe                    (i2c0_sda_oe                        ),
    .i2c0_scl_oe                    (i2c0_scl_oe                        ),
    .i2c1_sda_out                   (1'b0                               ),
    .i2c1_scl_out                   (1'b0                               ),
    .i2c1_sda_oe                    (1'b0                               ),
    .i2c1_scl_oe                    (1'b0                               )
  );

  // Pinmux write response = read response (bus_ack drives both)
  assign device_write_response[D12_PINMUX] = device_read_response[D12_PINMUX];

  // --------------------------------------------------------------------------

  // Avoid warnings about intentionally unused pins/wires
  wire unused_ok =
    &{1'b0,
    irq_external,
    irq_software,
    irq_external_response,
    irq_software_response,
    irq_timer_response,
    irq_fast_response[15:2],
    1'b0};



endmodule
