---
title: "GARV32 RISC-V SoC"
author: "Director, CSIR - CEERI (Dr. Gaurav Purohit)"
---

## How it works

The GARV32 is a 32-bit RISC-V System-on-Chip (SoC) designed for the Tiny Tapeout ASIC process. It features a custom rv32i core, an internal 256-byte SRAM, and a dynamic Pin Multiplexer. The SoC fetches instructions from an external QSPI Flash memory (Execute-In-Place) and can map its internal hardware peripherals (UART, SPI, I2C, I2S, PWM, GPIO) dynamically to any of the available physical IO pins via memory-mapped configuration registers.

## How to test

After power-on-reset, the system boots from the external QSPI flash. The user must provide a valid bootloader or compiled RISC-V application (`.bin`) on the external Flash chip connected to the dedicated QSPI pins. To communicate with the peripherals, software must first configure the Pin Mux registers (located at base address `0x800B_0000`) to route the internal peripheral signals to the desired Tiny Tapeout multiplexer pins (`uo_out` and `uio`). Standard testing involves loading a simple UART "Hello World" or LED blink program into the Flash and observing the routed outputs.
