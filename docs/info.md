---
title: "GARV32 RISC-V SoC"
author: "Director, CSIR - CEERI (Dr. Gaurav Purohit)"
---

## How it works

The GARV32 is a 32-bit RISC-V System-on-Chip (SoC) designed for the Tiny Tapeout ASIC process. It features a custom rv32i core, an internal 64-byte ultra-fast SRAM (for stack), and a 16-byte Instruction Cache. The SoC fetches instructions from an external QSPI Flash memory (Execute-In-Place) and routes heavy data segments (like .data and .bss) to an external QSPI PSRAM. To meet area constraints, all internal hardware peripherals (2 UART, 1 SPI, 1 I2C, 1 PWM, 4 GPIO) are hardwired directly to the available physical IO pins on the Tiny Tapeout boundary.

## How to test

After power-on-reset, the system boots from the external QSPI flash. The user must provide a valid bootloader or compiled RISC-V application (`.bin`) on the external Flash chip connected to the dedicated QSPI pins, and connect an external PSRAM chip for heavy variable storage. Standard testing involves loading a simple UART "Hello World" or LED blink program into the Flash and observing the routed outputs on the `uo_out` and `uio` pins.
