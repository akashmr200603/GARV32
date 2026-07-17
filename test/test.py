import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

@cocotb.test()
async def test_project(dut):
    dut._log.info("Starting GARV32 RISC-V SoC Test")

    # Set the clock period to 20 ns (50 MHz) as defined in info.yaml
    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())

    # Initialize inputs
    dut._log.info("Asserting Reset")
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0

    # Wait 10 cycles in reset
    await ClockCycles(dut.clk, 10)

    # uo_out[2] is QSPI Flash CS (active low). It should be HIGH (1) during reset.
    # uo_out[3] is QSPI PSRAM CS (active low). It should be HIGH (1) during reset.
    assert (int(dut.uo_out.value) & 0x04) != 0, "QSPI Flash CS should be inactive (HIGH) during reset"
    assert (int(dut.uo_out.value) & 0x08) != 0, "QSPI PSRAM CS should be inactive (HIGH) during reset"

    # Release Reset
    dut._log.info("Releasing Reset")
    dut.rst_n.value = 1

    # The rv5_por (Power-On-Reset) synchronizer takes 4 clock cycles to clear internal reset.
    # After that, the RV32I core will attempt to fetch its first instruction from 0x0000_0000 (ROM/Flash).
    # This will cause the QSPI controller to pull the Flash CS pin LOW.
    dut._log.info("Waiting for core to boot and access Flash...")
    await ClockCycles(dut.clk, 20)

    # Verify that the SoC is now actively trying to read from the QSPI Flash
    # uo_out[2] (Flash CS) should now be LOW (0)
    # uo_out[3] (PSRAM CS) should remain HIGH (1)
    
    assert (int(dut.uo_out.value) & 0x04) == 0, "GARV32 failed to assert QSPI Flash CS for boot!"
    assert (int(dut.uo_out.value) & 0x08) != 0, "GARV32 erroneously asserted PSRAM CS during boot!"
    
    dut._log.info("Boot sequence verified successfully! CPU is actively fetching from QSPI Flash.")
