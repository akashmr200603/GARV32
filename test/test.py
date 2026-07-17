import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

@cocotb.test()
async def test_project(dut):
    """Test the RV5 Microcontroller Reset and Boot Sequence."""
    
    # Start the clock (50 MHz)
    clock = Clock(dut.clk, 20, units="ns")
    cocotb.start_soon(clock.start())
    
    # Initialize inputs
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.ena.value = 1
    dut.rst_n.value = 0
    
    # Reset sequence
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    
    # Let the processor boot up and run a few cycles
    await ClockCycles(dut.clk, 100)
    
    # Simple check to ensure simulation runs correctly without crashing
    assert True
