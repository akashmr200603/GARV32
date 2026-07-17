# Makefile for Cocotb testing
PWD = $(shell pwd)
export PYTHONPATH := $(PWD)/../model:$(PYTHONPATH)
export COCOTB_REDUCED_LOG_FMT=1

# Hardware language and top level module
TOPLEVEL_LANG = verilog
TOPLEVEL = tb
MODULE = test

# Source files
VERILOG_SOURCES = $(PWD)/tb.v $(PWD)/../src/*.v

include $(shell cocotb-config --makefiles)/Makefile.sim
