TOP     := tb_stage1
SOURCES := rtl/real_bf.v rtl/stage1.v tb/tb_stage1.v
SNAP    := $(TOP)_sim

.PHONY: all compile elaborate simulate wave clean

all: simulate

compile:
	xvlog $(SOURCES)

elaborate: compile
	xelab $(TOP) -s $(SNAP) -debug typical

simulate: elaborate
	xsim $(SNAP) -R

wave: elaborate
	xsim $(SNAP) -gui

clean:
	rm -rf xsim.dir xelab.pb *.jou *.log *.pb *.wdb .Xil dfx_runtime.txt
