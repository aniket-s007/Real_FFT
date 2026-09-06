# Usage:
#   make                  -> build & run tb_stage1 (default)
#   make STAGE=2          -> build & run tb_stage2
#   make STAGE=3          -> build & run tb_stage3
#   make STAGE=top12      -> build & run tb_top_stage1_stage2 (stage1->stage2
#                             structural link, top_stage1_stage2.v)
#   make STAGE=top123     -> build & run tb_top_stage1_stage2_stage3
#                             (stage1->stage2->stage3 structural link)
#   make STAGE=timed      -> build & run tb_stage1_stage2_timed (simple
#                             explicit-timeline stage1->stage2 testbench)
#   make STAGE=2 wave     -> open tb_stage2 in the xsim GUI
#   make clean
#
# WIDTH sweeping for the later SQNR study: xelab's documented
# "-generic_top WIDTH=N" override mis-splits into two separate arguments
# on this Vivado 2025.1 Windows install regardless of quoting (verified
# directly, not just in this Makefile) and errors out, so it's not wired
# in here. For now, edit the `parameter WIDTH = 16;` line directly in the
# relevant tb_stageN.v to sweep it.

STAGE     ?= 3
BUILD_DIR := log

# So xvlog/xelab/xsim resolve even when Vivado's bin dir isn't on the
# system/terminal PATH -- prepend it here rather than relying on shell setup.
VIVADO_BIN  := F:\Vivado\2025.1\Vivado\bin
export PATH := $(VIVADO_BIN);$(PATH)

SOURCES_1    := ../rtl/real_bf.v ../rtl/stage1.v ../tb/tb_stage1.v
SOURCES_2    := ../rtl/real_bf.v ../rtl/rotator.v ../rtl/stage1.v ../rtl/stage2.v ../tb/tb_stage2.v
SOURCES_3    := ../rtl/real_bf.v ../rtl/rotator.v ../rtl/switch1.v ../rtl/stage1.v ../rtl/stage2.v ../rtl/stage3.v ../tb/tb_stage3.v
SOURCES_top12 := ../rtl/real_bf.v ../rtl/rotator.v ../rtl/stage1.v ../rtl/stage2.v ../rtl/top_stage1_stage2.v ../tb/tb_top_stage1_stage2.v
SOURCES_top123 := ../rtl/real_bf.v ../rtl/rotator.v ../rtl/switch1.v ../rtl/stage1.v ../rtl/stage2.v ../rtl/stage3.v ../rtl/top_stage1_stage2.v ../rtl/top_stage1_stage2_stage3.v ../tb/tb_top_stage1_stage2_stage3.v
SOURCES_timed := ../rtl/real_bf.v ../rtl/rotator.v ../rtl/stage1.v ../rtl/stage2.v ../tb/tb_stage1_stage2_timed.v

TOP     := tb_stage$(STAGE)
ifeq ($(STAGE),top12)
TOP     := tb_top_stage1_stage2
endif
ifeq ($(STAGE),top123)
TOP     := tb_top_stage1_stage2_stage3
endif
ifeq ($(STAGE),timed)
TOP     := tb_stage1_stage2_timed
endif
SOURCES := $(SOURCES_$(STAGE))
SNAP    := $(TOP)_sim

.PHONY: all compile elaborate simulate wave clean

all: simulate

compile:
	-mkdir $(BUILD_DIR)
	cd $(BUILD_DIR) && xvlog $(SOURCES)

elaborate: compile
	cd $(BUILD_DIR) && xelab $(TOP) -s $(SNAP) -debug typical

simulate: elaborate
	cd $(BUILD_DIR) && xsim $(SNAP) -R

wave: elaborate
	cd $(BUILD_DIR) && xsim $(SNAP) -gui

clean:
	-rmdir /s /q $(BUILD_DIR)
