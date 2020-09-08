#!/usr/bin/env bash

set -e

./litax25.py > adc_tb_top.sv
rm -rf obj_dir
verilator --exe --cc main.cpp adc_tb_top.sv --trace-underscore -Wno-fatal --x-initial 0 --trace --report-unoptflat
make -j 16 -C obj_dir/ -f Vadc_tb_top.mk
./obj_dir/Vadc_tb_top

