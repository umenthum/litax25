
#include <verilated.h>
#include <verilated_vcd_c.h>
#include "Vadc_tb_top.h"

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Vadc_tb_top* top = new Vadc_tb_top;
	Verilated::traceEverOn(true);
    VerilatedVcdC* tfp = new VerilatedVcdC;
	top->trace(tfp, 99);
	tfp->open("logs/vlt_dump.vcd");
    
    top->sys_clk = 0;
    top->sys_rst = 1;
    int t = 0;

    while ((!Verilated::gotFinish()) && (t < (2*20*6e6/1200))) {
        if (t == 5) top->sys_rst = 0;
        top->sys_clk = !top->sys_clk;
        top->eval();
		t++;
        tfp->dump(t);
    }

    tfp->close();
    tfp = NULL;
    top->final();
    delete top;
    top = NULL;
    exit(0);
}

