#!/usr/bin/env python3

from migen import *
from migen.fhdl import *
from migen.genlib.fsm import *
import numpy as np
import random

#fhdl reference:
"""
fsm = FSM(reset_state="RESET")
self.submodules += fsm

fsm.act("RESET",
    self.a.eq(1), #combinational
    If(self.x,
        NextValue(self.y, self.y + 1), #synchronous
        If(self.z,
            NextState("NEW_STATE")
        )
    )
)

self.sync += Case(val, {0 : a.eq(b), "default" : x.eq(y)})
self.sync += c.eq(z)
self.sync += If(foo, i.eq(j)).Else(i.eq(x))
"""

class signal_gen():
    def __init__(self, length, cycles_per_sample):
        self.bits = Array(Constant(random.randrange(2), (1, False)) for _ in range(length))
        self.clk_freq = 6e6
        self.sample_freq = self.clk_freq/cycles_per_sample
        self.baud = 1200.
        self.f1 = 1200.
        self.f0 = 2200.
        self.cycles_per_sample = cycles_per_sample

    def generator(self):
        phase = 0.
        curr_bit = self.bits[0].value
        f = self.f1 if curr_bit == 1 else self.f0
        j = 0
        samples_per_baud = int(self.sample_freq/self.baud)
        for i in range(samples_per_baud*len(self.bits)):
            coeff = self.cycles_per_sample*i*2.*np.pi/self.clk_freq
            if int(i * self.baud / self.sample_freq) > j:
                j = int(i * self.baud / self.sample_freq)
                bit_idx = int(i / samples_per_baud)
                new_bit = self.bits[bit_idx].value
                if new_bit != curr_bit: #determine new frequency and match the phase
                    new_f = self.f1 if new_bit == 1 else self.f0
                    phase = (coeff*(f - new_f)) + phase
                    curr_bit = new_bit
                    f = new_f

            ret = np.cos((coeff*f) + phase)
            yield curr_bit,ret

    def get_cycles_per_baud(self):
        return self.clk_freq/self.baud

class spi_tb(Module):
    def __init__(self, num_bits):
        cycles_per_sample   = num_bits + 3
        gen                 = signal_gen(20, cycles_per_sample)
        cycles_per_baud     = gen.get_cycles_per_baud()
        self.miso           = Signal()
        self.cs             = Signal()
        self.adc_data       = Signal(num_bits)
        samples             = [(b, Constant(int((2**(num_bits - 1))*x)-1, (num_bits,True))) for b,x in gen.generator()]
        self.adc_array      = Array(x[1] for x in samples)
        self.actual_arr     = Array(x[0] for x in samples)
        self.sample_idx     = Signal(int(np.ceil(np.log2(len(self.adc_array)))))
        self.sample_counter = Signal(int(np.ceil(np.log2(cycles_per_sample))))
        self.actual         = Signal()

        self.ios = {self.miso, self.cs}
        self.sync += \
            If(self.sample_counter >= cycles_per_sample,
                self.sample_counter.eq(0),
                self.sample_idx.eq(self.sample_idx + 1)
            ).Else(
                self.sample_counter.eq(self.sample_counter + 1)
            )

        fsm = FSM(reset_state="B-1")
        self.submodules += fsm

        fsm.act("B-1"  , self.miso.eq(1), If(~self.cs, NextState("WAIT0")))
        fsm.act("WAIT0", self.miso.eq(1), NextState("WAIT1")) #first cycle with CS low, Dout HI-Z (with pullup)
        fsm.act("WAIT1", self.miso.eq(1), NextState("WAIT2")) #Dout still HI-Z
        fsm.act("WAIT2", self.miso.eq(0), NextState("B" + str(num_bits - 1)))    #Dout gives "Null BIT" (low) first

        for i in range(num_bits - 1, -1, -1):
            fsm.act("B" + str(i),
                NextState("B" + str(i-1)),
                self.miso.eq(self.adc_data[i]))

        self.sync += If(fsm.ongoing("WAIT0"),
                self.adc_data.eq(self.adc_array[self.sample_idx]),
                self.actual.eq(self.actual_arr[self.sample_idx]))

class adc_spi(Module):
    def __init__(self, num_bits):
        self.miso       = Signal()
        self.cs         = Signal()
        self.sck        = Signal()
        self.data_valid = Signal()
        self.adc_data   = Signal(num_bits)

        self.ios = {self.miso, self.cs, self.data_valid, self.adc_data, self.sck}

        fsm = FSM(reset_state="B-1")
        self.submodules += fsm

        self.comb += self.sck.eq(ClockSignal("sys"))

        self.comb += self.cs.eq(fsm.ongoing("B-1"))
        self.comb += self.data_valid.eq(fsm.ongoing("B-1"))
        fsm.act("B-1"  , NextState("WAIT0"))
        fsm.act("WAIT0", NextState("WAIT1")) #first cycle with CS low, Dout HI-Z
        fsm.act("WAIT1", NextState("WAIT2")) #Dout still HI-Z
        fsm.act("WAIT2", NextState("B" + str(num_bits - 1)))    #Dout gives "Null BIT" (low) first

        for i in range(num_bits - 1, -1, -1):
            fsm.act("B" + str(i),
                NextState("B" + str(i-1)),
                NextValue(self.adc_data[i], self.miso))

class adc_tb_top(Module):
    def __init__(self, num_bits):
        self.data_valid = Signal()
        self.adc_data = Signal(num_bits)

        self.ios = {self.data_valid, self.adc_data}

        tb = spi_tb(num_bits)
        dut = adc_spi(num_bits)
        negedge = ClockDomain()
        self.clock_domains.cd_negedge = negedge
        self.submodules += tb
        self.submodules += dut
        tb = ClockDomainsRenamer("negedge")(tb)

        self.comb += dut.miso.eq(tb.miso)
        self.comb += tb.cs.eq(dut.cs)

        self.comb += negedge.rst.eq(ResetSignal("sys"))
        self.comb += negedge.clk.eq(~dut.sck)

        self.comb += self.data_valid.eq(dut.data_valid)
        self.sync += If(self.data_valid, self.adc_data.eq(dut.adc_data))

tb_top = adc_tb_top(10)
print(verilog.convert(tb_top, tb_top.ios, "adc_tb_top"))

