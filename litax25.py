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
    def __init__(self, length, params):
        self.bits = Array(Constant(random.randrange(2), (1, False)) for _ in range(length))
        self.params = params

    def generator(self):
        phase = 0.
        curr_bit = self.bits[0].value
        f = self.params.f1 if curr_bit == 1 else self.params.f0
        j = 0
        for i in range(self.params.presamples_per_symbol*len(self.bits)):
            coeff = self.params.cycles_per_sample*i*2.*np.pi/self.params.clk_freq
            if int(i * self.params.baud / self.params.sample_freq) > j:
                j = int(i * self.params.baud / self.params.sample_freq)
                bit_idx = int(i / self.params.presamples_per_symbol)
                new_bit = self.bits[bit_idx].value
                if new_bit != curr_bit: #determine new frequency and match the phase
                    new_f = self.params.f1 if new_bit == 1 else self.params.f0
                    phase = (coeff*(f - new_f)) + phase
                    curr_bit = new_bit
                    f = new_f

            ret = np.cos((coeff*f) + phase)
            yield curr_bit,ret

    def get_cycles_per_baud(self):
        return self.clk_freq/self.baud

class spi_tb(Module):
    def __init__(self, params):
        gen                 = signal_gen(20, params)
        self.miso           = Signal()
        self.cs             = Signal()
        self.adc_data       = Signal(params.adc_bit_width)
        samples             = [(b, Constant(int((2**(params.adc_bit_width - 1))*x)-1, (params.adc_bit_width,True))) for b,x in gen.generator()]
        self.adc_array      = Array(x[1] for x in samples)
        self.actual_arr     = Array(x[0] for x in samples)
        self.sample_idx     = Signal(int(np.ceil(np.log2(len(self.adc_array)))))
        self.sample_counter = Signal(int(np.ceil(np.log2(params.cycles_per_sample))))
        self.actual         = Signal()

        self.ios = {self.miso, self.cs}
        self.sync += \
            If(self.sample_counter >= params.cycles_per_sample,
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
        fsm.act("WAIT2", self.miso.eq(0), NextState("B" + str(params.adc_bit_width - 1)))    #Dout gives "Null BIT" (low) first

        for i in range(params.adc_bit_width - 1, -1, -1):
            fsm.act("B" + str(i),
                NextState("B" + str(i-1)),
                self.miso.eq(self.adc_data[i]))

        self.sync += If(fsm.ongoing("WAIT0"),
                self.adc_data.eq(self.adc_array[self.sample_idx]),
                self.actual.eq(self.actual_arr[self.sample_idx]))

class adc_spi(Module):
    def __init__(self, params):
        self.miso       = Signal()
        self.cs         = Signal()
        self.sck        = Signal()
        self.data_valid = Signal()
        self.adc_data   = Signal(params.adc_bit_width)

        self.ios = {self.miso, self.cs, self.data_valid, self.adc_data, self.sck}

        fsm = FSM(reset_state="B-1")
        self.submodules += fsm

        self.comb += self.sck.eq(ClockSignal("sys"))

        self.comb += self.cs.eq(fsm.ongoing("B-1"))
        self.comb += self.data_valid.eq(fsm.ongoing("B-1"))
        fsm.act("B-1"  , NextState("WAIT0"))
        fsm.act("WAIT0", NextState("WAIT1")) #first cycle with CS low, Dout HI-Z
        fsm.act("WAIT1", NextState("WAIT2")) #Dout still HI-Z
        fsm.act("WAIT2", NextState("B" + str(params.adc_bit_width - 1)))    #Dout gives "Null BIT" (low) first

        for i in range(params.adc_bit_width - 1, -1, -1):
            fsm.act("B" + str(i),
                NextState("B" + str(i-1)),
                NextValue(self.adc_data[i], self.miso))

class downsample(Module):
    def __init__(self, params):
        self.data_in    = Signal((params.adc_bit_width, True))
        self.valid_in   = Signal()
        self.data_out   = Signal((params.downsampled_width, True))
        self.valid_out  = Signal()
        self.counter    = Signal(int(np.ceil(np.log2(params.downsample_count))))
        self.acc        = Signal((params.downsampled_width, True))

        self.ios = {self.data_in, self.valid_in, self.data_out, self.valid_out}

        self.sync += If(self.valid_in,
            If(self.counter >= params.downsample_count,
                self.counter.eq(0),
                self.valid_out.eq(1),
                self.data_out.eq(self.acc + self.data_in),
                self.acc.eq(0)
            ).Else(
                self.counter.eq(self.counter + 1),
                self.valid_out.eq(0),
                self.acc.eq(self.acc + self.data_in)
            )
        ).Else(
            self.valid_out.eq(0)
        )

class sample_queue(Module):
    def __init__(self, params):
        self.valid_in   = Signal()
        self.data_in    = Signal(params.ds_truncated_width)
        self.data_out   = Signal(params.ds_truncated_width)
        self.queue_data = Array(Signal(params.ds_truncated_width) for _ in range(params.samples_per_symbol))
        self.tail       = Signal(int(np.ceil(np.log2(params.samples_per_symbol))))
        self.read_idx   = Signal(int(np.ceil(np.log2(params.samples_per_symbol))))

        self.ios = {self.read_idx, self.valid_in, self.data_in, self.data_out, self.tail}

        self.comb += self.data_out.eq(self.queue_data[self.read_idx])
        self.sync += If(self.valid_in,
            self.queue_data[self.tail].eq(self.data_in),
            If(self.tail >= params.samples_per_symbol,
                self.tail.eq(0)
            ).Else(
                self.tail.eq(self.tail + 1)
            )
        )


class demodulator(Module):
    def __init__(self, params):
        self.miso       = Signal()
        self.cs         = Signal()
        self.sck        = Signal()
        #self.valid_out  = Signal()
        #self.data_out   = Signal(params.downsampled_width)
        self.data_out   = Signal(params.ds_truncated_width)
        self.queue_idx  = Signal(int(np.ceil(np.log2(params.samples_per_symbol))))

        self.ios = {self.miso, self.cs, self.sck, self.data_out}

        #ADC/SPI controller
        adc_spi_interface = adc_spi(params)
        self.submodules += adc_spi_interface

        #connect ADC/SPI controller to our SPI signals
        self.comb += adc_spi_interface.miso.eq(self.miso)
        self.comb += self.cs.eq(adc_spi_interface.cs)
        self.comb += self.sck.eq(adc_spi_interface.sck)

        #the ADC gives us samples too fast - don't want to instantiate multipliers to process all of them
        downsampler = downsample(params)
        self.submodules += downsampler

        #connect downsampler to ADC/SPI controller
        #TODO there should be a CDC here - everything from downsampler on should be running on sys clock not ADC clock
        self.comb += downsampler.valid_in.eq(adc_spi_interface.data_valid)
        self.comb += downsampler.data_in.eq(adc_spi_interface.adc_data)

        queue = sample_queue(params)
        self.submodules += queue

        self.comb += queue.valid_in.eq(downsampler.valid_out)
        self.comb += queue.data_in.eq(downsampler.data_out[params.downsampled_width - params.ds_truncated_width:])
        self.comb += queue.read_idx.eq(self.queue_idx)

        #self.comb += self.valid_out.eq(downsampler.valid_out)
        #self.comb += self.data_out.eq(downsampler.data_out)

        self.comb += self.data_out.eq(queue.data_out)
        self.sync += If(self.queue_idx >= params.samples_per_symbol,
            self.queue_idx.eq(0)
        ).Else(
            self.queue_idx.eq(self.queue_idx + 1)
        )

class adc_tb_top(Module):
    def __init__(self, params):
        #self.data_valid = Signal()
        self.data_out   = Signal(params.downsampled_width)
        
        self.ios = {self.data_out}

        #Design Under Test - talk to TB ADC model and demodulate the signal
        dut = demodulator(params)
        self.submodules += dut

        #create a negedge clock domain for the TB because the real MCP3001 responds on the negative edge
        negedge = ClockDomain()
        self.clock_domains.cd_negedge = negedge
        self.comb += negedge.clk.eq(~dut.sck)
        self.comb += negedge.rst.eq(ResetSignal("sys"))

        #TB model of the ADC with SPI interface
        tb = spi_tb(params)
        self.submodules += tb
        tb = ClockDomainsRenamer("negedge")(tb)

        #DUT/TB connections
        self.comb += dut.miso.eq(tb.miso) #data from ADC TB model to the DSP pipeline
        self.comb += tb.cs.eq(dut.cs) #chip select for the ADC TB model

        #DUT/top level connections - just for test
        #self.comb += self.data_valid.eq(dut.valid_out)
        #self.sync += If(dut.valid_out, self.data_out.eq(dut.data_out))
        self.comb += self.data_out.eq(dut.data_out)

class dsp_params:
    def __init__(self):
        #bit width of the ADC
        self.adc_bit_width = 10

        #function of the ADC SPI interface - how many cycles does it take to read each sample
        self.cycles_per_sample = self.adc_bit_width + 3 

        #orange crab/litex/ECP5 system clock is set to 48MHz, the SPI signals looked a little
        #stressed on the oscilloscope at 12MHz but clean at 6MHz so lets divide by 8
        self.clk_freq = 6e6

        #how often are we getting a sample in?
        self.sample_freq = self.clk_freq/self.cycles_per_sample

        #AX.25 baud rate, Hz
        self.baud = 1200.

        #how many samples (pre-downsampling) are we going to collect for each sample
        self.presamples_per_symbol = int(self.sample_freq/self.baud)

        #after downsampling, how many samples do we want to have for each bit symbol?
        self.samples_per_symbol = 8

        #how many samples are we going to average together when we downsample?
        self.downsample_count = int(self.sample_freq/(self.samples_per_symbol*self.baud))

        #how many bits do we need to represent the downsampled signal - we don't actually need to
        #calculate the quotient to get the average of the last N samples because the amplitude is
        #arbitrary - but we do need enough bits to represent the sum of the last N samples
        self.downsampled_width = int(np.ceil(np.log2(self.downsample_count))) + self.adc_bit_width

        #let's truncate some of the less significant bits after downsampling - they're mostly noise
        #and extra bits here costs extra logic or delay in the multipliers
        self.ds_truncated_width = 8

        #AX.25 frequencies for bit 1/bit 0 symbols, Hz
        self.f1 = 1200.
        self.f0 = 2200.

params = dsp_params()
tb_top = adc_tb_top(params)
print(verilog.convert(tb_top, tb_top.ios, "adc_tb_top"))

