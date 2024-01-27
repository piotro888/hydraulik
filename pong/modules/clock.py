from amaranth import *

from transactron.core import TModule

class PLL(Elaboratable):
    def __init__(self, target=50_000_000) -> None:
        self.pps = Signal()
        self.target = target

        self.frac = Signal(32)
        self.prec = Signal(32)
        self.increment = Signal()
        self.refs = Signal()

        self.rt = Signal(20)
        self.err_log = Signal(16)

    def elaborate(self, platform):
        m = TModule()

        if platform:
            pin = platform.request("pps", 0) 
            m.d.comb += self.pps.eq(pin.pps)

        count = Signal(range(2*self.target))
        rate = Signal(range(2*self.target), reset=self.target)
        

        wait_init = Signal(reset=1)
        initial_sync = Signal(reset=1)
        
        pps_tick = Signal()
        prev_pps = Signal()
        ppsr = Signal(range(10))
        m.d.sync += prev_pps.eq(self.pps)
        m.d.comb += pps_tick.eq(~prev_pps & self.pps)
        
        frac_ticks = Signal(64)
        current_frac = Signal(64)
        last_err = Signal(33)

        m.d.sync += frac_ticks.eq(frac_ticks+current_frac)
        
        with m.If(count < rate):
            m.d.sync += count.eq(count+1)
        with m.Else():
            m.d.sync += count.eq(0)
            m.d.sync += frac_ticks.eq(0)
            m.d.comb += self.increment.eq(1)
        
        with m.If((wait_init | initial_sync) & pps_tick):
            m.d.sync += count.eq(0)
            m.d.sync += wait_init.eq(0)

        corr_rate = Signal(32, reset=1)
        

        m.submodules.bf1 = bfminus = BinFracCalc()
        m.submodules.bf2 = bfplus = BinFracCalc()
        m.submodules.bfi = bfi = BinFracCalc()
        m.d.comb += bfminus.divisor.eq(rate-corr_rate)
        m.d.comb += bfplus.divisor.eq(rate+corr_rate)
        m.d.comb += bfi.divisor.eq(rate)
        
        with m.If(wait_init):
            m.d.sync += current_frac.eq(bfi.result)

        with m.If(pps_tick):
            m.d.sync += ppsr.eq(ppsr+1)
            m.d.comb += self.refs.eq(1)
            with m.If(count == 0):
                m.d.sync += last_err.eq(0)
            with m.Elif(count <= (rate>>1)): # internal too fast
                m.d.sync += rate.eq(rate+corr_rate)
                m.d.sync += current_frac.eq(bfplus.result)
                m.d.sync += last_err.eq(count)
            with m.Else():
                m.d.sync += rate.eq(rate-corr_rate)
                m.d.sync += current_frac.eq(bfminus.result)
                m.d.sync += last_err.eq(rate-count)

        
        m.d.comb += self.frac.eq(frac_ticks>>32)
        m.d.comb += self.prec.eq((last_err*bfi.result)>>32)

        m.d.comb += self.rt.eq(rate)
        
        m.d.sync += self.err_log.eq(0)
        for i in range(32):
            with m.If(last_err.bit_select(i, 1)):
                m.d.sync += self.err_log.eq(i+1)
        
        with m.If(initial_sync & (last_err != 0) & (last_err < 200) & (ppsr > 4)):
            m.d.sync += initial_sync.eq(0)

        with m.If(initial_sync):
            m.d.comb += corr_rate.eq(Mux((last_err>>2).any(), last_err>>2, 1))

        return m
        

        

class BinFracCalc(Elaboratable):
    def __init__(self):
        self.divisor = Signal(32)
        self.result = Signal(64)

    def elaborate(self, plarform):
        m = TModule()
        bit = Signal(range(65), reset=64)
        current = Signal(34)
        op_divisor = Signal(32)
        op_result = Signal(64)

        with m.If(bit == 64):
            m.d.sync += self.result.eq(op_result[::-1])
            m.d.sync += op_divisor.eq(self.divisor)
            m.d.sync += op_result.eq(0)
            m.d.sync += current.eq(0b10)
            m.d.sync += bit.eq(0) 
        with m.Else():
            with m.If(current >=  op_divisor):
                m.d.sync += op_result.bit_select(bit, 1).eq(1)
                m.d.sync += current.eq((current-op_divisor)<<1)
            with m.Else():
                m.d.sync += current.eq(current<<1)
            m.d.sync += bit.eq(bit+1)

        return m

        

