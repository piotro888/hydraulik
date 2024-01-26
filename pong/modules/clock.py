from amaranth import *

from transactron.core import TModule

class PLL(Elaboratable):
    def __init__(self, target=50_000_000) -> None:
        self.pps = Signal()
        self.target = target

    def elaborate(self, platform):
        m = TModule()

        count = Signal(range(2*self.target))
        rate = Signal(range(2*self.target), reset=self.target)
        

        wait_init = Signal()
        
        pps_tick = Signal()
        prev_pps = Signal()
        m.d.sync += prev_pps.eq(self.pps)
        m.d.comb += pps_tick.eq(~prev_pps & self.pps)

        with m.If(count < rate):
            m.d.sync += count.eq(count+1)
        with m.Else():
            m.d.sync += count.eq(0)
        
        with m.If(wait_init & pps_tick):
            m.d.sync += count.eq(0)

        with m.If(pps_tick):
            with m.If(count == 0):
                pass
            with m.Elif(count < (rate>>1)): # internal too fast
                m.d.sync += rate.eq(rate+1)
            with m.Else():
                m.d.sync += rate.eq(rate-1)

        frac_part = Signal(128)
        frac_bit = Signal(range(128))
        frac_div = Signal(2*self.target)
        m.d.sync += 
        

        return m
        

        

        
