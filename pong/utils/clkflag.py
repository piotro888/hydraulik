from amaranth import *

class ClkFlag(Elaboratable):
    def __init__(self, div: int):
        self.tick = Signal()
        self.div = div
        self.counter = Signal(div)

    def elaborate(self, platform):
        m = Module()

        m.d.sync += self.counter.eq(self.counter + 1)
        m.d.comb += self.tick.eq(self.counter == 0)
        
        return m

CLK_FREQ = 50_000_000
class ClkFlagCnt(Elaboratable):
    def  __init__(self, freq: int):
        self.tick = Signal()
        
        self.count = int(round(CLK_FREQ/freq))

    def elaborate(self, platform):
        m = Module()

        counter = Signal(range(self.count))
        
        with m.If(counter == self.count-1):
            m.d.sync += counter.eq(0)
            m.d.comb += self.tick.eq(1)
        with m.Else():
            m.d.sync += counter.eq(counter+1)

        return m
