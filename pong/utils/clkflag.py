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

