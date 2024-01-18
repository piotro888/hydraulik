from amaranth import *
from pong.utils.clkflag import ClkFlagCnt

from transactron.core import Method, TModule, def_method

class Serial(Elaboratable):
    def __init__(self):
        self.o_tx = Signal()
        
        self.tx = Method(i=[("data", 8)])

    def elaborate(self, platform):
        m = TModule()
        
        m.submodules.clkf = clkf = ClkFlagCnt(115200)

        busy = Signal()
        byte = Signal(8)
        bit = Signal(range(8))
        
        with m.FSM("idle"):        
            with m.State("idle"):
                m.d.comb += self.o_tx.eq(1)
                
                with m.If(busy & clkf.tick):
                    m.next = "start"
            
            with m.State("start"):
                m.d.comb += self.o_tx.eq(0) # start bit

                with m.If(clkf.tick):
                    m.d.sync += bit.eq(0)
                    m.next = "data"

            with m.State("data"):
                m.d.comb += self.o_tx.eq(byte.bit_select(bit, 1))

                with m.If(clkf.tick):
                    m.d.sync += bit.eq(bit + 1)
                    with m.If(bit == 7):
                        m.next = "stop"
            
            with m.State("stop"):
                m.d.comb += self.o_tx.eq(1)
                
                with m.If(clkf.tick):
                    m.d.sync += bit.eq(bit + 1)
                    
                    with m.If(bit == 1):
                        m.d.sync += busy.eq(0)
                        m.next = "idle"

        @def_method(m, self.tx, ready=~busy)
        def _(data):
            m.d.sync += byte.eq(data)
            m.d.sync += busy.eq(1)

        return  m
    
