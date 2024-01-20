from amaranth import *
from pong.common import RX_LAYOUT
from transactron.core import Method, TModule, Transaction, def_method
from transactron.lib.connectors import Forwarder
from transactron.lib.simultaneous import condition

class CRCDiscarder(Elaboratable):
    def __init__(self, method_in: Method):
        self.method_in = method_in
        self.method_out = Method(o=RX_LAYOUT)
        
        self.buff0 = Record(RX_LAYOUT)
        self.buff1 = Record(RX_LAYOUT)
        self.buff2 = Record(RX_LAYOUT)
        self.buff3 = Record(RX_LAYOUT)

        self.fwd = Forwarder(RX_LAYOUT)

    def elaborate(self, platform):
        m = TModule()
        m.submodules.fwd = self.fwd

        vpos = Signal(range(5))
            
        @def_method(m, self.method_out)
        def _():
            return self.fwd.read(m)
        
        with Transaction().body(m):
            input = self.method_in(m)

#            with condition(m, priority=False) as c1:
#                with c1(input.end):
#                    m.d.sync += vpos.eq(0)
#                    self.fwd.write(m, input) 
#                with c1(~input.end):
#                    for i in range(3):
#                        m.d.sync += self.buff[i+1].eq(self.buff[i])
#                    m.d.sync += self.buff[0].eq(input)
#
#                    with condition(m, priority=False) as cond:
#                        with cond(vpos == 3):
#                            self.fwd.write(m, self.buff[3])
#                        with cond(vpos != 3):
            with m.If(input.end):
                m.d.sync += vpos.eq(0)
                self.fwd.write(m, input) 
            with m.Else():
                m.d.sync += self.buff0.eq(input)
                m.d.sync += self.buff1.eq(self.buff0)
                m.d.sync += self.buff2.eq(self.buff1)
                m.d.sync += self.buff3.eq(self.buff2)
                #for i in range(3):
                #    m.d.sync += self.buff[i+1].eq(self.buff[i])

                with m.If(vpos == 4):
                    self.fwd.write(m, self.buff3)
                with m.Else():
                    m.d.sync += vpos.eq(vpos+1)
        return m
