from amaranth import *

from pong.utils.clkflag import ClkFlag

from transactron import *

PHY_ETH0 = 0b10000
REG_LED_OVERRIDE = 25

MDIO_WRITE_LAYOUT = [("addr", 5), ("reg", 5), ("data", 16)]

class MDIOController(Elaboratable):
    def __init__(self):
        self.o_mdc = Signal()
        self.o_mdio = Signal()
        
        self.write = Method(i=MDIO_WRITE_LAYOUT)

    def elaborate(self, platform):
        m = TModule()
        
        tx_word = Signal(32)
        tx_cnt = Signal(range(32+1))

        active = Signal()
        
        m.submodules.clkdiv = clkdiv = ClkFlag(4) # 3.125 MHz
        with m.If(clkdiv.tick & active):
            m.d.sync += self.o_mdc.eq(~self.o_mdc)
            
            with m.If(self.o_mdc):
                idx = (31-tx_cnt).as_unsigned()
                m.d.sync += self.o_mdio.eq(Mux(tx_cnt == 32, 0, (tx_word >> idx) & 1))
                m.d.sync += tx_cnt.eq(tx_cnt + 1)
                
                with m.If(tx_cnt == 32):
                    m.d.sync += active.eq(0)
        
        @def_method(m, self.write, ready=~active)
        def _(addr, reg, data):
            m.d.sync += tx_word.eq(Cat(data, C(0), C(1), reg, addr, C(0b01, 2), C(0b01, 2)))
            m.d.sync += active.eq(1)

        return m
