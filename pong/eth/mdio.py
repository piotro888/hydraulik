from amaranth import *

from pong.utils.clkflag import ClkFlag

from transactron import *

PHY_ETH0 = 0b10000
REG_LED_OVERRIDE = 25
REG_AUTO_NEG_ADV = 4
REG_EXT_ADDR = 22
REG_1000T_CTRL = 9

MDIO_WRITE_LAYOUT = [("addr", 5), ("reg", 5), ("data", 16)]

class MDIOController(Elaboratable):
    def __init__(self):
        self.o_mdc = Signal()
        self.o_mdio = Signal(reset=1)
        self.oe_mdio = Signal(reset=0)
        
        self.write = Method(i=MDIO_WRITE_LAYOUT)

    def elaborate(self, platform):
        m = TModule()
        
        m.submodules.clkdiv = clkdiv = ClkFlag(6) 
        
        tx_word = Signal(32)

        bit = Signal(range(32))
        
        with m.FSM("idle") as fsm:
            with m.State("idle"):
                m.d.sync += self.o_mdc.eq(0)
                m.d.sync += self.oe_mdio.eq(0)
                with m.If(self.write.run):
                    m.next = "preamble"
            with m.State("preamble"):
                with m.If(clkdiv.tick):
                    m.d.sync += self.o_mdc.eq(~self.o_mdc)
                    m.d.sync += self.oe_mdio.eq(1)
                    m.d.sync += self.o_mdio.eq(1)
                    with m.If(self.o_mdc):
                        m.d.sync += bit.eq(bit+1)
                        with m.If(bit == 31):
                            m.next = "data"
            with m.State("data"):
                with m.If(clkdiv.tick):
                    m.d.sync += self.o_mdc.eq(~self.o_mdc)
                    with m.If(self.o_mdc):
                        m.d.sync += bit.eq(bit+1)
                        m.d.sync += self.o_mdio.eq(tx_word.bit_select((31-bit).as_unsigned(), 1))
                        with m.If(bit == 31):
                            m.next = "lastbit"
            with m.State("lastbit"):
                with m.If(clkdiv.tick):
                    m.d.sync += self.o_mdc.eq(~self.o_mdc)
                    m.next = "gap"
            with m.State("gap"):
                with m.If(clkdiv.tick):
                    m.d.sync += self.o_mdc.eq(0)
                    m.d.sync += self.o_mdc.eq(1)
                    m.d.sync += self.oe_mdio.eq(1)
                    m.d.sync += self.o_mdio.eq(1)
                    m.d.sync += bit.eq(bit+1)
                    with m.If(bit == 31):
                        m.next = "idle"


        @def_method(m, self.write, ready=fsm.ongoing("idle"))
        def _(addr, reg, data):
            m.d.sync += tx_word.eq(Cat(data, C(0), C(1), reg, addr, C(0b01, 2), C(0b01, 2)))

        return m
