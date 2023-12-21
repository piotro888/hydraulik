from amaranth import *

from transactron.core import TModule, Transaction 

from pong.eth.mdio import PHY_ETH0, REG_1000T_CTRL, REG_AUTO_NEG_ADV, REG_EXT_ADDR, MDIOController

class Pong(Elaboratable):
    def __init__(self):
        self.mdio = MDIOController()
    def elaborate(self, platform):
        m = TModule()

        m.submodules.mdio = self.mdio 
        if platform is not None:
            enet = platform.request("enet")
            m.d.comb += enet.mdc.eq(self.mdio.o_mdc)
            m.d.comb += enet.mdio.o.eq(self.mdio.o_mdio)
            m.d.comb += enet.mdio.oe.eq(1)#self.mdio.oe_mdio)
            m.d.comb += enet.rst.eq(ResetSignal())
        
        init = Signal(reset=1)
        with m.FSM("START"):
            with m.State("START"):
                with Transaction().body(m):
                    self.mdio.write(m, addr=PHY_ETH0, reg=REG_1000T_CTRL, data=0) # disable 1 Gig negotiation
                    #self.mdio.write(m, addr=PHY_ETH0, reg=25, data=0xffff) # LED OVERRIDE
                    m.next = "INIT1"
            with m.State("INIT1"):
                with Transaction().body(m):
                    #self.mdio.write(m, addr=PHY_ETH0, reg=REG_AUTO_NEG_ADV, data=(1<<7)|(1<<8)) # enable only 100 BASET negotiation
                    m.next = "INIT2"
            with m.State("INIT2"):
                with Transaction().body(m):
                    #self.mdio.write(m, addr=PHY_ETH0, reg=25, data=0xff) # LED OVERRIDE
                    m.next = "INIT_RESET"
            with m.State("INIT_RESET"):
                with Transaction().body(m):
                    #self.mdio.write(m, addr=PHY_ETH0, reg=0, data=(1<<15)) # soft reset 
                    m.next = "END"
            with m.State("END"):
                m.d.sync += init.eq(0)

        if platform is not None:
            m.d.comb += platform.request("led_r").eq(1)

        return m
