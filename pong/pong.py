from amaranth import *

from transactron.core import TModule, Transaction 

from pong.eth.mdio import MDIOController

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
            m.d.comb += enet.mdio.oe.eq(1)
        
        init = Signal()
        with Transaction(name = "init").body(m, request=~init):
            self.mdio.write(m, addr=0b10000, reg=25, data=2**10-1) # LED OVERRIDE TEST
            m.d.sync += init.eq(1)
        
        if platform is not None:
            m.d.comb += platform.request("led_r").eq(1)

        return m
