from amaranth import *
from amaranth.hdl.ast import SignalSet
from amaranth.lib.io import FlippedInterface
from pong.eth.frontend import EtherentInterface
from pong.frontend.front import Receiver

from transactron.core import TModule, Transaction 

from pong.eth.mdio import PHY_ETH0, REG_1000T_CTRL, MDIOController
from transactron.lib.connectors import ConnectTrans

class Pong(Elaboratable):
    def __init__(self):
        self.mdio = MDIOController()
        self.ethi = EtherentInterface()

    def elaborate(self, platform):
        m = TModule()

        m.submodules.mdio = self.mdio 
        m.submodules.eth0_i = ethi = self.ethi
        
        if platform is not None:
            enet = platform.request("enet")
            m.d.comb += enet.mdc.eq(self.mdio.o_mdc)
            m.d.comb += enet.mdio.o.eq(self.mdio.o_mdio)
            m.d.comb += enet.mdio.oe.eq(1)#self.mdio.oe_mdio)
            m.d.comb += enet.rst.eq(ResetSignal())
            m.d.comb += ethi.i_rx_clk.eq(enet.rx_clk)
            m.d.comb += ethi.i_rx_data.eq(enet.rx_data)
            m.d.comb += ethi.i_rx_valid.eq(enet.rx_dv)
            m.d.comb += enet.gtx_clk.eq(ethi.o_tx_clk)
            m.d.comb += enet.tx_data.eq(ethi.o_tx_data)
            m.d.comb += enet.tx_en.eq(ethi.o_tx_valid)
        
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
                    m.next = "INIT_RESET"
            with m.State("INIT_RESET"):
                with Transaction().body(m):
                    self.mdio.write(m, addr=PHY_ETH0, reg=0, data=(1<<15)) # soft reset 
                    m.next = "END"
            with m.State("END"):
                m.d.sync += init.eq(0)

        if platform is not None:
            leds = platform.request("led_r")
            ledsg = platform.request("led_g")
            bt = platform.request("button")
            #m.d.comb += leds.eq(ethi.rx_fifo.r_level)
            dset = Signal()
            sset = Signal()
            ttet = Signal()
            ledsd = Signal(16)
            with Transaction().body(m):
                rx = ethi.rx(m).data
                with m.If(~dset):
                    m.d.sync += ledsd.eq(rx)
                    m.d.sync += dset.eq(1)
                with m.Elif(~sset):
                    m.d.sync += ledsd.eq(ledsd | (rx<<8))
                    m.d.sync += sset.eq(1)
                with m.Elif(~ttet):
                    m.d.sync += ledsg.eq(rx)
                    m.d.sync += ttet.eq(1)
            m.d.comb += leds.eq(ledsd)
            
            with m.If(bt.i):
                m.d.sync += ledsd.eq(0)
                m.d.sync += dset.eq(0)
                m.d.sync += sset.eq(0)
                m.d.sync += ttet.eq(0)
                m.d.sync += ledsg.eq(0)
        #else:
        #with Transaction().body(m):
            #self.ethi.tx(m, self.ethi.rx(m))
        
        #rcv = Receiver(self.ethi.rx)
        #m.submodules.rcv = rcv
        #if platform is not None:
        #    leds = platform.request("led_r")
        #    m.d.comb  += leds.eq(rcv.led)

        return m
