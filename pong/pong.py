from amaranth import *
from pong.common import MY_IP, MY_MAC
from pong.eth.crcdiscard import CRCDiscarder
from pong.eth.frontend import EtherentInterface
from pong.frontend.parser import Parser
from pong.source.arbiter import PriorityStreamArbiter
from pong.sink.arpresolver import ArpCounter, ArpResolver
from pong.source.ButtonSource import ButtonArpSource
from pong.source.source import PacketSource
from pong.support.serial import Serial

from transactron.core import Method, TModule, Transaction, def_method 

from pong.eth.mdio import PHY_ETH0, REG_1000T_CTRL, MDIOController

class Pong(Elaboratable):
    def __init__(self):
        self.ethi = EtherentInterface()

    def elaborate(self, platform):
        m = TModule()

        m.submodules.mdio = mdio = MDIOController()
        m.submodules.eth0_i = ethi = self.ethi 
        m.submodules.eth1_i = ethi1 = EtherentInterface()
        m.submodules.uart = uart = Serial()
        
        if platform is not None:
            enet = platform.request("enet", 0)
            soft_loopback = 0
            reverse_loopback = 0
            if soft_loopback:
                # OK
                m.d.comb += ethi.i_rx_clk.eq(ethi.o_tx_clk)
                m.d.comb += ethi.i_rx_valid.eq(ethi.o_tx_valid)
                m.d.comb += ethi.i_rx_data.eq(ethi.o_tx_data)
            elif reverse_loopback:
                # NOT WORKINGGIGNIGNIRNIGSNRIGNIRSRNISGI
                m.d.comb += enet.rst.eq(ResetSignal())
                m.d.comb += enet.mdc.eq(mdio.o_mdc)
                m.d.comb += enet.mdio.o.eq(mdio.o_mdio)
                m.d.comb += enet.mdio.oe.eq(mdio.oe_mdio)
                m.d.comb += enet.gtx_clk.eq(enet.rx_clk)
                m.d.comb += enet.tx_data.eq(enet.rx_data)
                m.d.comb += enet.tx_en.eq(enet.rx_dv)
            else:
                m.d.comb += enet.mdc.eq(mdio.o_mdc)
                m.d.comb += enet.mdio.o.eq(mdio.o_mdio)
                m.d.comb += enet.mdio.oe.eq(mdio.oe_mdio)
                m.d.comb += enet.rst.eq(ResetSignal())
                m.d.comb += ethi.i_rx_clk.eq(enet.rx_clk)
                m.d.comb += ethi.i_rx_data.eq(enet.rx_data)
                m.d.comb += ethi.i_rx_valid.eq(enet.rx_dv)
                m.d.comb += enet.gtx_clk.eq(~ethi.o_tx_clk) # FUCKING CLOCK EDGEETTNESNTESNTES
                m.d.comb += enet.tx_data.eq(ethi.o_tx_data)
                m.d.comb += enet.tx_en.eq(ethi.o_tx_valid)

            ### PORT 1
            enet = platform.request("enet", 1)
            m.d.comb += enet.mdc.eq(mdio.o_mdc)
            m.d.comb += enet.mdio.o.eq(mdio.o_mdio)
            m.d.comb += enet.mdio.oe.eq(1)
            m.d.comb += enet.rst.eq(ResetSignal())
            m.d.comb += ethi1.i_rx_clk.eq(enet.rx_clk)
            m.d.comb += ethi1.i_rx_data.eq(enet.rx_data)
            m.d.comb += ethi1.i_rx_valid.eq(enet.rx_dv)
            m.d.comb += enet.gtx_clk.eq(ethi1.o_tx_clk)
            m.d.comb += enet.tx_data.eq(ethi1.o_tx_data)
            m.d.comb += enet.tx_en.eq(ethi1.o_tx_valid)
            ### UART
            puart = platform.request("uart")
            m.d.comb += puart.tx.eq(uart.o_tx)
        
        init = Signal(reset=1)
        btsig = Signal()
        with m.FSM("START"):
            with m.State("START"):
                with Transaction().body(m):
                    mdio.write(m, addr=PHY_ETH0, reg=REG_1000T_CTRL, data=0) # disable 1 Gig negotiation
                    #mdio.write(m, addr=PHY_ETH0, reg=25, data=0xffff-1) # LED OVERRIDE
                    m.next = "INIT1"
            with m.State("INIT1"):
                with Transaction().body(m):
                    mdio.write(m, addr=PHY_ETH0, reg=4, data=(1<<7)|(1<<8)) # enable only 100 BASET negotiation

                    m.next = "INIT2"
            with m.State("INIT2"):
                with Transaction().body(m):
                    #mdio.write(m, addr=PHY_ETH0, reg=0, data=(1<<15)) # soft reset 
                    m.next = "INIT_RESET"

            with m.State("INIT_RESET"):
                with Transaction().body(m, request=btsig):
                    #mdio.write(m, addr=PHY_ETH0, reg=20, data=(1<<14)) # F LINE LOOPBACK 
                    #mdio.write(m, addr=PHY_ETH0, reg=0, data=(1<<14)) # F MAC LOOPBACK 
                    #mdio.write(m, addr=PHY_ETH0, reg=0, data=(1<<15)) # soft reset 
                    m.next = "END"
            with m.State("END"):
                m.d.sync += init.eq(0)
        
        ledsd = Signal(16)
        if platform is not None:
            leds = platform.request("led_r")
            ledsg = platform.request("led_g")
            bt = platform.request("button")
            dset = Signal()
            sset = Signal()
            ttet = Signal()
  
            m.d.comb += leds.eq(ledsd)
            
            with m.If(bt.i):
                m.d.comb += btsig.eq(1)
                m.d.sync += ledsd.eq(0)
                m.d.sync += dset.eq(0)
                m.d.sync += sset.eq(0)
                m.d.sync += ttet.eq(0)
                #m.d.sync += ledsg.eq(0)

        ######### ZLEWMASTER
        m.submodules.crcdiscard = crcdiscard = CRCDiscarder(ethi.rx)
        m.submodules.parser = self.parser = Parser(crcdiscard.method_out)

        m.submodules.zlew = arp_zlew_potezny = ArpResolver(MY_MAC, MY_IP) 
        m.submodules.ctr = arp_ctr = ArpCounter()
        
        # SINKS
        self.parser.add_sink(0, arp_zlew_potezny)
        self.parser.add_sink(1, arp_ctr)
        

        # SOURCES
        m.submodules.btn_arp = btn_arp = ButtonArpSource(btsig)
        sources: list[PacketSource] = [
            arp_zlew_potezny,
            btn_arp
        ] 
        m.submodules.tx_arbiter = PriorityStreamArbiter([s.out for s in sources], ethi.tx) 
    
        
        if platform is not None:
            m.d.comb += ledsg.eq(arp_ctr.cnter) #type: ignore
        
        return m
