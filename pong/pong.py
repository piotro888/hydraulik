from amaranth import *
from amaranth.lib.cdc import ResetSynchronizer
from pong.common import MY_IP, MY_MAC
from pong.eth.crcdiscard import CRCDiscarder
from pong.eth.frontend import EtherentInterface
from pong.frontend.parser import Parser
from pong.modules.ppcpu import PPCPUWrapper
from pong.modules.wishbone_adapter import WishboneAdapter
from pong.sink.udprepeater import UdpRepeater
from pong.sink.wbsink import WishboneSink
from pong.source.arbiter import PriorityStreamArbiter
from pong.sink.arpresolver import ArpCounter, ArpResolver
from pong.source.ButtonSource import ButtonArpSource
from pong.source.source import PacketSource
from pong.source.wbsource import WishboneSource
from pong.support.serial import Serial
from pong.sink.ntpzlew import NtpZlew

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
            if soft_loopback:
                # OK
                m.d.comb += ethi.i_rx_clk.eq(ethi.o_tx_clk)
                m.d.comb += ethi.i_rx_valid.eq(ethi.o_tx_valid)
                m.d.comb += ethi.i_rx_data.eq(ethi.o_tx_data)
            else:
                m.d.comb += enet.mdc.eq(mdio.o_mdc)
                m.d.comb += enet.mdio.o.eq(mdio.o_mdio)
                m.d.comb += enet.mdio.oe.eq(mdio.oe_mdio)
                m.d.comb += enet.rst.eq(ResetSignal())
                m.d.comb += ethi.i_rx_clk.eq(enet.rx_clk)
                m.d.comb += ethi.i_rx_data.eq(enet.rx_data)
                m.d.comb += ethi.i_rx_valid.eq(enet.rx_dv)
                m.d.comb += enet.gtx_clk.eq(~ethi.o_tx_clk)
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
            #m.d.comb += puart.tx.eq(uart.o_tx)
        
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
                    #mdio.write(m, addr=PHY_ETH0, reg=20, data=(1<<14)) # LINE LOOPBACK? 
                    #mdio.write(m, addr=PHY_ETH0, reg=0, data=(1<<14)) # MAC LOOPBACK?
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
               # m.d.sync += ledsd.eq(0)
                m.d.sync += dset.eq(0)
                m.d.sync += sset.eq(0)
                m.d.sync += ttet.eq(0)
                #m.d.sync += ledsg.eq(0)
            
        WITH_CPU = 0
        ######### ZLEWMASTER
        m.submodules.crcdiscard = crcdiscard = CRCDiscarder(ethi.rx)
        m.submodules.parser = self.parser = Parser(crcdiscard.method_out)

        m.submodules.zlew = arp_zlew_potezny = ArpResolver(MY_MAC, MY_IP) 
        m.submodules.ctr = arp_ctr = ArpCounter()
        m.submodules.udpr = udp_repeater = UdpRepeater(6969, MY_IP, MY_MAC, self.parser.pmem)
        if WITH_CPU:
            m.submodules.wbsink = wbsink = WishboneSink(MY_MAC, 0x4100)
        m.submodules.ntps = ntps = NtpZlew()

        # SINKS
        self.parser.add_sink(0, arp_zlew_potezny)
        self.parser.add_sink(0, ntps)
        self.parser.add_sink(1, udp_repeater)
        self.parser.add_sink(1, arp_ctr)
        if WITH_CPU:
            self.parser.add_sink(2, wbsink)
        

        # SOURCES
        m.submodules.btn_arp = btn_arp = ButtonArpSource(btsig)
        if WITH_CPU:
            m.submodules.wb_source = wb_source = WishboneSource(0x4000, self.parser.pmem)
        sources: list[PacketSource] = [
            ntps,
            arp_zlew_potezny,
            udp_repeater, 
            btn_arp,
        ] + ([wb_source] if WITH_CPU else [])
        m.submodules.tx_arbiter = PriorityStreamArbiter([s.out for s in sources], ethi.tx) 
    
        if WITH_CPU:
            m.submodules.wba = wba = WishboneAdapter(mmap_devices=wbsink.mmap|wb_source.mmap)
            m.submodules.ppcpu = self.ppcpu = PPCPUWrapper(wba) 
            m.d.comb += self.ppcpu.irq.eq(wbsink.irq)
        
        if platform is not None:
            m.d.comb += ledsg.eq(arp_ctr.cnter) #type: ignore
            m.d.comb += ledsg.eq(udp_repeater.counter) #type: ignore
            m.d.comb += ledsd.eq((self.parser.pmem.read_idx<<8) | self.parser.pmem.level)
            m.d.comb += ledsd.eq(ntps.leds)
            m.d.comb += ledsg.eq(ntps.leds2)
           

            if WITH_CPU:
                m.d.comb += puart.tx.eq(self.ppcpu.uart_tx)#type: ignore
                m.d.comb += self.ppcpu.uart_rx.eq(puart.rx)#type: ignore
                
                m.d.comb += ledsg.eq(Cat(wbsink.done.lock, wbsink.take.run, self.parser.pmem.level.any())) #type: ignore
                m.d.comb += ledsd.eq(self.ppcpu.dbg_r0) 

        
        if WITH_CPU:
            print(wbsink.definestr)
            print(wb_source.definestr)

        return m
