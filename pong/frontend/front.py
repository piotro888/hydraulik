from amaranth import *
from pong.proto.eth import EthernetProto
from transactron.core import *
from transactron.lib.connectors import ConnectTrans

class Receiver(Elaboratable):
    def __init__(self, accept):
        self.accept = accept 
        self.led = Signal()
        
    def elaborate(self, platform):
        m = TModule()

        
        eth_switcher = Method(i=[("data", 8)])
        
        ethin = EthernetProto(eth_switcher)

        in_packet = Signal()
        packt_wait_drop = Signal()
        @def_method(m, eth_switcher)
        def _(data):
            with m.If(in_packet):
                pass
            with m.Else():
                m.d.sync += in_packet.eq(1)
                
                with m.If(ethin.dest_mac != 0xFFFFFFFFFFFFFFFF):
                    m.d.sync += packt_wait_drop.eq(1)

        m.d.comb += self.led.eq(in_packet & ~packt_wait_drop)
        
        m.submodules.ethinparse = ethin
        m.submodules.ethcon = ConnectTrans(self.accept, ethin.push)

        return m
                    

            
