from amaranth import * 

from pong.sink.sink import PacketSink
from transactron.core import TModule, def_method


class ArpResolver(Elaboratable, PacketSink):
    def __init__(self, mac_addr: int, ip_addr: int):
        self.ctors()
        
        self.mac_addr = mac_addr
        self.ip_addr = ip_addr

        self.cnter = Signal(8)

    def elaborate(self, platform):
        m = TModule()
        
        @def_method(m, self.filter)
        def _(arg):
            return arg.eth.valid & arg.arp.valid # & (arg.arp.tpa == self.ip_addr)
        
        @def_method(m, self.take)
        def _(arg):
            m.d.sync += self.cnter.eq(self.cnter+1) 

        return m
