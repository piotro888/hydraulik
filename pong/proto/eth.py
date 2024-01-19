from amaranth import *
from amaranth.lib.enum import IntEnum
from transactron import *

SELFMAC = 0x428880AAEFFF

MAC_LEN = 6*8
STRIP_PREFIX = (6+6+2)*8

class Ethertype(IntEnum, shape=16):
    IPV4 = 0x0800
    ARP = 0x0806

class EthNextType(IntEnum, shape=2):
    DROP = 0
    ARP = 1
    IPV4 = 2

class EthernetProto(Elaboratable):
    def __init__(self, forward: Method):
        self.push = Method(i=[("data", 8)])
        self.forward = forward 
    
        self.source_mac = Signal(MAC_LEN)
        self.dest_mac = Signal(MAC_LEN)
        self.fwd_type = Signal(EthNextType)

    def elaborate(self, module):
        m = TModule()

        prefix = Signal(STRIP_PREFIX)
        octet_count = Signal(range(1500+STRIP_PREFIX+4))
    

        m.d.comb += self.source_mac.eq(prefix[:MAC_LEN])
        m.d.comb += self.dest_mac.eq(prefix[MAC_LEN:MAC_LEN])

        ethertype = prefix.bit_select(2*MAC_LEN,2*8)
        

        with m.Switch(ethertype):
            #with m.Case(Ethertype.IPV4):
            #    m.d.comb += fwd_type.eq(EthNextType.IPV4)
            with m.Case(Ethertype.ARP):
                m.d.comb += self.fwd_type.eq(EthNextType.ARP)
            with m.Default():
                m.d.comb += self.fwd_type.eq(EthNextType.DROP)
        

        @def_method(m, self.push)
        def _(data):
            with m.If(octet_count < STRIP_PREFIX):
                m.d.sync += prefix.word_select(octet_count, 8).eq(data)
            with m.Else():
                self.forward(m, data)
            m.d.sync += octet_count.eq(octet_count + 1)

        return m
        

