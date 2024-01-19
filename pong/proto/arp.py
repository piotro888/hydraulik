from amaranth import *
from transactron import *

PACKET_LEN = 28*8

def ArpProto(Elaboratable):
    def __init__(self):
        self.push = Method(i=8)
        self.forward = Method(o=[("data", 8), ("dest", 3)])
        self.clear = Method()

    def elaborate(self, module):
        m = TModule()
        
        opcode = Signal()
        sender_pa = Signal(2*8)
        sender_ha = Signal(3*8)
        target_pa = Signal(2*8)
        sender_pa = Signal(3*8)
        
        prefix_buff = Signal(PACKET_LEN)
        octet_count = Signal(range(PACKET_LEN))

        # TODO: detect invalid wbhen len >
        
        # Ethernet + IPv4
        valid_pref = 0x0001_0800_06_04 
        valid_pref_len = 6

        @def_method(m, self.push)
        def _(data):
             m.d.sync += prefix_buff[octet_count*8 : 8].eq(data)
             m.d.sync += octet_count.eq(octet_count+1)


        return m
