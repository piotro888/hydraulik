from amaranth import *
from transactron import *

PACKET_LEN = 28*8

def ArpProtoOut(Elaboratable):
    def __init__(self):
        self.push = Method(o=[("data",8), ("end",1)])
        
        self.opcode = Signal()
        self.sender_pa = Signal(2*8)
        self.sender_ha = Signal(3*8)
        self.target_pa = Signal(2*8)
        self.sender_pa = Signal(3*8)
    
    def elaborate(self, module):
        m = TModule()
            
        prefix_buff = Signal(PACKET_LEN)
        octet_count = Signal(range(PACKET_LEN))

        # Ethernet + IPv4
        proto_pref = 0x0001_0800_06_04 
        proto_pref_len = 6*8

        data_frame = Cat(self.target_ha, self.target_pa, self.sender_ha, self.sender_pa, self.opcode, C(proto_pref, proto_pref_len))

        @def_method(m, self.push)
        def _():
            m.d.sync += octet_count.eq(octet_count+1)
            
            end = Signal()
            with m.If(octet_count == PACKET_LEN-1):
                m.d.sync += octet_count.eq(0)   
                m.d.comb += end.eq(0)

            return {
                "data": data_frame.word_select(PACKET_LEN-1-octet_count, 8),
                "end": end

            }

        return m
