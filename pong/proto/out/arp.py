from amaranth import *
from pong.proto.out.proto_out import ProtoOut
from transactron import *

PACKET_LEN = 28

class ArpProtoOut(Elaboratable, ProtoOut):
    def __init__(self):
        self.proto_out_ctors()
        
        self.opcode = Signal(16)
        self.sender_pa = Signal(2*16)
        self.sender_ha = Signal(3*16)
        self.target_pa = Signal(2*16)
        self.target_ha = Signal(3*16)
    
    def elaborate(self, module):
        m = TModule()
            
        octet_count = Signal(range(PACKET_LEN))

        # Ethernet + IPv4
        proto_pref = 0x0001_0800_06_04 
        proto_pref_len = 6*8

        data_frame = Cat(self.target_pa, self.target_ha, self.sender_pa, self.sender_ha, self.opcode, C(proto_pref, proto_pref_len))

        @def_method(m, self.push)
        def _():
            m.d.sync += octet_count.eq(octet_count+1)
            
            end = Signal()
            with m.If(octet_count == PACKET_LEN-1):
                m.d.sync += octet_count.eq(0) 
                m.d.comb += end.eq(1)

            return {
                "data": data_frame.word_select((PACKET_LEN-1-octet_count).as_unsigned(), 8),
                "end": end

            }

        return m
