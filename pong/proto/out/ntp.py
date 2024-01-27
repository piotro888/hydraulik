from amaranth import *

from pong.proto.out.proto_out import ProtoOut
from pong.utils.endian import endian_reverse
from transactron.core import Method, TModule, def_method

class NtpProtoOut(Elaboratable, ProtoOut):
    def __init__(self):
        self.proto_out_ctors()

        self.xmt = Signal(64)
        self.rec = Signal(64)
        self.org = Signal(64)
        self.ref = Signal(64)

        self.dispersion = Signal(32)
    
    def elaborate(self, platform):
        m = TModule()

        HDR_LEN = (8+4)*4 
        header =( 
                Cat(self.xmt, self.rec, self.org, self.ref, C(0x47505300, 32), self.dispersion, C(0x01, 32), C(-24, 8), C(4, 8), C(1, 8), C(4,3), C(4, 3), C(0, 2))
        )

        hdr_cnt = Signal(range(HDR_LEN+1))

        @def_method(m, self.push)
        def _():
            data = Signal(8)
            end = Signal()
            with m.If(hdr_cnt < HDR_LEN):
                m.d.comb += data.eq(header.word_select((HDR_LEN-hdr_cnt-1).as_unsigned(), 8))
                m.d.sync += hdr_cnt.eq(hdr_cnt+1)
            with m.Else():
                m.d.sync += hdr_cnt.eq(0)
                m.d.comb += end.eq(1)

            return { "data": data, "end": end }
                
            

        return m
