from amaranth import *

from pong.proto.out.proto_out import ProtoOut
from pong.utils.endian import endian_reverse
from transactron.core import Method, TModule, def_method

class UdpProtoOut(Elaboratable, ProtoOut):
    def __init__(self, forward: Method):
        self.proto_out_ctors()
        #self.forward_connector = Connect(STREAM_LAYOUT)
        #self.forward = self.forward_connector.write
        self.forward = forward

        self.source_port = Signal(16)
        self.dest_port = Signal(16)
        self.length = Signal(16) 
        self.checksum = Signal(16)
    
    def elaborate(self, platform):
        m = TModule()

        HDR_LEN = 4*16//8
        header =( 
                Cat(self.checksum, self.length, self.dest_port,  self.source_port)
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
                fwd = self.forward(m)
                with m.If(fwd.end):
                    m.d.comb += end.eq(1)
                    m.d.sync += hdr_cnt.eq(0)
                with m.Else():
                    m.d.comb += data.eq(fwd.data)

            return { "data": data, "end": end }
                
            

        return m
