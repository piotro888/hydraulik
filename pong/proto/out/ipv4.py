from amaranth import *
from amaranth.hdl.ir import reduce
from amaranth.lib.enum import operator
from pong.common import STREAM_LAYOUT

from pong.proto.out.proto_out import ProtoOut
from pong.utils.endian import endian_reverse
from transactron.core import Method, TModule, def_method
from transactron.lib.connectors import ConnectForward, Forwarder

class IPv4ProtoOut(Elaboratable, ProtoOut):
    def __init__(self, forward: Method):
        self.proto_out_ctors()
        self.forward = forward
        #self.forward_connector = ConnectForward(STREAM_LAYOUT)
        #self.forward = self.forward_connector.write

        self.length = Signal(16) 
        self.protocol = Signal(8)
        self.ttl = Signal(8)
        self.src_addr = Signal(32)
        self.dst_addr = Signal(32)
    
    def elaborate(self, platform):
        m = TModule()

        HDR_LEN = (5*32)//8
        header = Cat(self.dst_addr, self.src_addr, C(0,16), self.protocol, self.ttl, C(0, 8), C(0b010_00000, 8), C(0, 16), self.length, C(0, 8), C(0x45, 8))


        checksum = Signal(16)

        checksum_step_1 = Signal(32)
        checksum_step_2 = Signal(16)
        m.d.comb += checksum_step_1.eq(reduce(operator.add, [endian_reverse(m, header.word_select(i, 16)) for i in range(HDR_LEN//2)]))
        m.d.comb += checksum_step_2.eq((checksum_step_1 & 0xFFFF) + (checksum_step_1 >> 16))
        m.d.sync += checksum.eq(~checksum_step_2)

        
        header_w_checksum = header | (endian_reverse(m, checksum) << (2*32))

        hdr_cnt = Signal(range(HDR_LEN+2))

        @def_method(m, self.push)
        def _():
            data = Signal(8)
            end = Signal()
            with m.If(hdr_cnt < HDR_LEN):
                m.d.comb += data.eq(endian_reverse(m, header_w_checksum.word_select((HDR_LEN-1-hdr_cnt).as_unsigned(), 8)))
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
