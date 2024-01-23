from amaranth import *
from amaranth.lib.enum import IntEnum
from pong.common import STREAM_LAYOUT
from pong.utils.endian import endian_reverse
from transactron import *
from pong.proto.proto_in import ProtoIn
from transactron.lib.connectors import Connect, ConnectForward

MAX_LEN = 20*8

IPV4_PROTO_UDP = 17

class IPv4Proto(Elaboratable, ProtoIn):
    GET_LAYOUT = [
            ("protocol", 8),
            ("length", 16),
            ("ttl", 8),
            ("src_addr", 4*8),
            ("dst_addr", 4*8),
            ("valid", 1)
    ]

    def __init__(self):
        self.proto_in_ctors()
        self.forward_connector = ConnectForward(layout=STREAM_LAYOUT)
        self.forward = self.forward_connector.read

    def elaborate(self, module):
        m = TModule()
        m.submodules.forward_connector = self.forward_connector

        header = Signal(MAX_LEN)
        in_header = Signal(reset=1)
        octet_count = Signal(range(1500+1))
    
        valid = Signal()

        # ENDIANNES IN BYTE FIEDLDS IS ALSO SWAPPED
        ihl = header.bit_select(4-4, 4)*4

        @def_method(m, self.push)
        def _(data, end):
            fwd = Signal()
            with m.If(~end):
                with m.If(in_header):
                    m.d.sync += header.word_select(octet_count, 8).eq(data)
                    m.d.sync += octet_count.eq(octet_count+1)
                    with m.If((octet_count > 2) & (octet_count == ihl-1)):
                        m.d.sync += in_header.eq(0)
                with m.Else():
                    m.d.sync += octet_count.eq(octet_count+1)
                    m.d.comb += fwd.eq(1)
            with m.Else():
                m.d.comb += fwd.eq(1)
                m.d.sync += in_header.eq(1)
                # not the endian swap inside 8 bit fields!
                m.d.sync += valid.eq(
                    (header.bit_select(4-0, 4) == 4) & # v4
                    ((header.bit_select(32+24, 16) | header.bit_select(32+16, 5)) == 0) & # 0 frag ofset
                    (header.bit_select(2*32+0, 8) != 0) & # TTL
                    (endian_reverse(m, header.bit_select(16, 16)) == octet_count) # len
                    # TODO : CS
                )
            
            with m.If(fwd):
                self.forward_connector.write(m, data=data, end=end)    

        @def_method(m, self.get)
        def _():
            return {
                "protocol": header.bit_select(2*32+8, 8),
                "length": endian_reverse(m, header.bit_select(16,16)),
                "ttl": header.bit_select(2*32, 8),
                "src_addr": endian_reverse(m, header.bit_select(3*32, 32)),
                "dst_addr": endian_reverse(m, header.bit_select(4*32, 32)),
                "valid": valid
            }

        @def_method(m, self.clear)
        def _():
            m.d.sync += octet_count.eq(0)
            m.d.sync += valid.eq(0)

        return m
        

