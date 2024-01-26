from amaranth import *
from pong.utils.endian import endian_reverse
from transactron import *
from pong.proto.proto_in import ProtoIn


NTPPORT = 123

class NtpProto(Elaboratable, ProtoIn):
    GET_LAYOUT = [
            ("txts", 64),
            ("valid", 1),
    ]

    def __init__(self):
        self.proto_in_ctors()

    def elaborate(self, module):
        m = TModule()

        header = Signal(32*(8+4))
        octet_count = Signal(range(1500+1))
    
        valid = Signal()

        @def_method(m, self.push)
        def _(data, end):
            m.d.sync += octet_count.eq(octet_count+1)
            with m.If(~end):
                with m.If(octet_count < (8+4)*4):
                    m.d.sync += header.word_select(octet_count, 8).eq(data)
            with m.Else():
                m.d.sync += valid.eq(
                    octet_count >= (8+4)*4               )
            
        @def_method(m, self.get)
        def _():
            return {
                "txts": endian_reverse(m, header.bit_select(10*32, 64)),
                "valid": valid
            }


        @def_method(m, self.clear)
        def _():
            m.d.sync += octet_count.eq(0)
            m.d.sync += valid.eq(0)

        return m
        

