from amaranth import *
from pong.common import STREAM_LAYOUT
from pong.utils.endian import endian_reverse, endian_reverse_record
from transactron import *
from pong.proto.proto_in import ProtoIn
from transactron.lib.connectors import ConnectForward

class UdpProto(Elaboratable, ProtoIn):
    GET_LAYOUT = [
            ("src_port", 16),
            ("dst_port", 16),
            ("length", 16),
            ("checksum", 16),
            ("valid", 1)
    ]

    def __init__(self):
        self.proto_in_ctors()
        self.forward_connector = ConnectForward(layout=STREAM_LAYOUT)
        self.forward = self.forward_connector.read

    def elaborate(self, module):
        m = TModule()
        m.submodules.forward_connector = self.forward_connector

        header = Signal(16*4)
        octet_count = Signal(range(1500+1))
    
        valid = Signal()

        @def_method(m, self.push)
        def _(data, end):
            fwd = Signal()
            with m.If(~end):
                with m.If(octet_count < 8):
                    m.d.sync += header.word_select(octet_count, 8).eq(data)
                with m.Else():
                    m.d.comb += fwd.eq(1)
                m.d.sync += octet_count.eq(octet_count+1)
            with m.Else():
                m.d.comb += fwd.eq(1)
                m.d.sync += valid.eq(
                    octet_count == endian_reverse(m, header.bit_select(32, 16))
                )
            
            with m.If(fwd):
                self.forward_connector.write(m, data=data, end=end)    

        @def_method(m, self.get)
        def _():
            #rec = Record(self.GET_LAYOUT)
            #m.d.comb += rec.eq(Cat(header, valid))
            #return endian_reverse_record(m, rec)
            #return rec# Cat(header, valid)
            # FOR SOME REASON RETURNING RECORDS LIKE BEFORE DOESNT WORK - 0 RESULT
            return {
                "src_port": endian_reverse(m, header.word_select(0, 16)),
                "dst_port": endian_reverse(m, header.word_select(1, 16)),
                "length": endian_reverse(m, header.word_select(2, 16)),
                "checksum": endian_reverse(m, header.word_select(3, 16)),
                "valid": valid
            }


        @def_method(m, self.clear)
        def _():
            m.d.sync += octet_count.eq(0)
            m.d.sync += valid.eq(0)

        return m
        

