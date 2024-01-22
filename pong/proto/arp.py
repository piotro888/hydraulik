from amaranth import *
from pong.proto.out.eth import MIN_PAYLOAD_LEN
from pong.proto.proto_in import ProtoIn
from pong.utils.endian import endian_reverse, endian_reverse_record
from transactron import *

PACKET_LEN = 28*8
class ArpProto(ProtoIn, Elaboratable):
    GET_LAYOUT = [
        ("oper", 1*16),
        ("sha", 3*16),
        ("spa", 2*16),
        ("tha", 3*16),
        ("tpa", 2*16),
        ("valid", 1),
    ]

    def __init__(self):
        self.ctor()

    def elaborate(self, platform):
        m = TModule()
        
        prefix_buff = Signal(PACKET_LEN)
        octet_count = Signal(range(1500))
        pend = Signal()

        # Ethernet + IPv4
        VALID_PREF = 0x0001_0800_06_04 
        VALID_PREF_LEN = 6*8

        valid = Signal()
        m.d.comb += valid.eq(pend & (octet_count == MIN_PAYLOAD_LEN) & (endian_reverse(m, prefix_buff.bit_select(0, VALID_PREF_LEN)) == VALID_PREF))

        @def_method(m, self.push)
        def _(data, end):
            with m.If(~end):
                 m.d.sync += prefix_buff.word_select(octet_count, 8).eq(data)
                 m.d.sync += octet_count.eq(octet_count+1)
            m.d.sync += pend.eq(end)

        @def_method(m, self.clear)
        def _():
            m.d.sync += octet_count.eq(0)

        @def_method(m, self.get)
        def _():
            res = Record(self.GET_LAYOUT)
            m.d.comb += res.eq(Cat(prefix_buff[VALID_PREF_LEN:VALID_PREF_LEN+176], valid))
            return endian_reverse_record(m, res)

        return m
