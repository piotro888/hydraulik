from amaranth import *
from amaranth.lib.enum import IntEnum
from pong.common import STREAM_LAYOUT
from pong.utils.endian import endian_reverse
from transactron import *
from pong.proto.proto_in import ProtoIn
from transactron.lib.connectors import Connect, ConnectForward

SELFMAC = 0x428880AAEFFF

MAC_LEN = 6*8
STRIP_PREFIX = (6+6+2)*8

RUNT_FRAME_TRESHOLD = 64 - 4 # -CRC discarded before

class Ethertype(IntEnum, shape=16):
    IPV4 = 0x0800
    ARP = 0x0806

class EthernetProto(Elaboratable, ProtoIn):
    GET_LAYOUT = [
            ("dest_mac", MAC_LEN),
            ("source_mac", MAC_LEN),
            ("type", 16),
            ("valid", 1),
    ]

    def __init__(self):
        self.proto_in_ctors()
        self.forward_connector = ConnectForward(layout=STREAM_LAYOUT)
        self.forward = self.forward_connector.read

    def elaborate(self, module):
        m = TModule()
        m.submodules.forward_connector = self.forward_connector

        prefix = Signal(STRIP_PREFIX)
        octet_count = Signal(range(1500+STRIP_PREFIX+4))
    
        valid = Signal()

        @def_method(m, self.push)
        def _(data, end):
            fwd = Signal()
            with m.If(~end):
                with m.If(octet_count < STRIP_PREFIX//8):
                    m.d.sync += prefix.word_select(octet_count, 8).eq(data)
                with m.Else():
                    m.d.comb += fwd.eq(1)
                m.d.sync += octet_count.eq(octet_count + 1)
                m.d.sync += valid.eq(0)
            with m.Else():
                m.d.sync += valid.eq(octet_count >= RUNT_FRAME_TRESHOLD)
                m.d.comb += fwd.eq(1)
            
            with m.If(fwd): # transactron bug???
                self.forward_connector.write(m, data=data, end=end)    

        @def_method(m, self.get)
        def _():
            return {
                "dest_mac": endian_reverse(m, prefix[:MAC_LEN]),
                "source_mac": endian_reverse(m, prefix[MAC_LEN:2*MAC_LEN]),
                "type": endian_reverse(m, prefix.bit_select(2*MAC_LEN, 2*8)),
                "valid": valid
            }

        @def_method(m, self.clear)
        def _():
            m.d.sync += octet_count.eq(0)
            m.d.sync += valid.eq(0)

        return m
        

