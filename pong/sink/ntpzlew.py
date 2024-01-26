from amaranth import *
from pong.common import MY_IP, MY_MAC, STREAM_LAYOUT

from pong.common_typing import ZlewZDiura
from pong.proto.eth import Ethertype
from pong.proto.ipv4 import IPV4_PROTO_UDP
from pong.proto.ntp import NTPPORT
from pong.proto.out.eth import EthernetProtoOut
from pong.proto.out.ipv4 import IPv4ProtoOut
from pong.proto.out.udp import UdpProtoOut
from transactron.core import TModule, def_method
from transactron.lib.fifo import BasicFifo
from pong.proto.out.ntp import NtpProtoOut

class NtpZlew(Elaboratable, ZlewZDiura):
    def __init__(self):
        self.zlew_ctors()
        
        self.out_cont = BasicFifo(STREAM_LAYOUT, 2)
        self.out = self.out_cont.read

        self.leds = Signal(16)

    def elaborate(self, platform):
        m = TModule()
        m.submodules.out_cont = self.out_cont
        m.submodules.ntp = ntp = NtpProtoOut()
        m.submodules.udp = udp = UdpProtoOut(ntp.push)
        m.submodules.ipv4 = ipv4 = IPv4ProtoOut(udp.push)
        m.submodules.eth = eth = EthernetProtoOut(ipv4.push, self.out_cont.write)
        
        @def_method(m, self.filter)
        def _(arg):
            return arg.eth.valid & arg.ipv4.valid & arg.udp.valid & arg.ntp.valid
        
        m.d.comb += eth.source_mac.eq(MY_MAC)
        m.d.comb += eth.ethertype.eq(Ethertype.IPV4)
        m.d.comb += ipv4.ttl.eq(64)
        m.d.comb += ipv4.protocol.eq(IPV4_PROTO_UDP)
        m.d.comb += ipv4.src_addr.eq(MY_IP)
        m.d.comb += udp.source_port.eq(NTPPORT)

        @def_method(m, self.take)
        def _(arg):
            eth.start(m)
            m.d.sync += eth.dest_mac.eq(arg.eth.source_mac)
            m.d.sync += ipv4.dst_addr.eq(arg.ipv4.src_addr)
            m.d.sync += ipv4.length.eq(arg.ipv4.length)
            m.d.sync += udp.dest_port.eq(arg.udp.src_port)
            m.d.sync += udp.length.eq(arg.udp.length)
            m.d.sync += ntp.rec.eq(arg.ntp.txts)
            m.d.sync += ntp.xmt.eq(arg.ntp.txts)
            m.d.sync += ntp.org.eq(arg.ntp.txts)
            m.d.sync += ntp.ref.eq(arg.ntp.txts)
            m.d.sync += self.leds.eq(arg.ntp.txts)

        return m
