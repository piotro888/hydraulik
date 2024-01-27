from amaranth import *
from pong.common import MY_IP, MY_MAC, STREAM_LAYOUT

from pong.common_typing import ZlewZDiura
from pong.modules.clock import PLL
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

        self.leds = Signal(20)
        self.leds2 = Signal(8)

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

        timestamp = Signal(64)
        m.d.sync += ntp.xmt.eq(timestamp)
        
        
        in_ts = Signal(64)
        @def_method(m, self.take)
        def _(arg):
            eth.start(m)
            m.d.sync += eth.dest_mac.eq(arg.eth.source_mac)
            m.d.sync += ipv4.dst_addr.eq(arg.ipv4.src_addr)
            m.d.sync += ipv4.length.eq(arg.ipv4.length)
            m.d.sync += udp.dest_port.eq(arg.udp.src_port)
            m.d.sync += udp.length.eq(arg.udp.length)
            m.d.sync += ntp.rec.eq(timestamp)
            m.d.sync += ntp.org.eq(arg.ntp.txts)
            m.d.comb += in_ts.eq(arg.ntp.txts)
        

        #### NTP TIME

        test_pps = Signal(range(110))
        m.d.sync += test_pps.eq(test_pps+1)
        tpps = Signal()
        with m.If(test_pps == 110):
            m.d.sync += test_pps.eq(0)
        m.d.comb += tpps.eq(test_pps == 5)

        seconds = Signal(32)
        seconds_synced = Signal()
        # initial sync
        with m.If(self.take.run & ~seconds_synced):
            with m.If((in_ts.bit_select(32-4, 4) >= 0b0011) & (in_ts.bit_select(32-4, 4) <= 0b1100)):
                m.d.sync += seconds.eq(in_ts>>32)
                m.d.sync += seconds_synced.eq(1)

        m.submodules.clock = clock = PLL(50_000_000 if platform else 100)
        if not platform:
            m.d.comb += clock.pps.eq(tpps)

        
        with m.If(clock.increment):
            m.d.sync += seconds.eq(seconds+1)

        m.d.comb += timestamp.eq(Cat(clock.frac, seconds))
        
        with m.If(clock.refs):
            m.d.sync += ntp.ref.eq(timestamp)

        m.d.comb += ntp.dispersion.eq(clock.prec >> 16)
        
        m.d.comb += self.leds.eq((clock.rt<<10) | clock.err_log)
        m.d.comb += self.leds2.eq((seconds << 4) | (clock.frac.bit_select(32-4, 4)))

        return m
