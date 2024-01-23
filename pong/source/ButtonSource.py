from amaranth import *

from pong.common import MY_IP, MY_MAC, STREAM_LAYOUT
from pong.proto.eth import Ethertype
from pong.proto.out.eth import EthernetProtoOut
from pong.proto.out.arp import ArpProtoOut
from pong.source.source import PacketSource

from transactron import *
from transactron.lib.connectors import Connect, Forwarder

class ButtonArpSource(PacketSource, Elaboratable):
    def __init__(self, button: Value):
        self.source_ctors()
        self.button = button

        self.out_cont = Forwarder(STREAM_LAYOUT)

        self.out = self.out_cont.read

    def elaborate(self, platform):
        m = TModule()

        m.submodules += self.out_cont

        btn_sync = Signal()
        btn_flag = Signal()
        prev_btn = Signal()
        btn_req = Signal()
        m.d.sync += btn_sync.eq(self.button)
        m.d.sync += prev_btn.eq(btn_sync)

        m.d.comb += btn_flag.eq(~prev_btn & btn_sync)
        with m.If(btn_flag):
            m.d.sync += btn_req.eq(1)

        m.submodules.arp = arp_out = ArpProtoOut()
        m.submodules.eth = pth_out = EthernetProtoOut(arp_out.push, self.out_cont.write)

        with Transaction().body(m, request=btn_req):
            pth_out.start(m)
            m.d.sync += btn_req.eq(0)

        m.d.comb += arp_out.opcode.eq(1) # request
        m.d.comb += arp_out.sender_ha.eq(MY_MAC)
        m.d.comb += arp_out.sender_pa.eq(MY_IP)
        m.d.comb += arp_out.target_ha.eq(0)
        m.d.comb += arp_out.target_pa.eq(0x0a000808)
        
        m.d.comb += pth_out.source_mac.eq(MY_MAC)
        m.d.comb += pth_out.dest_mac.eq(-1)
        m.d.comb += pth_out.ethertype.eq(Ethertype.ARP)

        return m

    
