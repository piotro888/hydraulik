from amaranth import *
from pong.common import STREAM_LAYOUT
from pong.common_typing import ZlewZDiura
from pong.proto.eth import Ethertype
from pong.proto.out.arp import ArpProtoOut
from pong.proto.out.eth import EthernetProtoOut 
from pong.sink.sink import PacketSink
from transactron.core import TModule, def_method
from transactron.lib.fifo import BasicFifo

class ArpCounter(Elaboratable, PacketSink):
    def __init__(self):
        self.sink_ctors()
        
        self.cnter = Signal(8)

    def elaborate(self, platform):
        m = TModule()
        
        @def_method(m, self.filter)
        def _(arg):
            return arg.eth.valid & arg.arp.valid 
        
        @def_method(m, self.take)
        def _(arg):
            m.d.sync += self.cnter.eq(self.cnter+1) 

        return m

class ArpResolver(Elaboratable, ZlewZDiura):
    def __init__(self, mac_addr: int, ip_addr: int):
        self.sink_ctors()
        
        self.mac_addr = mac_addr
        self.ip_addr = ip_addr

        self.cnter = Signal(8)
        
        self.out_cont = BasicFifo(STREAM_LAYOUT, 2)
        self.out = self.out_cont.read

    def elaborate(self, platform):
        m = TModule()
        
        packet_mac = Signal(48)
        packet_ip = Signal(4*8)

        # SOURCE
        m.submodules += self.out_cont

        m.submodules.arp = arp_out = ArpProtoOut()
        m.submodules.eth = pth_out = EthernetProtoOut(arp_out.push, self.out_cont.write)

        m.d.comb += arp_out.opcode.eq(2) # response
        m.d.comb += arp_out.sender_ha.eq(self.mac_addr)
        m.d.comb += arp_out.sender_pa.eq(self.ip_addr)
        m.d.comb += arp_out.target_ha.eq(packet_mac)
        m.d.comb += arp_out.target_pa.eq(packet_ip)
        
        m.d.comb += pth_out.source_mac.eq(self.mac_addr)
        m.d.comb += pth_out.dest_mac.eq(packet_mac)
        m.d.comb += pth_out.ethertype.eq(Ethertype.ARP)
        
        # SINK
        @def_method(m, self.filter)
        def _(arg):
            return arg.eth.valid & arg.arp.valid & (arg.arp.tpa == self.ip_addr)
        
        @def_method(m, self.take)
        def _(arg):
            m.d.sync += packet_mac.eq(arg.arp.sha)
            m.d.sync += packet_ip.eq(arg.arp.spa)
            pth_out.start(m)

            m.d.sync += self.cnter.eq(self.cnter+1) 
        

        return m
