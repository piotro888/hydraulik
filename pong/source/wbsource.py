import re
from amaranth import *
from pong.common import STREAM_LAYOUT
from pong.proto.out.eth import EthernetProtoOut
from pong.proto.out.ipv4 import IPv4ProtoOut
from pong.proto.out.udp import UdpProtoOut
from pong.source.source import PacketSource

from transactron.core import TModule, Transaction
from transactron.lib.connectors import Forwarder

class WishboneSource(Elaboratable, PacketSource):
    def __init__(self, addr_base, packet_mem, name="source"):
        self.out_cont = Forwarder(STREAM_LAYOUT)
        self.out = self.out_cont.read
        
        self.packet_mem = packet_mem

        self.udp = UdpProtoOut(packet_mem.source)
        self.ipv4 = IPv4ProtoOut(self.udp.push)
        self.eth = EthernetProtoOut(self.ipv4.push, self.out_cont.write)
        
        self.start = Signal()
        self.drop = Signal()
        self.busy = Signal()
        
        self.name = name
        self.range_start = addr_base
        self.range_end = addr_base
        self.definestr = []
        self.mmap = {}
        self.set_mmap()
        self.definestr = "\n".join(self.definestr)

    def set_mmap(self):
        field_order = [
           ("eth",  self.eth.dest_mac),
           ("eth",  self.eth.source_mac),
           ("eth",  self.eth.ethertype),
           ("ipv4",  self.ipv4.protocol),
           ("ipv4", self.ipv4.length),
           ("ipv4", self.ipv4.ttl),
           ("ipv4", self.ipv4.src_addr),
           ("ipv4", self.ipv4.dst_addr),
           ("udp",  self.udp.source_port),
           ("udp", self.udp.dest_port),
           ("udp", self.udp.length),
           ("udp", self.udp.checksum),
           ("", self.start),
           ("", self.drop),
           ("", self.busy),
        ]
        # TODO: select source memory to send arb packets / arb iface 
        # TODO: quick copy from sink

        curr_addr = self.range_start
        self.definestr.append(f"/* generated for device '{self.name}' - base {hex(curr_addr*2)} (page {hex(curr_addr>>11)}) */")
        page_mask = 0x0fff
        for pre, field in field_order:
            field_name = re.sub(r"__", "_", field.name.upper())
            field_name = self.name.upper() + "_" + pre.upper() + ("_" if pre else "") + field_name

            self.definestr.append(f"#define {field_name} ((volatile u16*){hex((curr_addr*2)&page_mask)}) // len: {field.shape().width} ({(field.shape().width+15)//16}) (addr:{hex(curr_addr*2)})")
            for sbit in range(0, field.shape().width, 16):
                sel_cnt = min(16, field.shape().width-sbit)
                self.mmap[curr_addr] = (field.bit_select(sbit, sel_cnt), "BUSY" not in field_name)
                curr_addr += 1

        self.range_end = curr_addr
    def elaborate(self, platform):
        m = TModule()
        m.submodules.fwd = self.out_cont
        m.submodules.udp =  self.udp
        m.submodules.ipv4 = self.ipv4 
        m.submodules.eth = self.eth

        prev_start = Signal()
        start_f = Signal()
        start_m = Signal()
        m.d.sync += prev_start.eq(self.start)
        with m.If(~prev_start & self.start):
            m.d.sync += start_f.eq(~start_f)
        
        with Transaction().body(m, request=start_f ^ start_m):
            self.eth.start(m)
            m.d.sync += start_m.eq(~start_m)

        prev_drop = Signal()
        drop_f = Signal()
        drop_m = Signal()
        m.d.sync += prev_drop.eq(self.drop)
        with m.If(~prev_drop & self.drop):
            m.d.sync += drop_f.eq(~drop_f)

        with Transaction().body(m, request=drop_f ^ drop_m):
            self.packet_mem.reset(m)
            m.d.sync += drop_m.eq(~drop_m)

        m.d.comb += self.busy.eq(~self.eth.start.ready)

        return m
