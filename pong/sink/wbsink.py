from amaranth import *
from pong.frontend.parser import Parser

from pong.sink.sink import PacketSink
from transactron.core import TModule, def_method

import re

from transactron.utils.assign import assign

class WishboneSink(Elaboratable, PacketSink):
    def __init__(self, mac, addr_base, name="sink"):
        self.sink_ctors()
        self.mac = mac
        self.addr_base = addr_base
        self.name = name

        self.irq = Signal()
        self.mmap = {}
        self.definestr = []

        self.data = Record(Parser.GET_LAYOUT)
        self.done = Record([("lock", 1)]) 

        self.range_start = addr_base
        self.range_end = addr_base
        self.set_mmap()
        self.definestr = "\n".join(self.definestr)


    def set_mmap(self):
        field_order = [
            self.data.eth.dest_mac,
            self.data.eth.source_mac,
            self.data.eth.type,
            self.data.ipv4.protocol,
            self.data.ipv4.length,
            self.data.ipv4.ttl,
            self.data.ipv4.src_addr,
            self.data.ipv4.dst_addr,
            self.data.udp.src_port,
            self.data.udp.dst_port,
            self.data.udp.length,
            self.data.udp.checksum,
            self.done.lock,
        ]

        curr_addr = self.addr_base
        self.definestr.append(f"/* generated for device '{self.name}' - base {hex(curr_addr*2)} (page {hex(curr_addr>>11)}) */")
        page_mask = 0x0fff
        for field in field_order:
            field_name = re.sub(r"__", "_", field.name.upper().upper())
            field_name = self.name.upper() + field_name[field_name.find('_'):]

            self.definestr.append(f"#define {field_name} ((volatile u16*){hex((curr_addr*2)&page_mask)}) // len: {field.shape().width} ({(field.shape().width+15)//16}) (addr: {hex(curr_addr*2)})")

            for sbit in range(0, field.shape().width, 16):
                sel_cnt = min(16, field.shape().width-sbit)
                self.mmap[curr_addr] = (field.bit_select(sbit, sel_cnt), "LOCK" in field_name)
                curr_addr += 1

        self.range_end = curr_addr

    def elaborate(self, platform):
        m = TModule()


        @def_method(m, self.filter)
        def _(arg):
            return arg.eth.valid & ((arg.eth.dest_mac == self.mac) | (arg.eth.dest_mac == self.mac+1)) & arg.ipv4.valid & arg.udp.valid

        irq_flag_cnt = Signal(range(8))
        @def_method(m, self.take, ready=~self.done.lock)
        def _(arg):
            m.d.sync += assign(self.data, arg)
            m.d.sync += self.done.lock.eq(1)
            m.d.sync += irq_flag_cnt.eq(7)

        with m.If(irq_flag_cnt):
            m.d.sync += irq_flag_cnt.eq(irq_flag_cnt-1)
            m.d.comb += self.irq.eq(1)

        return m

