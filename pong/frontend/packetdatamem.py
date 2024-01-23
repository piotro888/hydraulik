from amaranth import *
from pong.common import STREAM_LAYOUT
from transactron.core import Method, TModule, Transaction, def_method
from transactron.lib.connectors import Forwarder
from transactron.lib.storage import MemoryBank


class PacketDataMem(Elaboratable):
    def __init__(self):
        self.level = Signal(range(1500+1))
        
        self.sink = Method(i=STREAM_LAYOUT)
        self.reset = Method()
        self.source = Method(o=STREAM_LAYOUT)

        self.iface_read = Method(i=[("addr", 11)])
        self.iface_read_resp = Method(o=[("data", 8)])
        self.iface_write = Method(i=[("addr", 11), ("data", 8)])

    def elaborate(self, platform):
        m = TModule()
        m.submodules.memory = memory = MemoryBank(data_layout=[("data", 8)], elem_count=1501, safe_writes=False)
        
        self.read_idx = Signal(range(1500))
        
        packet_ended = Signal()
        source_started = Signal()

        @def_method(m, self.sink, ready=~packet_ended)
        def _(data, end):
            m.d.sync += self.level.eq(self.level+1)
            memory.write(m, addr=self.level, data=data)

            with m.If(self.level == 0):
                memory.read_req(m, addr=0)
            
            with m.If(end):
                m.d.sync += packet_ended.eq(1)

        m.submodules.fifo_fwd = mem_res = Forwarder([("data", 8)])
        
        @def_method(m, self.source)
        def _():
            m.d.sync += source_started.eq(1)
            end = Signal()
            data = Signal(8)
            with m.If(self.read_idx == self.level):
                m.d.comb += end.eq(1)
                m.d.sync += self.level.eq(0)
                m.d.sync += self.read_idx.eq(0)
                m.d.sync += packet_ended.eq(0)
                m.d.sync += source_started.eq(0)
            with m.Else():
                m.d.sync += self.read_idx.eq(self.read_idx + 1)
                m.d.comb += data.eq(mem_res.read(m))
                memory.read_req(m, addr=self.read_idx+1)
            return {"data": data, "end": end}

        with Transaction().body(m):
            mem_res.write(m, memory.read_resp(m))

        @def_method(m, self.reset)
        def _():
            m.d.sync += self.level.eq(0)
            m.d.sync += self.read_idx.eq(0)
            m.d.sync += packet_ended.eq(0)
            m.d.sync += source_started.eq(0)

        ext_op_ready = packet_ended & ~source_started

        @def_method(m, self.iface_read, ready=ext_op_ready)
        def _(addr):
            memory.read_req(m, addr=self.read_idx)

        @def_method(m, self.iface_read_resp, ready=ext_op_ready)
        def _():
            return mem_res.read(m) 

        @def_method(m, self.iface_write, ready=ext_op_ready)
        def _(addr, data):
            memory.write(m, addr=addr, data=data)

        return m
