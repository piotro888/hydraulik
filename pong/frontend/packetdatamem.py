from amaranth import *
from pong.common import STREAM_LAYOUT
from transactron.core import Method, TModule, Transaction, def_method
from transactron.lib.connectors import Forwarder
from transactron.lib.storage import MemoryBank
from transactron.lib.simultaneous import condition

MEM_LEN_CAP = 64
MEM_LEN_CAP_BITS = MEM_LEN_CAP.bit_length()

class PacketDataMem(Elaboratable):
    def __init__(self):
        self.level = Signal(MEM_LEN_CAP_BITS)
        
        self.sink = Method(i=STREAM_LAYOUT)
        self.reset = Method()
        self.source = Method(o=STREAM_LAYOUT, nonexclusive=True) # One call a time should be garenteed by take logic. This will unblock Transaction conflicts instead.

        self.iface_read = Method(i=[("addr", 11)])
        self.iface_read_resp = Method(o=[("data", 8)])
        self.iface_write = Method(i=[("addr", 11), ("data", 8)])
        

        self.read_idx = Signal(MEM_LEN_CAP_BITS)
    def elaborate(self, platform):
        m = TModule()
        m.submodules.memory = memory = MemoryBank(data_layout=[("data", 8)], elem_count=MEM_LEN_CAP, safe_writes=False)
        

        
        packet_ended = Signal()
        source_started = Signal()
        
        zero_index_reg = Signal(8)

        @def_method(m, self.sink, ready=~packet_ended)
        def _(data, end):
            with m.If(end):
                m.d.sync += packet_ended.eq(1)
            with m.Else():
                memory.write(m, addr=self.level.bit_select(0, MEM_LEN_CAP_BITS), data=data)
                m.d.sync += self.level.eq(self.level+1)


            with m.If(self.level == 0):
                # expoted to start servicing source requests immediately without additional signalling
                m.d.sync += zero_index_reg.eq(data)            


        m.submodules.fifo_fwd = mem_res = Forwarder([("data", 8)])
        
        # transactron issues :ccc
        # exported to another txn
        data_read_sync = Signal(8)
        data_read_comb = Signal(8)
        new_request = Signal()
        with Transaction().body(m, request=new_request):
            memory.read_req(m, addr=self.read_idx+1)

        with Transaction().body(m, request=source_started) as t:
            res = memory.read_resp(m)
            m.d.sync += data_read_sync.eq(res)
            m.d.comb += data_read_comb.eq(res)
        data_read = Mux(t.grant, data_read_comb, data_read_sync)

        @def_method(m, self.source)
        def _():
            m.d.sync += source_started.eq(1)
            end = Signal()
            data = Signal(8)
            #with condition(m, priority=False, nonblocking=False) as cond:
                
            with m.If(self.read_idx == self.level):
                m.d.comb += end.eq(1)
                m.d.sync += self.level.eq(0)
                m.d.sync += self.read_idx.eq(0)
                m.d.sync += packet_ended.eq(0)
                m.d.sync += source_started.eq(0)
            with m.Else():#
                m.d.sync += self.read_idx.eq(self.read_idx+1)
                m.d.comb += data.eq(Mux(self.read_idx == 0, zero_index_reg, data_read))
                m.d.comb += new_request.eq(1)
            
            return {"data": data, "end": end}

        #with Transaction().body(m):
        #    mem_res.write(m, memory.read_resp(m))

        @def_method(m, self.reset)
        def _():
            m.d.sync += self.level.eq(0)
            m.d.sync += self.read_idx.eq(0)
            m.d.sync += packet_ended.eq(0)
            m.d.sync += source_started.eq(0)

        # iface is meant to used betwen end of recevie and start of send
        # it will not interfere with packet si/so requests
        ext_op_ready = packet_ended & ~source_started

        @def_method(m, self.iface_read, ready=ext_op_ready)
        def _(addr):
            memory.read_req(m, addr=self.read_idx.bit_select(0, MEM_LEN_CAP_BITS))

        @def_method(m, self.iface_read_resp, ready=ext_op_ready)
        def _():
            return memory.read_resp(m)

        @def_method(m, self.iface_write, ready=ext_op_ready)
        def _(addr, data):
            with m.If(addr == 0):
                m.d.sync += zero_index_reg.eq(data)
            with m.If(addr > self.level):
                m.d.sync += self.level.eq(addr+1)
            memory.write(m, addr=addr.bit_select(0, MEM_LEN_CAP_BITS), data=data)

        return m
