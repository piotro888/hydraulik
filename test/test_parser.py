

from amaranth import *
from amaranth.sim import Settle, Simulator
from pong.common import STREAM_LAYOUT
from pong.frontend.parser import Parser

from pong.sink.sink import PacketSink
from transactron.core import TModule, TransactionModule, def_method
from coreblocks.test.common import TestbenchIO, def_method_mock, SimpleTestCircuit
from transactron.lib.adapters import Adapter
from transactron.utils.amaranth_ext.elaboratables import ModuleConnector


class MockSink(Elaboratable, PacketSink):
    def __init__(self) -> None:
        self.sink_ctors()
        self.en = Signal()
        self.ready = Signal()
        self.arp_sha = Signal(3*16)
        self.taken = Signal()

    def elaborate(self, platform):
        m = TModule()

        @def_method(m, self.filter)
        def _(arg):
            return {"want":self.en}

        @def_method(m, self.take, ready=self.ready)
        def _(arg):
            m.d.sync += self.arp_sha.eq(arg.arp.sha)
            m.d.comb += self.taken.eq(1)

        return m

sink1 = MockSink()
sink2 = MockSink()
sink3 = MockSink()
sink4 = MockSink()

source = TestbenchIO(Adapter(o=STREAM_LAYOUT))
parser = Parser(source.adapter.iface)
parser.add_sink(0, sink1)
parser.add_sink(0, sink2)
parser.add_sink(1, sink3)
parser.add_sink(2, sink4)

td = SimpleTestCircuit(parser)
dut = ModuleConnector(io=source, s1=sink1, s2=sink2, s3=sink3,s4=sink4, dut=TransactionModule(td)) 

cnt = 0
idx = 0

arp_pkt = [0x00,0x01,0x08,0x00,0x06,0x04,0x00,0x01,0x00,0x0e,0xc6,0xe2,0x42,0x0e,0x0a,0x00,0x03,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x0a,0x00,0x03,0x04]

@def_method_mock(lambda: source, enable=lambda: cnt < 4)
def req_mock():
    global cnt, idx

    if idx == len(arp_pkt):
        data = 0
        end = 1
        cnt = cnt+1
        idx = 0
    else:
        data = arp_pkt[idx]
        idx = idx+1
        end = 0

    return {
            "data": data,
            "end": end  
    }

def active():
    yield sink4.en.eq(1) # dummy filter so packets will not be dropped

    for _ in range(100):
        yield # push pkt, none ready
    
    # prio 0 should be selected & wait
    yield sink1.en.eq(1)
    yield sink3.en.eq(1)
    yield sink3.ready.eq(1)
    yield Settle()
    assert (yield sink3.taken) == 0
    yield
    yield
    yield
    # take by 0
    yield sink1.ready.eq(1)
    yield Settle()
    assert (yield sink1.taken) == 1
    yield
    yield
    yield
    yield
    yield sink1.en.eq(0) # allow layer 2
    while (yield sink3.taken) == 0:
        yield

    yield sink1.ready.eq(0)
    yield sink1.en.eq(1)

    for _ in range(50):
        yield

    yield sink2.en.eq(1)
    yield
    yield sink1.en.eq(0)
    yield sink1.ready.eq(1)
    yield
    yield sink1.ready.eq(0)
    yield sink1.en.eq(1)
    yield
    yield
    yield sink2.ready.eq(1)
    while (yield sink2.taken) == 0:
        yield

    yield sink1.ready.eq(0)
    yield sink2.ready.eq(0)
    for _ in range(50):
        yield
    yield sink1.en.eq(1)
    yield sink2.en.eq(1)
    yield sink1.ready.eq(1)
    yield sink2.ready.eq(1)
    yield
    assert (yield sink1.taken) + (yield sink2.taken) == 1
    yield
    yield


sim = Simulator(dut)

sim.add_clock(2e-8) # 50 MHz

sim.add_sync_process(req_mock)
sim.add_sync_process(active)

with sim.write_vcd("trace.vcd"):
    sim.run()

