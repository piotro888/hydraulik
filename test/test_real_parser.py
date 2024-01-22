

from amaranth import *
from amaranth.sim import Settle, Simulator
from pong.common import RX_LAYOUT
from pong.eth.crcdiscard import CRCDiscarder
from pong.frontend.parser import Parser

from pong.sink.sink import PacketSink
from transactron.core import TModule, TransactionModule, def_method
from coreblocks.test.common import TestbenchIO, def_method_mock, SimpleTestCircuit
from transactron.lib.adapters import Adapter
from transactron.utils.amaranth_ext.elaboratables import ModuleConnector


class MockArpSink(Elaboratable, PacketSink):
    def __init__(self) -> None:
        self.ctors()
        self.en = Signal()
        self.arp_sha = Signal(3*16)
        self.taken = Signal()

    def elaborate(self, platform):
        m = TModule()

        @def_method(m, self.filter)
        def _(arg):
            return arg.eth.valid & arg.arp.valid

        @def_method(m, self.take, )
        def _(arg):
            m.d.sync += self.arp_sha.eq(arg.arp.sha)
            m.d.sync += self.taken.eq(~self.taken)

        return m

sink1 = MockArpSink()

source = TestbenchIO(Adapter(o=RX_LAYOUT))
crcdiscard = CRCDiscarder(source.adapter.iface)
parser = Parser(crcdiscard.method_out)
parser.add_sink(0, sink1)
td = SimpleTestCircuit(parser)

dut = TransactionModule(ModuleConnector(io=source, s1=sink1, crc=crcdiscard, dut=td))

cnt = 0
idx = 0

pkts = [ [0xff, 0xff, 0xff],
            [   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x0e, 0xc6, 0xe2, 0x42, 0x0e, 0x08, 0x06, 0x00, 0x01,
  0x08, 0x00, 0x06, 0x04, 0x00, 0x01, 0x00, 0x0e, 0xc6, 0xe2, 0x42, 0x0e, 0x0a, 0x00, 0x08, 0x08,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x08, 0x01, 
             ]+ [0x0]*18 + [
    0xcc, 0xcc, 0xcc, 0xcc]

]
print(len(pkts[1]))

@def_method_mock(lambda: source, enable=lambda: cnt < 2)
def req_mock():
    global cnt, idx
    pkt = pkts[cnt]

    if idx == len(pkt):
        data = 0
        end = 1
        cnt = cnt+1
        idx = 0
    else:
        data = pkt[idx]
        idx = idx+1
        end = 0

    return {
            "data": data,
            "end": end  
    }

def active():
    for _ in range(100):
        yield # push pkt, none ready
    
sim = Simulator(dut)

sim.add_clock(2e-8) # 50 MHz

sim.add_sync_process(req_mock)
sim.add_sync_process(active)

with sim.write_vcd("trace.vcd"):
    sim.run()

