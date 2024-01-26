

from amaranth import *
from amaranth.sim import Settle, Simulator
from pong.common import STREAM_LAYOUT
from pong.eth.crcdiscard import CRCDiscarder
from pong.frontend.parser import Parser

from pong.sink.sink import PacketSink
from transactron.core import TModule, TransactionModule, def_method
from coreblocks.test.common import TestbenchIO, def_method_mock, SimpleTestCircuit
from transactron.lib.adapters import Adapter
from transactron.utils.amaranth_ext.elaboratables import ModuleConnector


class MockArpSink(Elaboratable, PacketSink):
    def __init__(self) -> None:
        self.sink_ctors()
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

source = TestbenchIO(Adapter(o=STREAM_LAYOUT))
crcdiscard = CRCDiscarder(source.adapter.iface)
parser = Parser(crcdiscard.method_out)
parser.add_sink(0, sink1)
td = SimpleTestCircuit(parser)

dut = TransactionModule(ModuleConnector(io=source, s1=sink1, crc=crcdiscard, dut=td))

cnt = 0
idx = 0

udp_pkt = "01005e7ffffa000ec6e2420e0800450000c305be40000111716a0a000808effffffa82d0076c00af02c34d2d534541524348202a20485454502f312e310d0a484f53543a203233392e3235352e3235352e3235303a313930300d0a4d414e3a2022737364703a646973636f766572220d0a4d583a20310d0a53543a2075726e3a6469616c2d6d756c746973637265656e2d6f72673a736572766963653a6469616c3a310d0a555345522d4147454e543a204368726f6d69756d2f3131312e302e353536332e3635204c696e75780d0a0d0acccccccc"
ntp_pkt = "428880aaef00000ec6e2420e08004500004c12304000401104690a0008080a000801007b007b00382452e30003fa000100000001000000000000000000000000000000000000000000000000000000000000e95e9d297e153083cccccccc"
def str_pkt_to_arr(s: str) -> list[int]:
    res = []
    for i in range(len(s)//2):
        hex = s[2*i:2*i+2]
        res.append(int(hex, 16))
    return res

pkts = [ [0xff, 0xff, 0xff],
        str_pkt_to_arr(ntp_pkt),
        str_pkt_to_arr(udp_pkt),
            [   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x0e, 0xc6, 0xe2, 0x42, 0x0e, 0x08, 0x06, 0x00, 0x01,
  0x08, 0x00, 0x06, 0x04, 0x00, 0x01, 0x00, 0x0e, 0xc6, 0xe2, 0x42, 0x0e, 0x0a, 0x00, 0x08, 0x08,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x08, 0x01, 
             ]+ [0x0]*18 + [
    0xcc, 0xcc, 0xcc, 0xcc]

]
print(len(pkts[1]))

@def_method_mock(lambda: source, enable=lambda: cnt < len(pkts))
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
    for _ in range(1000):
        yield # push pkt, none ready
    
sim = Simulator(dut)

sim.add_clock(2e-8) # 50 MHz

sim.add_sync_process(req_mock)
sim.add_sync_process(active)

with sim.write_vcd("trace.vcd"):
    sim.run()

