from amaranth import *
from amaranth.sim import Simulator
from pong.proto.eth import Ethertype

from pong.proto.out.eth import PUSH_LAYOUT, REQUEST_LAYOUT, EthernetProtoOut
from transactron.core import TModule, TransactionModule
from coreblocks.test.common import TestbenchIO, AdapterTrans, def_method_mock, SimpleTestCircuit
from transactron.lib.adapters import Adapter
from transactron.utils.amaranth_ext.elaboratables import ModuleConnector


tbio = TestbenchIO(Adapter(o=REQUEST_LAYOUT))
_tbio = TestbenchIO(Adapter(i=PUSH_LAYOUT))
td = SimpleTestCircuit(EthernetProtoOut(tbio.adapter.iface, _tbio.adapter.iface))
dut = ModuleConnector(io=tbio, io2=_tbio,
        dut=TransactionModule(td)) 

idx = 0

#arp_data = [0x00,0x01,0x08,0x00,0x06,0x04,0x00,0x01,0x00,0x0e,0xc6,0xe2,0x42,0x0e,0x0a,0x00,0x03,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x0a,0x00,0x03,0x04]
arp_data = [0xde, 0xad, 0xbe, 0xef]

@def_method_mock(lambda: tbio, enable=lambda: True)
def req_mock():
    global idx
    idx = idx+1
    return {
            "data": arp_data[idx-1],
            "end": idx == len(arp_data)
    }

def active():
    yield  td._dut.ethertype.eq(0x0004)#Ethertype.ARP)
    yield td._dut.dest_mac.eq(0xaaaaaaaaaaaa)#0x000ec6e2420e)
    yield td._dut.source_mac.eq(0x555555555555)#-1)

    yield from td.start.call()
    yield from _tbio.enable()
    
    for i in range(500):
        yield


sim = Simulator(dut)

sim.add_clock(2e-8) # 50 MHz

sim.add_sync_process(req_mock)
sim.add_sync_process(active)

with sim.write_vcd("trace.vcd"):
    sim.run()


