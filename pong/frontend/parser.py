from typing import TYPE_CHECKING
from amaranth import *
from amaranth.hdl.ir import reduce
from amaranth.lib.enum import operator
from pong.proto.arp import ArpProto
from pong.proto.eth import EthernetProto, Ethertype
from transactron.core import *
from transactron.utils.amaranth_ext.elaboratables import OneHotSwitchDynamic

if TYPE_CHECKING:
    from pong.sink.sink import PacketSink

class Parser(Elaboratable):
    GET_LAYOUT = [
        ("eth", EthernetProto.GET_LAYOUT),
        ("arp", ArpProto.GET_LAYOUT),
    ]


    def __init__(self, accept: Method):
        self.accept = accept
        
        self.peth = EthernetProto()
        self.parp = ArpProto()
        
        self.sinks: dict[int, list["PacketSink"]] = {}

    def add_sink(self, priority: int, sink: "PacketSink"):
        if priority not in self.sinks:
            self.sinks[priority] = []
        self.sinks[priority].append(sink)

    def _get_data_dict(self, m: TModule):
        with Transaction().body(m):
            return {
                "eth": self.peth.get(m),
                "arp": self.parp.get(m)
            }

    def _sink_do(self, m: TModule, packet_done: Value) -> Signal:
        data = self._get_data_dict(m)

        max_prio = max(self.sinks.keys())
        prio_sorted = sorted(list(self.sinks.keys()), reverse=True)
        
        sink_filters: dict[int, list[Value]] = {}
        # call all filters first
        with Transaction().body(m):
            for prio, sinks in self.sinks.items():
                sink_filters[prio] = []
                for s in sinks:
                    sink_filters[prio].append(s.filter(m, data).want)

        # select priority level to execute -> if at least one fileter is ready wait for take on level
        prio_onehot = Signal(range((max_prio+1)*2))
        with m.If(packet_done):
            for prio in prio_sorted:
                sinks = self.sinks[prio]
                with m.If(reduce(operator.or_, sink_filters[prio])):
                    m.d.comb += prio_onehot.eq((1<<prio))
        
        # call only one take from selected priority level that has its filter wanted and is ready
        done = Signal()
        for prio in OneHotSwitchDynamic(m, prio_onehot):
            if prio not in self.sinks:
                continue

            prev_txn = None
            for idx, sink in enumerate(self.sinks[prio]):
                with Transaction().body(m, request=sink_filters[prio][idx]) as txn:
                    sink.take(m, data) 
                    m.d.comb += done.eq(1)

                if prev_txn:
                    prev_txn.add_conflict(txn)
                prev_txn = txn

        nobody_wants_me = ~reduce(operator.or_, [s for sl in sink_filters.values() for s in sl])
        with m.If(nobody_wants_me & packet_done): # :CCCCCC
            m.d.comb += done.eq(1)

        return done
               
    def elaborate(self, platform):
        m = TModule()
        
        #### STAGE 1
        m.submodules.peth = peth = self.peth
        stall = Signal()
        packet_done = Signal() 
        packet_drop_stub = Signal()

        m.d.comb += packet_done.eq(stall) # for now it is true - pure comb (with intentional one cycle break)
        with Transaction().body(m, request=~stall):
            in_pkt = self.accept(m)
            peth.push(m, in_pkt)
            with m.If(in_pkt.end):
                m.d.sync += stall.eq(1)

        #### STAGE 2

        m.submodules.parp = parp = self.parp 
        with Transaction().body(m):
            ethertype = peth.get(m).type
            fwd = peth.forward(m)
            with m.Switch(ethertype):
                with m.Case(Ethertype.ARP):
                    parp.push(m, fwd)
                with m.Default():
                    m.d.comb += packet_drop_stub.eq(1)


        ## SINK INSERTION
        done = self._sink_do(m, (packet_done & (~packet_drop_stub)) | (packet_drop_stub & packet_done))
        
        with Transaction().body(m, request=done):
            m.d.sync += stall.eq(0)
            self.peth.clear(m)
            self.parp.clear(m)

        return m

                    
