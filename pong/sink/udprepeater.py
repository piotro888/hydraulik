from amaranth import *
from pong.common import MY_IP, MY_MAC, STREAM_LAYOUT

from pong.common_typing import ZlewZDiura
from pong.frontend.packetdatamem import PacketDataMem
from pong.proto.eth import  Ethertype
from pong.proto.ipv4 import IPV4_PROTO_UDP
from pong.proto.out.eth import EthernetProtoOut
from pong.proto.out.ipv4 import IPv4ProtoOut
from pong.proto.out.udp import UdpProtoOut
from transactron.core import Method, TModule, Transaction, def_method
from transactron.lib.fifo import BasicFifo

class UdpRepeater(Elaboratable, ZlewZDiura):
    def __init__(self, port: int, pkt_mem: PacketDataMem):
        self.zlew_ctors()

        self.pkt_mem = pkt_mem
        self.port = port
        
        self.counter = Signal(8)
        self.out_cont = BasicFifo(STREAM_LAYOUT, 2)
        self.out = self.out_cont.read

    def elaborate(self, platoform):
        m = TModule()
        m.submodules.out_cont = self.out_cont

        #m.submodules.udpconn = ConnectTrans(ipv4.forward, udp.push)
        #m.submodules.memconn = ConnectTrans(self.pkt_mem.source, udp.forward)
        me = Method(o=STREAM_LAYOUT)
        @def_method(m, me)
        def _():
           return {"end":1, "data":0}
        
        m.submodules.udp = udp = UdpProtoOut(self.pkt_mem.source)
        m.submodules.ipv4 = ipv4 = IPv4ProtoOut(udp.push)
        m.submodules.eth = eth = EthernetProtoOut(ipv4.push, self.out_cont.write)

        def assign_source(m, arg):
            m.d.comb += eth.dest_mac.eq(arg.eth.source_mac)
            m.d.comb += eth.source_mac.eq(MY_MAC)
            m.d.comb += eth.ethertype.eq(Ethertype.IPV4)
            m.d.comb += ipv4.ttl.eq(64)
            m.d.comb += ipv4.protocol.eq(IPV4_PROTO_UDP)
            m.d.comb += ipv4.src_addr.eq(MY_IP)
            m.d.comb += ipv4.dst_addr.eq(arg.ipv4.src_addr)
            m.d.comb += ipv4.length.eq(arg.ipv4.length)
            m.d.comb += udp.source_port.eq(self.port)
            m.d.comb += udp.dest_port.eq(arg.udp.src_port)
            m.d.comb += udp.length.eq(arg.udp.length)


        @def_method(m, self.filter)
        def _(arg):
            # Hack: fileter always runs so we can take args comb from it
            assign_source(m, arg)

            # Real fileter
            return (
                    arg.eth.valid & (arg.eth.dest_mac == MY_MAC) & 
                    arg.ipv4.valid & (arg.ipv4.dst_addr == MY_IP) &
                    arg.udp.valid & (arg.udp.dst_port == self.port)
                )
        
        # no need to copy! FIFO will be non-empty, so next packet will not overwritte until fifo finished
        #pkt_source_mac = Signal(MAC_LEN)
        #pkt_source_ip = Signal(IP_LEN)
        #pkt_source_port = Signal(UDP_PORT_LEN)
        
        with m.FSM("idle") as fsm:
            with m.State("idle"):
                with m.If(self.take.run):
                    m.next = "memop"
            with m.State("memop"):
                with Transaction().body(m):
                    self.pkt_mem.iface_write(m, addr=0, data=self.counter)
                    m.next = "memop2"
            with m.State("memop2"):
                with Transaction().body(m):
                    self.pkt_mem.iface_write(m, addr=1, data=0xff-self.counter)
                    m.next = "send"
            with m.State("send"):
                with Transaction().body(m):
                    eth.start(m)        
                    m.next = "idle" # take will not be ready before entire fifo is read

        
        @def_method(m, self.take, ready=fsm.ongoing("idle"))
        def _(arg):
            m.d.sync += self.counter.eq(self.counter + 1)

        return m
