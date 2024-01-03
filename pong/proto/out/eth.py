from amaranth import *
from pong.utils.crc import CRC
from transactron import *
from pong.proto.eth import Ethertype

SELFMAC = 0x428880AAEF00

MAC_LEN = 6*8
PREFIX_LEN = (6+6+2)

MIN_PAYLOAD_LEN = 46
 
REQUEST_LAYOUT = [("data",8), ("end", 1)]
PUSH_LAYOUT = [("data", 8)]
class EthernetProtoOut(Elaboratable):
    def __init__(self, request: Method, push: Method):
        self.push = push 
        self.request = request
        self.start = Method()

        self.source_mac = Signal(MAC_LEN)
        self.dest_mac = Signal(MAC_LEN)

        self.ethertype = Signal(Ethertype, reset=Ethertype.ARP) 

    def elaborate(self, module):
        m = TModule()

        m.submodules.crc = crc = CRC()

        octet_count = Signal(range(PREFIX_LEN))
        data_count = Signal(range(1500))
        crc_count = Signal(range(4))
        crc_value = Signal(32)
        break_count = Signal(range(12))

        frame_prefix = Cat(self.ethertype, self.source_mac, self.dest_mac) 
        
        data = Signal(8)
        data_v = Signal()
        with m.FSM("idle") as fsm:
            with m.State("idle"):
                with m.If(self.start.run):
                    with Transaction().body(m):
                        m.d.sync += octet_count.eq(0)
                        crc.clear(m)
                        m.next = "prefix"
            with m.State("prefix"):
                m.d.comb += data.eq(frame_prefix.word_select((PREFIX_LEN-octet_count-1).as_unsigned(), 8))
                m.d.comb += data_v.eq(1)
                m.d.sync += octet_count.eq(octet_count + 1)
                with m.If(octet_count == PREFIX_LEN-1):
                    m.next = "data"
                    m.d.sync += data_count.eq(0)
            with m.State("data"):
                with Transaction().body(m):
                    r = self.request(m)
                    m.d.comb += data_v.eq(1)
                    m.d.comb += data.eq(r.data)
                    m.d.sync += data_count.eq(data_count + 1)
                    with m.If(r.end):
                        #m.next = "data_fill"    
                #       with m.If(data_count < 46-1):
                 #           m.next = "data_fill"
                 #       with m.Else():
                        m.next = "crc"
            #with m.State("data_fill"):
            #    m.d.comb += data_v.eq(1)
            #    m.d.comb += data.eq(0)
            #    m.d.sync += data_count.eq(data_count + 1)
            #    with m.If(data_count == 46-1):
            #        m.next = "crc"
            with m.State("crc"):
                with Transaction().body(m):
                    crc_data = crc.get(m)
                    m.d.sync += crc_value.eq(crc_data)
                    m.d.comb += data_v.eq(1)
                    m.d.comb += data.eq(crc_data)
                    m.d.sync += crc_count.eq(1)
                    m.next = "crc_send"
            with m.State("crc_send"):
                m.d.sync +=  crc_count.eq(crc_count+1)
                m.d.comb += data_v.eq(1)
                m.d.comb += data.eq(crc_value.word_select(crc_count, 8))
                with m.If(crc_count == 3):
                    m.next = "break"
                    m.d.sync += break_count.eq(0)
            with m.State("break"):
                m.d.sync += break_count.eq(break_count + 1)
                with m.If(break_count == 11):
                    m.next = "idle"

        with Transaction().body(m):
            with m.If(data_v):
                self.push(m, data)
                crc.push(m, data)

        @def_method(m, self.start, ready=fsm.ongoing("idle"))
        def _():
            pass

        return m
        

