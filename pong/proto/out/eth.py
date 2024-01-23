from amaranth import *
from pong.proto.out.proto_out import ProtoOut
from pong.utils.crc import CRC
from transactron import *
from pong.proto.eth import Ethertype

SELFMAC = 0x428880AAEF00

MAC_LEN = 6*8
PREFIX_LEN = (6+6+2)

MIN_PAYLOAD_LEN = 46
 
REQUEST_LAYOUT = [("data",8), ("end", 1)]
PUSH_LAYOUT = [("data", 8), ("end", 1)]

class EthernetProtoOut(Elaboratable, ProtoOut):
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
        crc_count = Signal(range(4))
        crc_value = Signal(32)
        end = Signal()
        data_size = Signal(range(1600))

        frame_prefix = Cat(self.ethertype, self.source_mac, self.dest_mac) 
        
        
        data = Signal(8)
        
        def tx_data():
            self.push(m, data=data, end=end)
            crc.push(m, data)
        
        with m.FSM("idle") as fsm:
            with m.State("idle"):
                with m.If(self.start.run):
                    with Transaction().body(m):
                        m.d.sync += octet_count.eq(0)
                        crc.clear(m)
                        m.next = "prefix"
            with m.State("prefix"):
                with Transaction().body(m):
                    m.d.comb += data.eq(frame_prefix.word_select((PREFIX_LEN-octet_count-1).as_unsigned(), 8))
                    m.d.sync += octet_count.eq(octet_count + 1)
                    tx_data()
                    with m.If(octet_count == PREFIX_LEN-1):
                        m.next = "data"
                        m.d.sync += data_size.eq(0)
            with m.State("data"):
                with Transaction().body(m):
                    with Transaction().body(m):
                        r = self.request(m)
                        m.d.comb += data.eq(r.data)
                        m.d.sync += data_size.eq(data_size + 1)
                        tx_data()
                        with m.If(r.end):
                            with m.If(data_size < MIN_PAYLOAD_LEN-1): 
                                m.next = "fill"
                            with m.Else():
                                m.next = "crc"
            with m.State("fill"):
                m.d.comb += data.eq(0)
                with Transaction().body(m):
                    m.d.sync += data_size.eq(data_size + 1)
                    tx_data()
                
                    with m.If(data_size == MIN_PAYLOAD_LEN-1):
                        m.next = "crc"
            with m.State("crc"):
                with Transaction().body(m):
                    crc_data = crc.get(m)
                    m.d.comb += data.eq(crc_data)
                    m.d.sync += crc_value.eq(crc_data)
                    tx_data()
                    m.d.sync += crc_count.eq(1)
                    m.next = "crc_send"
            with m.State("crc_send"):
                with Transaction().body(m):
                    m.d.sync +=  crc_count.eq(crc_count+1)
                    m.d.comb += data.eq(crc_value.word_select(crc_count, 8))
                    tx_data()
                    with m.If(crc_count == 3):
                        m.next = "end"
            with m.State("end"):
                with Transaction().body(m):
                    m.d.comb += data.eq(0)
                    m.d.comb += end.eq(1)
                    tx_data()
                    m.next = "idle"

        @def_method(m, self.start, ready=fsm.ongoing("idle"))
        def _():
            pass

        return m
        

