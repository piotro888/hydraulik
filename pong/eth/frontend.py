from amaranth import *
from transactron.core import Method, TModule, def_method
from amaranth.lib.fifo import AsyncFIFO

FRAME_LAYOUT = [("data", 8), ("end", 1)]

class EtherentInterface(Elaboratable):
    def __init__(self):
        self.i_rx_clk = Signal()
        self.i_rx_data = Signal(4)
        self.i_rx_valid = Signal()

        self.o_tx_clk = Signal()
        self.o_tx_data = Signal(4)
        self.o_tx_valid = Signal()
        
        self.rx = Method(o=FRAME_LAYOUT)
        self.tx = Method(i=FRAME_LAYOUT)
        
        self.rx_fifo = AsyncFIFO(width=9, depth=2048, r_domain="sync", w_domain="rx") 
        self.tx_fifo = AsyncFIFO(width=9, depth=2048, w_domain="sync", r_domain="tx") 
    
    def elaborate(self, platform):
        m = TModule()
        
        tx_clk = Signal()
        m.d.sync += tx_clk.eq(~tx_clk)


        ############################### RX #####################
        
        m.domains.rx = cd_rx = ClockDomain(local=True)
        m.d.comb += cd_rx.clk.eq(self.i_rx_clk),
        m.d.comb += cd_rx.rst.eq(ResetSignal()) # type: ignore
        
        half_octet = Signal(4)
        half_octet_valid = Signal()
        
        in_frame = Signal()
        eof = Signal()
        preamble_cnt = Signal(range(15))
        empty_count = Signal(range(15))
        with m.FSM("idle", domain="rx"):
            with m.State("idle"):
                with m.If(self.i_rx_valid & (self.i_rx_data == 0x5)):
                    m.next = "preamble"
                    m.d.rx += preamble_cnt.eq(0)
                    m.d.rx += half_octet_valid.eq(0)
            with m.State("preamble"):
                with m.If(~self.i_rx_valid):
                    m.next = "idle"
                with m.Elif((self.i_rx_data == 0xd) & (preamble_cnt > 10)):
                    m.next = "frame"
                with m.Elif(self.i_rx_data == 0x5):
                    m.d.rx += preamble_cnt.eq(preamble_cnt+1)
                with m.Else():
                    m.next = "idle"
            with m.State("frame"):
                with m.If(self.i_rx_valid):
                    m.d.comb += in_frame.eq(1)
                    m.d.rx += empty_count.eq(0)
                with m.Else():
                    m.d.rx += empty_count.eq(empty_count+1)
                    with m.If(empty_count == 10):
                        m.d.comb += eof.eq(1)
                        m.next = "idle"

        rx_octet = Cat(half_octet, self.i_rx_data)
        rx_wr_d = Signal(9)
        rx_wr_v = Signal()
        
        m.d.rx += rx_wr_v.eq(0)
        with m.If(self.i_rx_valid & in_frame):
            with m.If(half_octet_valid):
                m.d.rx += half_octet_valid.eq(0)
                m.d.rx += rx_wr_v.eq(1)
                m.d.rx += rx_wr_d.eq(rx_octet)
            with m.Else():
                m.d.rx += half_octet_valid.eq(1)
                m.d.rx += half_octet.eq(self.i_rx_data)
        with m.Elif(eof):
            m.d.rx += rx_wr_v.eq(1)
            m.d.rx += rx_wr_d.eq(1<<8)
        
        m.submodules.rx_fifo = rx_fifo = self.rx_fifo

        m.d.comb += rx_fifo.w_data.eq(rx_wr_d)
        m.d.comb += rx_fifo.w_en.eq(rx_wr_v)
        
        @def_method(m, self.rx, ready=rx_fifo.r_rdy)
        def _():
            m.d.comb += rx_fifo.r_en.eq(1)
            return rx_fifo.r_data

        ############################### TX #####################

        m.domains.tx = cd_tx = ClockDomain(local=True)
        m.d.comb += cd_tx.clk.eq(tx_clk)
        m.d.comb += cd_tx.rst.eq(ResetSignal()) # type: ignore
        m.submodules.tx_fifo = tx_fifo = self.tx_fifo

        m.d.comb += self.o_tx_clk.eq(tx_clk)

        preamble_cnt = Signal(range(16))
        octet_part = Signal()
        ipg_cnt = Signal(range(12))
         
        with m.FSM("idle", domain="tx"):
            with m.State("idle"):
                with m.If(tx_fifo.r_rdy):
                    m.d.tx += self.o_tx_valid.eq(1)
                    m.d.tx += self.o_tx_data.eq(0x5)
                    m.d.tx += preamble_cnt.eq(1)
                    m.next = "preamble"
            with m.State("preamble"):
                with m.If(preamble_cnt == 15):
                    m.d.tx += self.o_tx_data.eq(0xd) # SFD
                    m.d.tx += octet_part.eq(0)
                    m.next = "data"
                m.d.tx += preamble_cnt.eq(preamble_cnt + 1)
            with m.State("data"):
                # tx fifo must supply full packet (not checked)!
                with m.If(~octet_part):
                    with m.If(tx_fifo.r_data & (1<<8)): # end of packet
                        m.d.tx += self.o_tx_valid.eq(0)
                        m.d.tx += self.o_tx_data.eq(0)
                        m.d.tx += ipg_cnt.eq(0)
                        m.d.comb += tx_fifo.r_en.eq(1)
                        m.next = "ipg"
                    with m.Else():
                        m.d.tx += self.o_tx_data.eq(tx_fifo.r_data.word_select(0, 4))     
                        m.d.tx += octet_part.eq(1)
                with m.Else():
                    m.d.comb += tx_fifo.r_en.eq(1)
                    m.d.tx += self.o_tx_data.eq(tx_fifo.r_data.word_select(1, 4))
                    m.d.tx += octet_part.eq(0)
            with m.State("ipg"): # interpacket gap
                with m.If(ipg_cnt == 12-1):
                    m.next = "idle"
                m.d.tx += ipg_cnt.eq(ipg_cnt+1)
                 

        @def_method(m, self.tx, ready=tx_fifo.w_rdy)
        def _(data, end):
            m.d.comb += tx_fifo.w_en.eq(1)
            m.d.comb += tx_fifo.w_data.eq(Cat(data, end))
        
        return m

