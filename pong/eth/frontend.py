from amaranth import *
from transactron.core import Method, TModule, def_method
from amaranth.lib.fifo import AsyncFIFO

FRAME_LAYOUT = 8 # OCTETS

class EtherentInterface(Elaboratable):
    def __init__(self):
        self.i_rx_clk = Signal()
        self.i_rx_data = Signal(4)
        self.i_rx_valid = Signal()

        self.o_tx_clk = Signal()
        self.o_tx_data = Signal(4)
        self.o_tx_valid = Signal()
        
        self.rx = Method(o=[("data", 8), ("end", 1)])
        self.tx = Method(i=[("data", 8), ("end", 1)])
        
        self.rx_fifo = AsyncFIFO(width=9, depth=2048, r_domain="sync", w_domain="rx") 
        self.tx_fifo = AsyncFIFO(width=9, depth=128, w_domain="sync", r_domain="tx") 
    
    def elaborate(self, platform):
        m = TModule()


        m.domains.rx = cd_rx = ClockDomain(local=True)
        m.d.comb += cd_rx.clk.eq(self.i_rx_clk),
        m.d.comb += cd_rx.rst.eq(ResetSignal()) # type: ignore
        
        half_octet = Signal(4)
        half_octet_valid = Signal()
        rx_write = Signal()
        
        in_frame = Signal()
        eof = Signal()
        preamble_cnt = Signal(range(15))
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
                with m.Else():
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

        m.domains.tx = cd_tx = ClockDomain(local=True)
        m.d.comb += cd_tx.clk.eq(self.i_rx_clk)
        m.d.comb += cd_tx.rst.eq(ResetSignal()) # type: ignore
        m.submodules.tx_fifo = tx_fifo = self.tx_fifo
        m.d.comb += self.o_tx_clk.eq(cd_tx.clk)

        m.d.sync += tx_fifo.w_en.eq(0)
        @def_method(m, self.tx, ready=tx_fifo.w_rdy)
        def _(data, end):
            m.d.sync += tx_fifo.w_en.eq(1)
            m.d.sync += tx_fifo.w_data.eq(data)
        
        tx_octet = Signal()
        with m.If(~tx_octet & tx_fifo.r_rdy):
            m.d.tx += tx_octet.eq(1)
            m.d.comb += self.o_tx_data.eq(tx_fifo.r_data >> 4)
            m.d.comb += self.o_tx_valid.eq(1) 
        with m.If(tx_octet):
            m.d.tx += tx_octet.eq(0)
            m.d.comb += self.o_tx_data.eq(tx_fifo.r_data)
            m.d.comb += self.o_tx_valid.eq(1)
            m.d.comb += tx_fifo.r_en.eq(1)
        

        return m

