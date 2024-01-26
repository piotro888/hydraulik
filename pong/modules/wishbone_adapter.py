from amaranth import *
from transactron import *

class WishboneAdapter(Elaboratable):
    def __init__(self, *, mmap_devices=[], mrw_devices=[]):
        self.wb_clk = Signal()
        self.cyc = Signal()
        self.stb = Signal()
        self.addr = Signal(24)
        self.we = Signal()
        self.data_r = Signal(16)
        self.data_w = Signal(16)
        self.ack = Signal()
        self.err = Signal()

        self.mmap_devices = mmap_devices
        self.mrw_devices = mrw_devices # TODO: TODO

    def elaborate(self, platform):
        m = TModule()
        
        read_data = Signal(16)
        
        clk_edge = Signal()
        prev_clk = Signal()
        m.d.sync += prev_clk.eq(self.wb_clk)
        m.d.comb += clk_edge.eq(self.wb_clk & ~prev_clk)

        found = Signal()
        with m.FSM("idle"):
            with m.State("idle"):
                with m.If(self.cyc & self.stb & clk_edge):
                    for (addr, (sig, writeable)) in self.mmap_devices.items():
                        with m.If(self.addr == addr):
                            with m.If(self.we):
                                if not writeable: #~writeable aaaaaaa
                                    m.next = "err"
                                else:
                                    m.d.sync += sig.eq(self.data_w)
                            with m.Else():
                                m.d.sync += read_data.eq(sig)

                            m.d.comb += found.eq(1)
                            m.next = "reg_ack"
                    
                    with m.If(~found):
                        m.next = "err"

            with m.State("reg_ack"):
                m.d.comb += self.ack.eq(1) 
                m.d.comb += self.data_r.eq(read_data)
                with m.If(clk_edge):
                    m.next = "idle"

            with m.State("err"):
                m.d.comb += self.err.eq(1)
                with m.If(clk_edge):
                    m.next = "idle"
            
        return m
