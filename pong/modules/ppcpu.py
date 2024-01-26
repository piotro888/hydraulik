from amaranth import *
from pong.modules.wishbone_adapter import WishboneAdapter
from transactron.core import TModule

class PPCPUWrapper(Elaboratable):
    def __init__(self, wba: WishboneAdapter):
        self.uart_rx = Signal(1)
        self.uart_tx = Signal(1)
        self.pc_leds = Signal(18)
        self.dbg_r0 = Signal(16)
        self.irq = Signal()
        
        self.ppcpu = Instance("ppcpu_soc",
            ("i", "i_clk", ClockSignal()),
            ("i", "i_rst", ~ResetSignal()),
            ("i", "uart_rx", self.uart_rx),
            ("o", "uart_tx", self.uart_tx),
            ("o", "pc_leds", self.pc_leds),
            ("o", "dbg_r0", self.dbg_r0),
            ("o", "hw_clk", wba.wb_clk),
            ("o", "hw_cyc", wba.cyc),
            ("o", "hw_stb", wba.stb),
            ("o", "hw_adr", wba.addr),
            ("o", "hw_we", wba.we),
            ("i", "hw_i_data", wba.data_r),
            ("o", "hw_o_data", wba.data_w),
            ("i", "hw_ack", wba.ack),
            ("i", "hw_err", wba.err),
            ("i", "hw_irq", self.irq), 

        )


    def elaborate(self, platform):
        m = Module()
        m.submodules.ppcpu_soc = self.ppcpu 
        return m

