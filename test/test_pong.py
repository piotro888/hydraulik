from amaranth.sim import Simulator

from test.build import Top

dut = Top()

rx_data = [0xf, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0xd, 0x3, 0x3, 0x3, 0x3, 0x0, 0x2, 0xf, 0xf, 0xf, 0xf]

def bench():
    for d in rx_data:
        yield dut.c.ethi.i_rx_clk.eq(~dut.c.ethi.i_rx_clk)
        yield  
        yield dut.c.ethi.i_rx_clk.eq(~dut.c.ethi.i_rx_clk)
        yield
        yield dut.c.ethi.i_rx_valid.eq(1)
        yield dut.c.ethi.i_rx_data.eq(d)


    for _ in range(1000):
        yield dut.c.ethi.i_rx_clk.eq(~dut.c.ethi.i_rx_clk)
        yield  
        yield dut.c.ethi.i_rx_clk.eq(~dut.c.ethi.i_rx_clk)
        yield   
        yield dut.c.ethi.i_rx_valid.eq(0)
    

sim = Simulator(dut)

sim.add_clock(2e-8) # 50 MHz

sim.add_sync_process(bench)

with sim.write_vcd("trace.vcd"):
    sim.run()
