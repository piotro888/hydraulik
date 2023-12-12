from amaranth.sim import Simulator

from test.build import Top

dut = Top()

def bench():
    for _ in range(10000):
        yield


sim = Simulator(dut)

sim.add_clock(2e-8) # 50 MHz

sim.add_sync_process(bench)

with sim.write_vcd("trace.vcd"):
    sim.run()
