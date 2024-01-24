from amaranth.sim import Simulator

from test.build import Top

dut = Top()

udp_tst_packet = "428880aaef00000ec6e2420e080045000021422840004011d49b0a0008080a000801cda31b39000d24274b0b49ea0a"

def pktstr_to_rx(s:str):
    res = []
    res += [0x5]*15 + [0xd] # header
    
    sl = list(s)
    for i in range(0, len(s), 2):
        # swap 4:8 inside-byte endiannes
        sl[i:i+2] = (s[i+1], s[i])
    s = ''.join(sl)

    for c in s:
        res.append(int(c, 16))

    if len(s) < 64*2:
        res += [0x0]*(64*2-len(s))

    res += [0xc] * 8 # dummy crc

    return res


rx_data = pktstr_to_rx(udp_tst_packet)

def bench():
    for d in rx_data:
        yield dut.c.ethi.i_rx_clk.eq(~dut.c.ethi.i_rx_clk)
        yield  
        yield dut.c.ethi.i_rx_clk.eq(~dut.c.ethi.i_rx_clk)
        yield
        yield dut.c.ethi.i_rx_valid.eq(1)
        yield dut.c.ethi.i_rx_data.eq(d)

    for _ in range(30):
        yield dut.c.ethi.i_rx_clk.eq(~dut.c.ethi.i_rx_clk)
        yield  
        yield dut.c.ethi.i_rx_clk.eq(~dut.c.ethi.i_rx_clk)
        yield
        yield dut.c.ethi.i_rx_valid.eq(0)
    
    for d in rx_data:
        yield dut.c.ethi.i_rx_clk.eq(~dut.c.ethi.i_rx_clk)
        yield  
        yield dut.c.ethi.i_rx_clk.eq(~dut.c.ethi.i_rx_clk)
        yield
        yield dut.c.ethi.i_rx_valid.eq(1)
        yield dut.c.ethi.i_rx_data.eq(d)

    for _ in range(500):
        yield dut.c.ethi.i_rx_clk.eq(~dut.c.ethi.i_rx_clk)
        yield  
        yield dut.c.ethi.i_rx_clk.eq(~dut.c.ethi.i_rx_clk)
        yield   
        yield dut.c.ethi.i_rx_valid.eq(0)
        if (yield dut.c.ethi.o_tx_valid):
            print(hex((yield dut.c.ethi.o_tx_data))[2:], end="")
    print()
    

sim = Simulator(dut)

sim.add_clock(2e-8) # 50 MHz

sim.add_sync_process(bench)

with sim.write_vcd("trace.vcd"):
    sim.run()
