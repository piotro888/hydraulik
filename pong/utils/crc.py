from amaranth import *
from amaranth.hdl.ir import reduce
from transactron import *
import operator

# based on https://bues.ch/cms/hacking/crcgen.html  
CRC_TABLE = [
[2, 8],
[0, 3, 9],
[0, 1, 4, 10],
[1, 2, 5, 11],
[0, 2, 3, 6, 12],
[1, 3, 4, 7, 13],
[4, 5, 14],
[0, 5, 6, 15],
[1, 6, 7, 16],
[7, 17],
[2, 18],
[3, 19],
[0, 4, 20],
[0, 1, 5, 21],
[1, 2, 6, 22],
[2, 3, 7, 23],
[0, 2, 3, 4, 24],
[0, 1, 3, 4, 5, 25],
[0, 1, 2, 4, 5, 6, 26],
[1, 2, 3, 5, 6, 7, 27],
[3, 4, 6, 7, 28],
[2, 4, 5, 7, 29],
[2, 3, 5, 6, 30],
[3, 4, 6, 7, 31],
[0, 2, 4, 5, 7],
[0, 1, 2, 3, 5, 6],
[0, 1, 2, 3, 4, 6, 7],
[1, 3, 4, 5, 7],
[0, 4, 5, 6],
[0, 1, 5, 6, 7],
[0, 1, 6, 7],
[1, 7]
]

class CRC(Elaboratable):
    def __init__(self):
        self.push = Method(i=[("data", 8)])
        self.get = Method(o=[("data",32)])
        self.clear = Method()

    def elaborate(self, platform):
        m = TModule()
        
        crc = Signal(32)
        new_crc = Signal(32)
        @def_method(m, self.push)
        def _(data):
            for i, e in enumerate(CRC_TABLE):
                elements = [crc.bit_select(x, 1) for x in e]
                elements += [(data.bit_select(x, 1) if x < 8 else 0) for x in e]
                entry = reduce(operator.xor, elements)
                m.d.comb += new_crc.bit_select(i, 1).eq(entry)
            m.d.sync += crc.eq(new_crc)

        @def_method(m, self.get)
        def _():
            return ~crc
        
        @def_method(m, self.clear)
        def _():
            m.d.sync += crc.eq(0xFFFFFFFF)

        return  m
