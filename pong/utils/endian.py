from amaranth import *

from transactron.core import TModule

def endian_reverse(m: TModule, v: Value) -> Signal:
    res = Signal.like(v)
    vw = v.shape().width //8 # type: ignore
    for i in range(vw): 
        m.d.comb += res.word_select(i, 8).eq(v.word_select(vw-i-1, 8))

    return res

        
def endian_reverse_record(m: TModule, r: Record) -> Record:
    res = Record.like(r)
    for n, s in res.fields.items():
        if s.shape().width % 8 == 0:
            m.d.comb += res[n].eq(endian_reverse(m, r[n]))
        else:
            m.d.comb += res[n].eq(r[n])

    return res
