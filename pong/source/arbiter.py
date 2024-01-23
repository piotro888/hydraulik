from amaranth import Elaboratable, Signal
from pong.common import STREAM_LAYOUT
from pong.proto import out

from transactron.core import Method, TModule, Transaction


class PriorityStreamArbiter(Elaboratable):
    def __init__(self, inputs: list[Method], output: Method):
        self.inputs = inputs
        self.output = output

    def elaborate(self, platform):
        m = TModule()

        
        in_progress = Signal()
        select_new = Signal()
        selected = Signal(range(len(self.inputs)))
        next_select = Signal.like(selected)

        for i, input in reversed(list(enumerate(self.inputs))):
            with m.If(input.ready):
                m.d.comb += next_select.eq(i)
    
        m.d.comb += select_new.eq(~in_progress)
        for i, input in enumerate(self.inputs):
            with Transaction().body(m, request=(selected == i)):
                in_res = input(m)
                self.output(m, in_res)

                m.d.sync += in_progress.eq(1)
                with m.If(in_res.end):
                    m.d.comb += select_new.eq(1)
                    m.d.sync += in_progress.eq(0)

        with m.If(select_new):
            m.d.sync += selected.eq(next_select)

        return m


