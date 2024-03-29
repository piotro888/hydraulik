from amaranth import *
from ext.de2 import DE2_115Platform
from pong.pong import Pong
from transactron.core import TransactionModule


class Top(Elaboratable):
    def __init__(self):
        self.c = Pong()
    def elaborate(self, platform):
        m = Module()
        tm = TransactionModule(m)
        m.submodules.c = self.c
        return tm

if __name__ == "__main__":
    plat = DE2_115Platform()
    with open("pong/modules/ppcpu_soc.v") as f:
        plat.add_file("pong/modules/ppcpu_soc.v", f.read())
    plat.build(Top(), do_program=True)
