from amaranth import *
from ext.de2 import DE2_115Platform
from pong.pong import Pong
from transactron.core import TransactionModule


class Top(Elaboratable):
    def elaborate(self, platform):
        m = Module()
        tm = TransactionModule(m)
        m.submodules.c = Pong()
        return tm

if __name__ == "__main__":
    plat = DE2_115Platform()
    plat.build(Top(), do_program=True)
