from typing import Protocol
from pong.frontend.parser import Parser

from transactron.core import Method


class PacketSink(Protocol):
    filter: Method # condition to request
    take: Method # get packet data (may be not ready). Parser will be cleared after call

    def sink_ctors(self):
        self.filter = Method(i=Parser.GET_LAYOUT, o=[("want",1)])
        self.take = Method(i=Parser.GET_LAYOUT)

