from typing import Protocol
from pong.sink.sink import PacketSink
from pong.source.source import PacketSource


class ZlewZDiura(PacketSink, PacketSource, Protocol):
    def zlew_ctors(self):
        self.sink_ctors()
        self.source_ctors()
