from typing import Protocol

from transactron import Method

from pong.common import STREAM_LAYOUT

class PacketSource(Protocol):
    out: Method

    def source_ctors(self):
        self.out = Method(o=STREAM_LAYOUT)
