from typing_extensions import Protocol

from pong.common import STREAM_LAYOUT

from transactron.core import Method
from transactron.utils._typing import MethodLayout

class ProtoIn(Protocol):
    push: Method
    clear: Method
    get: Method
    forward: Method
    GET_LAYOUT: MethodLayout

    def proto_in_ctors(self):
        self.push = Method(i=STREAM_LAYOUT)
        self.clear = Method()
        self.get = Method(o=self.GET_LAYOUT, nonexclusive=True)
        self.forward = Method(o=STREAM_LAYOUT)

    
