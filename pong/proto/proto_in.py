from typing_extensions import Protocol

from pong.common import RX_LAYOUT

from transactron.core import Method
from transactron.utils._typing import MethodLayout

class ProtoIn(Protocol):
    push: Method
    clear: Method
    get: Method
    forward: Method
    GET_LAYOUT: MethodLayout

    def ctor(self):
        self.push = Method(i=RX_LAYOUT)
        self.clear = Method()
        self.get = Method(o=self.GET_LAYOUT, nonexclusive=True)
        self.forward = Method(o=RX_LAYOUT)

    
