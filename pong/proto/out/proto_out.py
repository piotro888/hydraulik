from typing import Protocol
from pong.common import STREAM_LAYOUT

from transactron.core import Method


class ProtoOut(Protocol):
    push: Method
    
    def proto_out_ctors(self):
        self.push = Method(o=STREAM_LAYOUT)
        #Fix use of Connect in transactron first. 
        #For now it is more convinent to use less strict convention with __init__ in constructor
        #but If can be used instead of condtion inside without silently breaking things
        #self.forward = Method(i=STREAM_LAYOUT)
