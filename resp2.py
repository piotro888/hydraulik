from socket import *
import sys

s = socket(AF_PACKET, SOCK_RAW)

s.bind(("enp3s0f3u3u1", 0))

hexlist = "428880aaef01000ec6e2420e08004500001ee26640004011335f0a0009030a000902c636"+hex(int(sys.argv[1]) | 0x10000)[3:]+"000a25254f4b"
hexarr = []
for i in range(0, len(hexlist), 2):
    hexarr.append(int(hexlist[i:i+2], 16))
    #hexarr += [0]*

packet = bytearray(hexarr)

print(s.send(packet))
