from socket import *

s = socket(AF_PACKET, SOCK_RAW)

s.bind(("enp3s0f3u3u1", 0))

hexlist = [0x1, 0x1, 0x1]
#hexlist = [0x12, 0x34, 0x56, 0x78, 0x9a, 0xff]
hexlist += [0]*60

packet = bytearray(hexlist)

print(s.send(packet))
