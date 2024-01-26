from socket import *
import random


sock = socket(AF_INET, SOCK_DGRAM)

#s = socket(AF_PACKET, SOCK_RAW, IPPROTO_UDP)
data = [random.randint(0, 255) for _ in range(random.randrange(5, 20))] 

dport = 4242
#dport = 6969
#ip = "10.0.8.1"
dip = "10.0.9.3"
#length = 8+len(data)
#csum = 0
#udp_header = struct.pack('!HHHH', sport, dport, length, checksum)


sock.sendto(bytearray(data), (dip, dport))

