from socket import *
import random


sock = socket(AF_INET, SOCK_DGRAM)

#s = socket(AF_PACKET, SOCK_RAW, IPPROTO_UDP)
data = [random.randint(0, 255) for _ in range(random.randrange(5, 20))] 

sport = 8842
dport = 6969
dip = "10.0.8.1"
#length = 8+len(data)
#csum = 0
#udp_header = struct.pack('!HHHH', sport, dport, length, checksum)


sock.sendto(bytearray(data), (dip, dport))

