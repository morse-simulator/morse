#------------Setup--------------#
from socket import *
from cPickle import *

ServerIP = "140.93.0.93"

Serverport = 70000
Clientname = ''
ClientPort = 90001
sClient = socket(AF_INET,SOCK_DGRAM)
host = (ServerIP,Serverport)	
sClient.setblocking(0)


#sReply = socket(AF_INET,SOCK_DGRAM)
#host = (ServerIP,ClientPort)	
#sReply.setblocking(0)


#------------ASK FOR NEW COORDINATES--------------#	
v = raw_input("Enter V speed: ")
w = raw_input("Enter W speed: ")
#locZ = raw_input("Enter Z coordinate: ")

v_w = [float(v), float(w)]

print ("Sending the command: {0}".format(v_w))

#------------RECEIVE/SEND--------------#	
# Sending
Data = dumps((v_w))
sent = sClient.sendto(Data,host)

print ("Just sent %d bytes to server" % sent)

# Receiving
"""
try:
	Data, SRIP = sReply.recvfrom(1024)
	UPData = loads(Data)
	read_data = [UPData[0],UPData[1],UPData[2]]
	print ("The data read from MORSE: {0}".format(read_data))
except:
	pass
"""
#---------------THE-END----------------#
