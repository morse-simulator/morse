#------------Setup--------------#
from socket import *
from cPickle import *

ServerIP = "140.93.0.93"

Serverport = 70000
Clientname = ''
ClientPort = 10001
sClient = socket(AF_INET,SOCK_DGRAM)
host = (ServerIP,Serverport)	
sClient.setblocking(0)



#------------ASK FOR NEW COORDINATES--------------#	
v = raw_input("Enter V coordinate: ")
w = raw_input("Enter W coordinate: ")
#locZ = raw_input("Enter Z coordinate: ")

v_w = [float(v), float(w)]

print ("Sending the command: {0}".format(v_w))

#------------RECEIVE/SEND--------------#	
Data = dumps((v_w))
sent = sClient.sendto(Data,host)

print ("Just sent %d bytes to server" % sent)

try:
	Data, SRIP = sClient.recvfrom(1024)
	UPData = loads(Data)
	PosServer = [UPData[0],UPData[1],UPData[2]]
	print ("The position of the server: {0}".format(PosServer))
except:
	pass
#---------------THE-END----------------#
