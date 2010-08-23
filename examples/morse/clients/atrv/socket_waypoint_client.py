#------------Setup--------------#
from socket import *
from cPickle import *

ServerIP = "140.93.0.93"

Serverport = 60000
Clientname = ''
ClientPort = 10001
sClient = socket(AF_INET,SOCK_DGRAM)
host = (ServerIP,Serverport)	
sClient.setblocking(0)



#------------ASK FOR NEW COORDINATES--------------#	
px = raw_input("Enter X coordinate: ")
py = raw_input("Enter Y coordinate: ")
pz = raw_input("Enter Z coordinate: ")
speed = raw_input("Enter speed: ")
#locZ = raw_input("Enter Z coordinate: ")

waypoint = [float(px), float(py), float(pz), float(speed)]

print ("Sending the command: {0}".format(waypoint))

#------------RECEIVE/SEND--------------#	
Data = dumps((waypoint))
sent = sClient.sendto(Data,host)

print ("Just sent %d bytes to server" % sent)

try:
	Data, SRIP = sClient.recvfrom(1024)
	UPData = loads(Data)
	PosServer = [UPData[0],UPData[1],UPData[2]]
	print ("The angle of the robot: {0}".format(PosServer))
except:
	pass
#---------------THE-END----------------#
