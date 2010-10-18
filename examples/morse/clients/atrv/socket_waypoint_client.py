#------------Setup--------------#
from socket import *
from cPickle import *

ServerIP = "localhost"

Serverport = 60000
sClient = socket(AF_INET,SOCK_DGRAM)
host = (ServerIP,Serverport)	
sClient.setblocking(0)

px = 0
py = 0
pz = 0
speed = 1

while True:

#--------- ASK FOR OPTIONS ----------------#

	print ("Select an option:")
	print ("a) Enter destination")
	print ("b) Enter speed")
	print ("c) Read coordinates")
	print ("q) Quit client program")
	op = raw_input("Enter option: ")
	
	if op == 'a':

#------------ASK FOR NEW COORDINATES--------------#	
		px = raw_input("Enter X coordinate: ")
		py = raw_input("Enter Y coordinate: ")
		pz = raw_input("Enter Z coordinate: ")

		waypoint = [float(px), float(py), float(pz), float(speed)]

		print ("Sending the command: {0}".format(waypoint))

# Sending
		Data = dumps((waypoint))
		sent = sClient.sendto(Data,host)

		print ("Just sent %d bytes to server" % sent)

	elif op == 'b':
		speed = raw_input("Enter speed: ")
		waypoint = [float(px), float(py), float(pz), float(speed)]
		print ("Sending the command: {0}".format(waypoint))

# Sending
		Data = dumps((waypoint))
		sent = sClient.sendto(Data,host)

		print ("Just sent %d bytes to server" % sent)


	elif op == 'c':
# Receiving
		try:
			print ("Trying to read data:")
			Data, SRIP = sClient.recvfrom(1024)
			#UPData = loads(Data)
			#PosServer = [UPData[0],UPData[1],UPData[2]]
			print ("Data read: {0}".format(Data))
		except:
			pass

	elif op == 'q':
		break

	else:
		print ("Unknown option. Try again.")

#---------------THE-END----------------#
