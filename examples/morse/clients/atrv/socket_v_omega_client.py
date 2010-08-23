#------------Setup--------------#
from socket import *
from cPickle import *

ServerIP = "localhost"

Serverport = 60000
sClient = socket(AF_INET,SOCK_DGRAM)
host = (ServerIP,Serverport)	
sClient.setblocking(0)

while True:

#--------- ASK FOR OPTIONS ----------------#

	print ("Select an option:")
	print ("a) Enter speed")
	print ("b) Read coordinates")
	print ("q) Quit client program")
	op = raw_input("Enter option: ")
	
	if op == 'a':

#------------ASK FOR NEW COORDINATES--------------#	
		v = raw_input("Enter V speed: ")
		w = raw_input("Enter W speed: ")
#locZ = raw_input("Enter Z coordinate: ")

		v_w = [float(v), float(w)]

		print ("Sending the command: {0}".format(v_w))

# Sending
		Data = dumps((v_w))
		sent = sClient.sendto(Data,host)

		print ("Just sent %d bytes to server" % sent)

	elif op == 'b':
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
