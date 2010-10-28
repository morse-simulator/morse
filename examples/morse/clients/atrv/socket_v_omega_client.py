import sys
import socket
import cPickle


server_ip = "localhost"
server_port = 60000
connected = False

def read_data(client_socket):
	""" Read the input socket until no more data is available """
	finished = False
	data_in = ''

	if connected:
		# Receiving
		while not finished:
			try:
				data_in, SRIP = client_socket.recvfrom(1024)
				#print ("READ: {0}".format(data_in))
			except socket.error as detail:
				finished = True
				#print ("Socket error: %s" % detail)
				continue

		return data_in


def print_data(data):
	""" Choose how to print, depending on the type """
	for item in data:
		# Recursively call this function if item is a list
		if isinstance(item, list):
			print_data(item)
		if isinstance(item, basestring):
			print ("\t%s" % item)
		if isinstance(item, float):
			print ("\t%.4f" % item)


def usage(program_name):
	print ("Usage: {0} [server_port_number]\n", program_name)


def main():
	global server_port
	global connected

	# Read the arguments
	argc = len(sys.argv)
	if argc == 2:
		server_port = int(sys.argv[1])
	elif argc > 3:
		usage(sys.argv[0])
		sys.exit()

	client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	host = (server_ip, server_port)	
	client_socket.setblocking(0)


	while True:

	#--------- ASK FOR OPTIONS ----------------#

		print ("Select an option:")
		print ("a) Enter speed")
		print ("b) Read coordinates")
		print ("q) Quit client program")
		op = raw_input("Enter option: ")
		
		if op == 'a':
			# Ask for the new speeds
			v = raw_input("Enter V speed: ")
			w = raw_input("Enter W speed: ")

			v_w = [float(v), float(w)]
			print ("Sending the command: {0}".format(v_w))

			# Send the data
			data_out = cPickle.dumps((v_w))
			sent = client_socket.sendto(data_out,host)

			print ("Just sent %d bytes to server" % sent)
			connected = True

		elif op == 'b':
			data_in = read_data(client_socket)
			try:
				pickled_data = cPickle.loads(data_in)
				print_data(pickled_data)
			except EOFError as detail:
				print ("\tNo data available for the moment")

		elif op == 'q':
			break

		else:
			print ("Unknown option. Try again.")



if __name__ == "__main__":
    main()
