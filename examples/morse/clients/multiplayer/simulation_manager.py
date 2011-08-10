# Based on an example from:
#  http://ilab.cs.byu.edu/python/threadingmodule.html

"""
Simulation Manager that coordinates multiple MORSE nodes.
Uses threads to handle multiple clients at a time.
Entering any line of input at the terminal will exit the server.
"""

import select
import socket
import sys
import threading
import pickle
import time

counter = 0
client_counter = 0
client_list = {}
robot_list = {}


class Server:
    def __init__(self):
        self.host = ''
        self.port = 65000
        self.backlog = 5
        self.size = 1024
        self.server = None
        self.threads = []

        print ("Server listening on port %d" % self.port)


    def open_socket(self):
        try:
            self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server.bind((self.host,self.port))
            self.server.listen(5)
        except socket.error as xxx_todo_changeme:
            (value,message) = xxx_todo_changeme.args
            if self.server:
                self.server.close()
            print("Could not open socket: " + message)
            sys.exit(1)

    def run(self):
        self.open_socket()
        input = [self.server,sys.stdin]
        running = 1
        while running:
            inputready,outputready,exceptready = select.select(input,[],[])

            for s in inputready:

                if s == self.server:
                    # handle the server socket
                    c = Client(self.server.accept())
                    c.start()
                    self.threads.append(c)

                elif s == sys.stdin:
                    # handle standard input
                    junk = sys.stdin.readline()
                    running = 0 

        # close all threads

        self.server.close()
        for c in self.threads:
            c.join()

class Client(threading.Thread):
    def __init__(self, xxx_todo_changeme1):
        (client,address) = xxx_todo_changeme1
        threading.Thread.__init__(self)
        self.client = client
        self.address = address
        self.size = 1024
        self.start_time = time.time()

    def run(self):
        global counter
        global client_counter
        global client_list
        global robot_list


        running = 1
        while running:
            data = self.client.recv(self.size)
            if data:
                data = pickle.loads(data)
                client_name = data[0]
                client_robot_list = data[1]

                counter = counter + 1

                # Add a new client to the list
                if client_name not in client_list:
                    client_counter = client_counter + 1
                    #client_list[client_name] = False
                #else:
                    #client_list[client_name] = True
                client_list[client_name] = True
                

                # Build/update the list of robots from the data received from all the clients
                for robot_name, robot_position in client_robot_list.items():
                    robot_list[robot_name] = robot_position

                response = pickle.dumps(robot_list)

                #print ("New robot list: ", robot_list)
                #print ("Client %d, connected from %s, has been served" % (counter, self.address))

                reception_time = time.time() - self.start_time
                print ("Received data at time: %.6f" % reception_time)
                time.sleep(0.5)
                self.client.send(response)

                """
                if self._clients_have_replied (client_list):
                    self.client.send(response)
                    # Reset the counter for the client updates
                    for key in client_list.keys():
                        client_list[key] = False
                else:
                    print ("Still waiting for some clients")
                """

            else:
                self.client.close()
                running = 0


    def _clients_have_replied (self, client_list):
        """ Check if all the current clients have updated their data """
        print ("Cliet check:")
        for key, value in client_list.items():
            print ("%s = %s" % (key, value))
            if value == False:
                return False
        return True


if __name__ == "__main__":
    print ("Simulation Manager started...")
    print ("Enter any input in this terminal to terminate the server")
    s = Server()
    s.run()
