#include <string>
#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <unistd.h>


#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>

#define X 0
#define Y 1
#define Z 2

using namespace std;
using namespace yarp::os;

BufferedPort<Bottle> LocalAdminInPort; // Port to receive data from Blender
BufferedPort<Bottle> LocalAdminOutPort; // Port to receive data from Blender


void sigproc(int);
void usage (char* program_name);

int main(int argc, char* argv[])
{
	// No parameter needed (for the moment)
	if (argc != 1)
	{
		usage (argv[0]);
		exit (1);
	}

	string command;
	string robot_list;
	bool waiting = true;

	Bottle *incomingBottle;

	// Define the names of the ports opened by this program
	string local_admin_in_port = "/scene/admin/in/";
	string local_admin_out_port = "/scene/admin/out/";

	// Define the names for the comunication ports opened by Blender
	string ors_admin_in_port = "/ors/admin/in";
	string ors_admin_out_port = "/ors/admin/out";

	cout << "********* Scene Initialization Client *********" << endl;
	cout << "Get the list of robots provided by the ORS" << endl;
	cout << "Press ctrl+c to exit." << endl;

	//We catch ctrl+c to cleanly close the application
	signal( SIGINT,sigproc);

	//setvbuf(stdin, NULL, _IONBF, 0);

	//Initialization of Yarp network
	Network::init();

	cout << "\n * YARP network initialized." << endl;

	// Connect to Open Robots Simulator
	LocalAdminInPort.open(local_admin_in_port.c_str());
	LocalAdminOutPort.open(local_admin_out_port.c_str());

	Network::connect(LocalAdminOutPort.getName().c_str(), ors_admin_in_port.c_str());
	Network::connect( ors_admin_out_port.c_str(), LocalAdminInPort.getName().c_str());

	cout << " * Receiving ORS communication port: " << ors_admin_in_port << endl;
	cout << " * Transmitting ORS communication port: " << ors_admin_out_port << endl;

	cout << " * Requesting list of robots..." << endl;

	// Prepare the data bottle to send
    Bottle& cmdBottle = LocalAdminOutPort.prepare();
    cmdBottle.clear();
    cmdBottle.addString("list_robots");
    // cin.ignore();
    LocalAdminOutPort.write();


	cout << " * Waiting for the reply..." << endl;

	while(waiting)
	{
		// Read on the Blender output port, but don't wait.
		incomingBottle = LocalAdminInPort.read(false);

		if (incomingBottle != NULL)
		{
			cout << " RESPONSE: " <<  incomingBottle->toString().c_str() << endl;
			robot_list = (std::string) incomingBottle->toString();

			if (! (robot_list == "") )
				waiting = false;
		}
	}

	cout << " * Got the list of scene elements: " << robot_list << endl;
}

void sigproc(int sig){
  signal(SIGINT, sigproc); /*  */
  /* NOTE some versions of UNIX will reset signal to default
     after each call. So for portability reset signal each time */

  cout << " * Exiting now!" << endl;
  LocalAdminOutPort.close();
  LocalAdminInPort.close();
  Network::fini();
  cout << "*******************************" << endl;
  exit(0);
}

void usage (char* program_name)
{
    printf ("Usage: %s [robot_name] [number_of_cameras]\n", program_name);
    exit (1);
}
