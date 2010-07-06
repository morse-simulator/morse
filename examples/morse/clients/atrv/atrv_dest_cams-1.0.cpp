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

BufferedPort<Bottle> toATRVPort; //we send commands to the ATRV on this port
BufferedPort<Bottle> fromATRVGPSPort; //we read GPS from the ATRV on this port


void sigproc(int);
void usage (char* program_name);

int main(int argc, char* argv[]) {

  string	robot_name;
  string	component_name;
  int		num_cameras;
  bool		connected = false;

  // Use parameters as the name of the robot and the component
  if (argc == 3)
  {
    robot_name = argv[1];
    num_cameras = atoi(argv[2]);
    // component_name = argv[2];
  }
  // If parameters are not given, use default values
  else if (argc == 1)
  {
    robot_name = "OBATRV";
    num_cameras = 0;
  }
  else
  {
    usage (argv[0]);
    exit (1);
  }

  char cmd;

  string port_prefix = "/ors/robots/" + robot_name + "/";
  string view_prefix = "/img/" + robot_name + "/";
  string yarpview_path = "/home/gechever/openrobots/bin/yarpview";

  // Define the names for the comunication ports
  string atrv_motor_port = port_prefix + "OBMotion_Controller/in";
  string atrv_output_port_gps = port_prefix + "OBGPS/out";

  string local_port_prefix = "/atrv_client/" + robot_name;

  cout << "********* ATRV client *********" << endl;
  cout << "Display the ATRV drone gps position + move the ATRV in OpenRobots simulator" << endl;
  cout << "Press ctrl+c to exit." << endl;

  //We catch ctrl+c to cleanly close the application
  signal( SIGINT,sigproc);


  //setvbuf(stdin, NULL, _IONBF, 0);

  //Initialization of Yarp network
  Network::init();

  cout << "\n * YARP network initialized." << endl;


  // Start the yarpview windows and connect them the corresponding ports
  cout << " * Initializing yarpview windows." << endl;
  for (int i=0; i<num_cameras; i++)
  {
	  // Convert the integer to string
	  std::stringstream out;
	  out << i+1;

	  // Prepare the ports to be used
	  string img_view_port = view_prefix + "Camera" + out.str();
	  string atrv_camera_port = port_prefix + "Camera" + out.str();

	  cout << "\tConnecting: " << img_view_port << " to " << atrv_camera_port << endl;

	  // string command = yarpview_path + " " + img_view_port;
	  // system (command.c_str());
	  // int result = execlp("eog", "/home/gechever/Desktop/dala_blender_links.png");
	  // int result = execl(yarpview_path.c_str(), atrv_camera_port.c_str(), img_view_port.c_str());
	  // cout << "EXECUTION OF execl RETURNED: " << result << endl;

	  Network::connect( atrv_camera_port.c_str(), img_view_port.c_str() );
  }


  //Connect to OpenRobots simulator
  toATRVPort.open((local_port_prefix + "/out/destination").c_str());
  fromATRVGPSPort.open((local_port_prefix + "/in/gps").c_str());

  connected = Network::connect(toATRVPort.getName().c_str(), atrv_motor_port.c_str());
  if (!connected)
  {
	  printf ("\nClient ERROR: Port '%s' not found. Quitting\n", atrv_motor_port.c_str());
	  exit (1);
  }

  connected = Network::connect(atrv_output_port_gps.c_str() ,fromATRVGPSPort.getName().c_str());
  if (!connected)
  {
	  printf ("\nClient WARNING: Port '%s' not found. Quitting\n", atrv_output_port_gps.c_str());
	  // exit (1);
  }

  cout << " * Writing commands to " << atrv_motor_port << endl;
  cout << " * Listening status on " << atrv_output_port_gps << endl;

  cout << " * Enter destination coordinates, separated by a space" << endl;
  cout << " * Example: 1.52 3.38 0.0" << endl;
  cout << " * NOTE: The third coordinate (Z) is not used in the case of ATRV" << endl;

  cout << " * Starting now..." << endl;



  double dest[3] = {0.0, 0.0, 0.0};

  while(true)
  {
    Bottle *incomingBottle;

    //read on the ATRV output port, but don't wait.
    incomingBottle = fromATRVGPSPort.read(false);


    if (incomingBottle != NULL)
    {
      double x = incomingBottle->get(0).asDouble();
      double y = incomingBottle->get(1).asDouble();
      double z = incomingBottle->get(2).asDouble();
      cout << "Current ATRV GPS position ->  x: " << x << "  y: " << y <<"  z: "<< z <<endl;
    }

    cout << "Enter destination coordinates:"<<endl;

	cin >> dest[X] >> dest[Y] >> dest[Z];

    cout << "Destination: " << dest[X] << ", " << dest[Y] << ", " << dest[Z] << endl;

	// Prepare the data bottle to send
    Bottle& cmdBottle = toATRVPort.prepare();
    cmdBottle.clear();
    cmdBottle.addDouble(dest[X]);
    cmdBottle.addDouble(dest[Y]);
    cmdBottle.addDouble(dest[Z]);

    cin.ignore();
    toATRVPort.write();
  }
}

void sigproc(int sig){
  signal(SIGINT, sigproc); /*  */
  /* NOTE some versions of UNIX will reset signal to default
     after each call. So for portability reset signal each time */

  cout << " * Exiting now!" << endl;
  toATRVPort.close();
  fromATRVGPSPort.close();
  Network::fini();
  cout << "*******************************" << endl;
  exit(0);
}

void usage (char* program_name)
{
    printf ("Usage: %s [robot_name] [number_of_cameras]\n", program_name);
    exit (1);
}
