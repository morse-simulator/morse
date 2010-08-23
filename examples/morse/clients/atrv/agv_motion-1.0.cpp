#include <string>
#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>


#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>

using namespace std;
using namespace yarp::os;

BufferedPort<Bottle> toUGVPort; //we send commands to the UGV on this port
BufferedPort<Bottle> fromUGVGPSPort; //we read status from the UGV on this port


void sigproc(int);
void usage (char* program_name);

int main(int argc, char* argv[]) {

  string	robot_name;
  bool		connected = false;

  // Use parameters as the name of the robot and the component
  if (argc == 2)
  {
    robot_name = argv[1];
  }
  // If parameters are not given, use default values
  else if (argc == 1)
  {
    robot_name = "OBATRV";
  }
  else
  {
    usage (argv[0]);
    exit (1);
  }

  char cmd;

  string port_prefix = "/ors/robots/" + robot_name + "/";

  string ugv_motor_port = "/ors/robots/OBATRV/OBMotion_Controller/in";

  string local_port_prefix = "/ugv_client/" + robot_name;

  cout << "********* UGV client *********" << endl;
  cout << "Display the UGV drone gps position + move the UGV in OpenRobots simulator" << endl;
  cout << "Press ctrl+c to exit." << endl;

  //We catch ctrl+c to cleanly close the application
  signal( SIGINT,sigproc);


  //setvbuf(stdin, NULL, _IONBF, 0);

  //Initialization of Yarp network
  //Network::init();
  Network yarp_object;

  cout << "\n * YARP network initialized." << endl;

  //Connect to OpenRobots simulator
  toUGVPort.open((local_port_prefix + "/motion/out").c_str());
  fromUGVGPSPort.open((local_port_prefix + "/gps/in").c_str());

  //connected = Network::connect(toUGVPort.getName().c_str(), ugv_motor_port.c_str());
  connected = yarp_object.connect(toUGVPort.getName().c_str(), ugv_motor_port.c_str());

  if (!connected)
  {
	  printf ("\nClient ERROR: Ports not found. Quitting\n");
	  exit (1);
  }


  cout << " * Writing commands to " << ugv_motor_port << endl;

  cout << " * KEYS: W/S: move forward/backward ; A/D: turn left/right ; any other key to stop." << endl;
  cout << " * Starting now..." << endl;

  double vx=0.0,vy=0.0,vz=0.0;
  double ax=0.0,ay=0.0,az=0.0;
  double rx=0.0,ry=0.0,rz=0.0;

  while(true)
  {
    Bottle *incomingBottle;

    //read on the UGV output port, but don't wait.
    incomingBottle = fromUGVGPSPort.read(false);


    if (incomingBottle != NULL)
    {
      double x = incomingBottle->get(0).asDouble();
      double y = incomingBottle->get(1).asDouble();
      double z = incomingBottle->get(2).asDouble();
      cout << "Current UGV GPS position ->  x: " << x << "  y: " << y <<"  z: "<< z <<endl;
    }

/*
    //read on the UGV output port, but don't wait.
    incomingBottle = fromUGVVelocityPort.read(false);

    if (incomingBottle != NULL) {

      double vx = incomingBottle->get(0).asDouble();
      double vy = incomingBottle->get(1).asDouble();
      double vz = incomingBottle->get(2).asDouble();
      cout << "Current UGV Velocity     -> vx: " << vx << " vy: " << vy <<" vz: "<< vz <<endl;
    }


    //read on the UGV output port, but don't wait.
    incomingBottle = fromUGVAccelerationPort.read(false);

    if (incomingBottle != NULL) {

      double ax = incomingBottle->get(0).asDouble();
      double ay = incomingBottle->get(1).asDouble();
      double az = incomingBottle->get(2).asDouble();
      cout << "Current UGV Acceleration -> ax: " << ax << " ay: " << ay <<" az: "<< az <<endl;
    }
*/

    cout << "Enter cmd:"<<endl;

    cmd = cin.get();

    cout <<"cmd: "<<cmd<<endl;
    Bottle& cmdBottle = toUGVPort.prepare();
    cmdBottle.clear();
    switch (cmd)
	{
		case 'i':
		  vx += 1.0;
		  break;
		case 'k':
		  vx -= 1.0;
		  break;
		case 'j':
		  rz -= 1.0;
		  break;
		case 'l':
		  rz += 1.0;
		  break;
    }
    cmdBottle.addDouble(vx);
    // cmdBottle.addDouble(vy);
    // cmdBottle.addDouble(vz);

    /*
    cmdBottle.addDouble(ax);
    cmdBottle.addDouble(ay);
    cmdBottle.addDouble(az);
    */

    // cmdBottle.addDouble(rx);
    // cmdBottle.addDouble(ry);
    cmdBottle.addDouble(rz);

    cin.ignore();
    toUGVPort.write();

    cout << "Send UGV velocity        -> vx: " << vx << " vy: " << vy <<" vz: "<< vz <<endl;
    cout << "Send UGV rotation        -> rx: " << rx << " ry: " << ry <<" rz: "<< rz <<endl;
  }
}

void sigproc(int sig){
  signal(SIGINT, sigproc); /*  */
  /* NOTE some versions of UNIX will reset signal to default
     after each call. So for portability reset signal each time */

  cout << " * Exiting now!" << endl;
  toUGVPort.close();
  fromUGVGPSPort.close();
  //Network::fini();
  cout << "*******************************" << endl;
  exit(0);
}

void usage (char* program_name)
{
    printf ("Usage: %s [robot_name] [motion_component_name]\n", program_name);
    exit (1);
}
