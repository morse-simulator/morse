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

BufferedPort<Bottle> toRessacPort; //we send commands to RESSAC on this port
BufferedPort<Bottle> fromRessacGPSPort; //we read status from RESSAC on this port
BufferedPort<Bottle> fromRessacAltitudePort;
BufferedPort<Bottle> fromRessacVelocityPort;
BufferedPort<Bottle> fromRessacAccelerationPort;


void sigproc(int);

int main(void) {

  char cmd;

  string ressac_motor_port = "/openrobots_simu/ressac/vxvyvz";
  string ressac_output_port_altitude = "/openrobots_simu/ressac/altitude";
  string ressac_output_port_gps = "/openrobots_simu/ressac/gps";
  string ressac_output_port_velocity = "/openrobots_simu/ressac/acc/vxvyvz";
  string ressac_output_port_acceleration = "/openrobots_simu/ressac/acc/axayaz";

  string local_port = "ressac_client";

  cout << "********* Ressac client *********" << endl;
  cout << "Display RESSAC drone gps position + move RESSAC in OpenRobots simulator" << endl;
  cout << "Press ctrl+c to exit." << endl;

  //We catch ctrl+c to cleanly close the application
  signal( SIGINT,sigproc);


  //setvbuf(stdin, NULL, _IONBF, 0);

  //Initialization of Yarp network
  Network::init();

  cout << "\n * YARP network initialized." << endl;

  //Connect to OpenRobots simulator
  toRessacPort.open(("/" + local_port + "/out").c_str());
  fromRessacGPSPort.open(("/" + local_port + "/in/gps").c_str());
  fromRessacAltitudePort.open(("/" + local_port + "/in/altitude").c_str());
  fromRessacVelocityPort.open(("/" + local_port + "/in/acc/vxvyvz").c_str());
  fromRessacVelocityPort.open(("/" + local_port + "/in/acc/axayaz").c_str());

  Network::connect(toRessacPort.getName().c_str(), ressac_motor_port.c_str());
  Network::connect(ressac_output_port_altitude.c_str() ,fromRessacAltitudePort.getName().c_str());
  Network::connect(ressac_output_port_gps.c_str() ,fromRessacGPSPort.getName().c_str());
  Network::connect(ressac_output_port_velocity.c_str() ,fromRessacVelocityPort.getName().c_str());
  Network::connect(ressac_output_port_acceleration.c_str() ,fromRessacAccelerationPort.getName().c_str());

  cout << " * Writing commands to " << ressac_motor_port << endl;
  cout << " * Listening status on " << ressac_output_port_altitude << endl;
  cout << " * Listening status on " << ressac_output_port_gps << endl;
  cout << " * Listening status on " << ressac_output_port_velocity << endl;
  cout << " * Listening status on " << ressac_output_port_acceleration << endl;

  cout << " * KEYS: T/G: move forward/backward ; F/H: turn left/right ; any other key to stop." << endl;
  cout << " * Starting now..." << endl;

  while(true){
    Bottle *incomingBottle;

    //read on the RESSAC output port, but don't wait.
    incomingBottle = fromRessacGPSPort.read(false);


    if (incomingBottle != NULL) {

      double x = incomingBottle->get(0).asDouble();
      double y = incomingBottle->get(1).asDouble();
      double z = incomingBottle->get(2).asDouble();
      cout << "Current RESSAC GPS position ->  x: " << x << "  y: " << y <<"  z: "<< z <<endl;
    }

    //read on the RESSAC output port, but don't wait.
    incomingBottle = fromRessacVelocityPort.read(false);

    if (incomingBottle != NULL) {

      double vx = incomingBottle->get(0).asDouble();
      double vy = incomingBottle->get(1).asDouble();
      double vz = incomingBottle->get(2).asDouble();
      cout << "Current RESSAC Velocity     -> vx: " << vx << " vy: " << vy <<" vz: "<< vz <<endl;
    }


    //read on the RESSAC output port, but don't wait.
    incomingBottle = fromRessacAccelerationPort.read(false);

    if (incomingBottle != NULL) {

      double ax = incomingBottle->get(0).asDouble();
      double ay = incomingBottle->get(1).asDouble();
      double az = incomingBottle->get(2).asDouble();
      cout << "Current RESSAC Acceleration -> ax: " << ax << " ay: " << ay <<" az: "<< az <<endl;
    }


    //read on the RESSAC output port, but don't wait.
    incomingBottle = fromRessacAltitudePort.read(false);

    if (incomingBottle != NULL) {
      double altitude = incomingBottle->get(0).asDouble();
      cout << "Current RESSAC altitude -> " << altitude <<endl;
    }


    cout << "Enter cmd:"<<endl;


    cmd = cin.get();

    cout <<"cmd: "<<cmd<<endl;
    Bottle& cmdBottle = toRessacPort.prepare();
    cmdBottle.clear();
    double vx=0.0,vy=0.0,vz=0.0;
    double ax=0.0,ay=0.0,az=0.0;
    switch (cmd){
    case 't':
      vx=1.0;
      break;
    case 'g':
      vx=-1.0;
      break;
    case 'f':
      vy=-1.0;
      break;
    case 'h':
      vy=1.0;
      break;
    }
    cmdBottle.addDouble(vx);
    cmdBottle.addDouble(vy);
    cmdBottle.addDouble(vz);

    cmdBottle.addDouble(ax);
    cmdBottle.addDouble(ay);
    cmdBottle.addDouble(az);
    cin.ignore();
    toRessacPort.write();

    cout << "SEND RESSAC velocity        -> vx: " << vx << " vy: " << vy <<" vz: "<< vz <<endl;
  }
}

void sigproc(int sig){
  signal(SIGINT, sigproc); /*  */
  /* NOTE some versions of UNIX will reset signal to default
     after each call. So for portability reset signal each time */

  cout << " * Exiting now!" << endl;
  toRessacPort.close();
  fromRessacAltitudePort.close();
  fromRessacGPSPort.close();
  fromRessacVelocityPort.close();
  fromRessacAccelerationPort.close();
  Network::fini();
  cout << "*******************************" << endl;
  exit(0);
}
