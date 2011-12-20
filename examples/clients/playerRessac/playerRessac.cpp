#include <string>
#include <fstream>
#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>

using namespace std;
using namespace yarp::os;

BufferedPort<Bottle> toRessacPositionPort; //we send commands to RESSAC on this port
BufferedPort<Bottle> toRessacVelocityPort;
BufferedPort<Bottle> toRessacAttitudePort;
BufferedPort<Bottle> toTargetPositionPort; //we send commands to TARGET on this port
BufferedPort<Bottle> toTargetVelocityPort;

void sigproc(int);

int main(void){

  char cmd;

  /*
  string ressac_position_port = "/ors/robots/OBRessac/Motion_Controller/vxvyvz ";
  string ressac_rotation_port = "/ors/robots/OBRessac/Motion_Controller/vxvyvz ";

  //string target_position_port = "/ors/robots/OBATRV/Motion_Controller/destination";
  string target_position_port ="/ors/robots/OBATRV/Motion_Controller/vxvyvz";
  */

  string ressac_position_port = "/ors/robots/OBRessac/Position_Controller/position";
  string ressac_rotation_port = "/ors/robots/OBRessac/Position_Controller/angle";

  //string target_position_port = "/ors/robots/OBATRV/Motion_Controller/destination";
  string target_position_port ="/ors/robots/OBATRV/Position_Controller/position";
  string ressac_port = "ressac";
  string target_port = "target";


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
  toRessacPositionPort.open(("/" + ressac_port + "/position").c_str());
  toTargetPositionPort.open(("/" + target_port + "/position").c_str());


  Network::connect(toRessacPositionPort.getName().c_str(), ressac_position_port.c_str());

  Network::connect(toTargetPositionPort.getName().c_str(), target_position_port.c_str());


  cout << " * Writing commands to " << ressac_position_port << endl;
  cout << " * Writing commands to " << target_position_port << endl;

  cout << " * Starting now..." << endl;

  ifstream file;
  file.open("data_sync_vol4.txt", ifstream::in);

  //nextCmd=time(NULL);
  if(!file.is_open()){
    cout <<"File not found."<<endl;
  }
  else
  {
    cout <<"File opened."<<endl;
    double nextTime=3.5822500e+005;
    double prevTime=3.5822500e+005;
    double rx=0;
    double ry=0;
    double rz=0;
    double rvx=0;
    double rvy=0;
    double rvz=0;
    double rax=0;
    double ray=0;
    double raz=0;
    double rrx=0;
    double rry=0;
    double rrz=0;
    double tx=0;
    double ty=0;
    double tz=0;
    double tvx=0;
    double tvy=0;
    double tvz=0;

    double prrx=0;
    double prry=0;
    double prrz=0;

    double clockTime=0;
    while(!file.eof()){
      usleep((nextTime-prevTime)*1000.0);

      //	nextCmd=time(NULL);
      prevTime=nextTime;
      file >> nextTime;
      file >> rx;
      file >> ry;
      file >> rz;
      file >> rvx;
      file >> rvy;
      file >> rvz;
      file >> rax;
      file >> ray;
      file >> raz;
      file >> rrx;
      file >> rry;
      file >> rrz;
      file >> tx;
      file >> ty;
      file >> tz;
      file >> tvx;
      file >> tvy;
      file >> tvz;

      static double stx=tx;
      static double sty=ty;
      static double stz=tz;



      Bottle& cmdRessacBottle = toRessacPositionPort.prepare();
      cmdRessacBottle.clear();


      cmdRessacBottle.addDouble(rx);
      cmdRessacBottle.addDouble(ry);
      cmdRessacBottle.addDouble(rz);


      cmdRessacBottle.addDouble(rrx);
      cmdRessacBottle.addDouble(rry);
      cmdRessacBottle.addDouble(rrz);



      /*
      cmdBottle.addDouble(ax);
      cmdBottle.addDouble(ay);
      cmdBottle.addDouble(az);
      */

      toRessacPositionPort.write();

      cout << (nextTime-prevTime)<<" Send Ressac position        -> rx: " << rx << " ry: " << ry <<" rz: "<< rz <<endl;
      cout << (nextTime-prevTime)<<" Send Ressac rotation        -> ax: " << rrx-prrx << " ay: " << rry-prry <<" az: "<< rrz-prrz <<endl;

      prrx=rrx;
      prry=rry;
      prrz=rrz;

      Bottle& cmdTargetBottle = toTargetPositionPort.prepare();
      cmdTargetBottle.clear();

      cmdTargetBottle.addDouble(tvx);
      cmdTargetBottle.addDouble(-tvy);
      cmdTargetBottle.addDouble(tvz);


      /*    cmdBottle.addDouble(ax);
	    cmdBottle.addDouble(ay);
	    cmdBottle.addDouble(az);

      */
      toTargetPositionPort.write();

      cout << (nextTime-prevTime)<<" Send Target position        -> tx: " << tvx << " ty: " << tvy <<" tz: "<< tvz <<endl;


    }
    Bottle& cmdRessacBottle = toRessacPositionPort.prepare();
    cmdRessacBottle.clear();

    cmdRessacBottle.addDouble(0);
    cmdRessacBottle.addDouble(0);
    cmdRessacBottle.addDouble(0);

    cmdRessacBottle.addDouble(0);
    cmdRessacBottle.addDouble(0);
    cmdRessacBottle.addDouble(0);
    toRessacPositionPort.write();
  }
}

void sigproc(int sig){
  signal(SIGINT, sigproc); /*  */
  /* NOTE some versions of UNIX will reset signal to default
     after each call. So for portability reset signal each time */

  cout << " * Exiting now!" << endl;
  toRessacPositionPort.close();
  toRessacVelocityPort.close();
  toRessacAttitudePort.close();
  toTargetPositionPort.close();
  toTargetVelocityPort.close();
  Network::fini();
  cout << "*******************************" << endl;
  exit(0);
}
