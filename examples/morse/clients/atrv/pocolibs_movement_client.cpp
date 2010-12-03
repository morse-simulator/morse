#include <string>
#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>

#include <posterLib.h>
#include <genPos/genPosStruct.h>

using namespace std;

POSTER_ID id;

void usage (char* program_name);
void sigproc(int);

int main(int argc, char* argv[])
{
	char cmd;
	char*	poster_name;

	if (argc == 2)
	{
		poster_name = argv[1];
	}
	// If parameters are not given, use default values
	else if (argc == 1)
	{
		//poster_name = "OBATRV_OBMotion_Controller";
		//poster_name = "p3dSpeedRef";
		poster_name = "simu_locoSpeedRef";
	}
	else
	{
		usage (argv[0]);
		exit (1);
	}

	GENPOS_CART_SPEED  poster_struct;

	poster_struct.v = 0.0;
	poster_struct.w = 0.0;

	//We catch ctrl+c to cleanly close the application
	signal( SIGINT,sigproc);

	STATUS s = posterCreate(poster_name, sizeof(GENPOS_CART_SPEED), &id);
	if (s == ERROR)
	{
		// throw PosterCreateException(posterName);
		cout << "Could not create poster. Exiting!" << endl;
		return (1);
	}

    cout << "Poster '" << poster_name << "' created." << endl;

	cout << " * KEYS: i/k: move forward/backward ; j/l: turn left/right ; any other key to stop." << endl;
	cout << " * Starting now..." << endl;

	while(true)
	{
		cout << "Enter cmd:"<<endl;

		cmd = cin.get();

		cout <<"cmd: "<<cmd<<endl;
		switch (cmd)
		{
			case 'i':
				poster_struct.v += 1.0;
				break;
			case 'k':
				poster_struct.v -= 1.0;
				break;
			case 'j':
				poster_struct.w += 1.0;
				break;
			case 'l':
				poster_struct.w -= 1.0;
				break;
			default:
				continue;
		}
		printf ("Sending: v = %f, w = %f\n", poster_struct.v, poster_struct.w);

		int err = posterWrite(id, 0, static_cast<void*>(&poster_struct), sizeof(GENPOS_CART_SPEED));
		if (err != sizeof(GENPOS_CART_SPEED))
			// throw PosterWriteException<T> (posterName, err);
			printf ("Could not write the poster\n");
	}

}

void sigproc(int sig){
  signal(SIGINT, sigproc); /*  */
  /* NOTE some versions of UNIX will reset signal to default
     after each call. So for portability reset signal each time */
  posterDelete(id);
  cout << " * Exiting now!" << endl;
  cout << "*******************************" << endl;
  exit(0);
}


void usage (char* program_name)
{
    printf ("Usage: %s [genPos poster name]\n", program_name);
    exit (1);
}
