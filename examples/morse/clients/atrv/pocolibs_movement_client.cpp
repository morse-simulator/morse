#include <string>
#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>

#include <posterLib.h>
#include <genPos/genPosStruct.h>

using namespace std;


POSTER_ID id;

void sigproc(int);
void usage (char* program_name);

int main(int argc, char* argv[])
{
	char cmd;
	char* poster_name = "p3dSpeedRef";

	GENPOS_CART_STATE*  poster_struct;

	poster_struct->v = 0.0;
	poster_struct->w = 0.0;

	STATUS s = posterCreate(poster_name, sizeof(GENPOS_CART_STATE), &id);
	if (s == ERROR)
	{
		// throw PosterCreateException(posterName);
		cout << "Could not create poster. Exiting!" << endl;
		return (1);
	}

	cout << " * KEYS: W/S: move forward/backward ; A/D: turn left/right ; any other key to stop." << endl;
	cout << " * Starting now..." << endl;

	while(true)
	{
		cout << "Enter cmd:"<<endl;

		cmd = cin.get();

		cout <<"cmd: "<<cmd<<endl;
		switch (cmd)
		{
			case 'i':
				poster_struct->v += 1.0;
				break;
			case 'k':
				poster_struct->v += 1.0;
				break;
			case 'j':
				poster_struct->w -= 1.0;
				break;
			case 'l':
				poster_struct->w += 1.0;
				break;
		}

		int err = posterWrite(id, 0, static_cast<void*>(poster_struct), sizeof(GENPOS_CART_STATE));
		if (err != sizeof(GENPOS_CART_STATE))
			// throw PosterWriteException<T> (posterName, err);
			printf ("Could not write the poster\n");

		//We catch ctrl+c to cleanly close the application
		signal( SIGINT,sigproc);
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
    printf ("Usage: %s [robot_name] [motion_component_name]\n", program_name);
    exit (1);
}
