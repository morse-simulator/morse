#include <string>
#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>

#include <posterLib.h>
#include <pom/pomStruct.h>

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
        poster_name = "simu_platineRef";
	}
	else
	{
		usage (argv[0]);
		exit (1);
	}

	POM_SE_POSTER  poster_struct;
    POM_EULER_V euler_v_struct;
    POM_EULER euler_struct;

	euler_struct.yaw = 0.0;
	euler_struct.pitch = 0.0;
    double pan = 0.0;
    double tilt = 0.0;

	//We catch ctrl+c to cleanly close the application
	signal( SIGINT,sigproc);

	STATUS s = posterCreate(poster_name, sizeof(POM_SE_POSTER), &id);
	if (s == ERROR)
	{
		// throw PosterCreateException(posterName);
		cout << "Could not create poster. Exiting!" << endl;
		return (1);
	}

    cout << "Poster '" << poster_name << "' created." << endl;
	cout << " * Starting now..." << endl;

	while(true)
	{
		cout << "Enter [pan]:"<<endl;
		cin >> pan;
		cout << "Enter [tilt]:"<<endl;
		cin >> tilt;

        euler_struct.yaw = pan;
        euler_struct.pitch = tilt;
		printf ("Sending: pan = %f, tilt = %f\n", euler_struct.yaw, euler_struct.pitch);
        euler_v_struct.euler = euler_struct;
        poster_struct.seConfig = euler_v_struct;

		int err = posterWrite(id, 0, static_cast<void*>(&poster_struct), sizeof(POM_SE_POSTER));
		if (err != sizeof(POM_SE_POSTER))
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
