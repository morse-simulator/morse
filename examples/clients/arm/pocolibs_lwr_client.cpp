#include <string>
#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>

#include <posterLib.h>
#include <gbM/gbStruct.h>
// #include <gbM/gbGENOM.h>
// #include <genManip/genManipStruct.h>
// #include <genBasic/genBasicStruct.h>
// #include <lwr/lwrStruct.h>



typedef struct LWR_ARM_INST {
    Gb_q7    currConf;
} LWR_ARM_INST;


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
        poster_name = "testLwrPoster";
        //poster_name = "lwrCurrentPoseArmRight";
	}
	else
	{
		usage (argv[0]);
		exit (1);
	}

    LWR_ARM_INST local_lwr;
    Gb_q7 local_conf;

    /*
	local_conf.q1 = 0.0;
	local_conf.q2 = 0.0;
	local_conf.q3 = 0.0;
	local_conf.q4 = 0.0;
	local_conf.q5 = 0.0;
	local_conf.q6 = 0.0;
	local_conf.q7 = 0.0;
    */

    double angle[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	//We catch ctrl+c to cleanly close the application
	signal( SIGINT,sigproc);

	STATUS s = posterCreate(poster_name, sizeof(LWR_ARM_INST), &id);
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
        for (int i=0; i<7; i++)
        {
            printf ("Enter [q%d]:\n", i+1);
            cin >> angle[i];
        }

        local_conf.q1 = angle[0];
        local_conf.q2 = angle[1];
        local_conf.q3 = angle[2];
        local_conf.q4 = angle[3];
        local_conf.q5 = angle[4];
        local_conf.q6 = angle[5];
        local_conf.q7 = angle[6];

        local_lwr.currConf = local_conf;

		printf ("Sending: [");
        /*
        for (int i=0; i<7; i++)
            printf ("%.4f ", angle[i]);
        */
        printf ("%.4f ", local_lwr.currConf.q1);
        printf ("%.4f ", local_lwr.currConf.q2);
        printf ("%.4f ", local_lwr.currConf.q3);
        printf ("%.4f ", local_lwr.currConf.q4);
        printf ("%.4f ", local_lwr.currConf.q5);
        printf ("%.4f ", local_lwr.currConf.q6);
        printf ("%.4f ", local_lwr.currConf.q7);

		printf ("]\n");

		int err = posterWrite(id, 0, static_cast<void*>(&local_lwr), sizeof(LWR_ARM_INST));
		if (err != sizeof(LWR_ARM_INST))
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
