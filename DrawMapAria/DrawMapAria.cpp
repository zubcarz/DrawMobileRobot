//add lib Aria MobilesRobot
#include "Aria.h"


void *printConsole(const char *message, int mode);

//Main Method
int main(int argc, char **argv)
{
	//init aria Obj
	Aria::init();
	ArArgumentParser parser(&argc, argv);
	parser.loadDefaultArguments();
	ArRobot robot;
	ArRobotConnector robotConnector(&parser, &robot);

	if (!robotConnector.connectRobot())
	{
		printConsole("teleopActionsExample: Could not connect to the robot.",1);
		if (parser.checkHelpAndWarnUnparsed())
		{
			Aria::logOptions();
			Aria::exit(1);
		}
	}
	if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
	{
		Aria::logOptions();
		Aria::exit(1);
	}
	printConsole("teleopActionsExample: Connected.", 2);

	// run the robot, true means that the run will exit if connection lost
	robot.run(true);

	Aria::exit(0);
	
}

// view message in console 
void *printConsole(const char *message, int mode) {
	printf("-------------------------------------\n");
	switch (mode) {
		// highlight message
	case 1:
		ArLog::log(ArLog::Terse, message);
		break;
	case 2:
		ArLog::log(ArLog::Normal, message);
		break;
	}
	printf("-------------------------------------\n");

	return NULL;
}

