//probe cicle 2
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


	//Limites de velocidad y rotacion en base a los datos recibidos por los sensores de ultrasonido

	// limiter for close obstacles
	ArActionLimiterForwards limiter("speed limiter near", 300, 600, 250);
	// limiter for far away obstacles
	ArActionLimiterForwards limiterFar("speed limiter far", 300, 1100, 400);
	// limiter that checks IR sensors (like Peoplebot has)
	ArActionLimiterTableSensor tableLimiter;
	// limiter so we don't bump things backwards
	ArActionLimiterBackwards backwardsLimiter;
	// the joydrive action
	ArActionJoydrive joydriveAct;
	// the keydrive action
	ArActionKeydrive keydriveAct;

	// sonar device, used by the limiter actions.
	ArSonarDevice sonar;


	if (!joydriveAct.joystickInited())
		printConsole("Do not have a joystick, only the arrow keys on the keyboard will work.",3);

	// add the sonar to the robot
	robot.addRangeDevice(&sonar);
	robot.setAbsoluteMaxTransVel(400);

	// enable the motor
	robot.enableMotors();

	//add actions
	robot.addAction(&tableLimiter, 100);
	robot.addAction(&limiter, 95);
	robot.addAction(&limiterFar, 90);
	robot.addAction(&backwardsLimiter, 85);
	robot.addAction(&joydriveAct, 50);
	//add key actions
	robot.addAction(&keydriveAct, 45);

	//priority low
	joydriveAct.setStopIfNoButtonPressed(false);

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
	case 3:
		printf(message);
		printf("/n");
		break;
	}
	printf("-------------------------------------\n");

	return NULL;
}

