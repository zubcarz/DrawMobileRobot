//add lib Aria MobilesRobot
#include "Aria.h"
//include operation with strings
#include <string.h>
//include inn and out console
#include <iostream>
//include lib of time
#include <ctime>
#include <cstdlib>
// constructor create files of text
#include <fstream>  

using namespace std;

void *printConsole(const char *message, int mode);
void *delay(const char *wait);
void *printParametersRobot(void);
void *writeDraw(int id, float time, double posX, double posY, double angTh);
void *writeHead();
void *clearOutput();

static ArRobot robot;
static int countRegisters;
static float timeRegister;
static const char secondsToDelay[] = "0.5";

//Main Method
int main(int argc, char **argv)
{
	//init aria Obj
	Aria::init();
	ArArgumentParser parser(&argc, argv);
	parser.loadDefaultArguments();
	//robot connector
	ArRobotConnector robotConnector(&parser, &robot);
	//laser connector
	ArLaserConnector laserConnector(&parser, &robot, &robotConnector);

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

	// Connector for compasses
	ArCompassConnector compassConnector(&parser);

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

	// subcribe Analog Gyro
	ArAnalogGyro gyro(&robot);


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
	robot.runAsync(true);


	if (!laserConnector.connectLasers())
	{
		ArLog::log(ArLog::Terse, "Could not connect to configured lasers. Exiting.");
		Aria::exit(3);
		return 3;
	}


	ArLog::log(ArLog::Normal, "Connected to all lasers.");

	writeHead();
	
	//print sample time 
	std::cout << "Time to delay: " << secondsToDelay << std::endl;
	
	while (true) {
		delay(secondsToDelay);
	}

	// Block execution of the main thread here and wait for the robot's task loop
	// thread to exit (e.g. by robot disconnecting, escape key pressed, or OS
	// signal)
	robot.waitForRunExit();
	
	Aria::exit(0);
	return 0;
}

// view message in console 
void *printConsole(const char *message, int mode = 3) {
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
		printf("\n");
		break;
	}
	printf("-------------------------------------\n");

	return NULL;
}

// view message in console 
void *printConsole(string message, int mode = 3) {
	printf("-------------------------------------\n");
	switch (mode) {
	case 3:
		printf("Follow this command: %s", message);
		printf("\n");
		break;
	}
	printf("-------------------------------------\n");

	return NULL;
}

void *delay(const char *wait) {
	clock_t startTime = clock(); //Start timer
	double secondsPassed;
	double secondsToDelay = atof(wait);
	
	bool flag = true;

	while (flag)
	{
		secondsPassed = (clock() - startTime) / CLOCKS_PER_SEC;
		if (secondsPassed >= secondsToDelay)
		{
			printParametersRobot();
		
			flag = false;
		}
	}
	return NULL;
}


void *printParametersRobot(void) {

	double xPos = robot.getX();
	double yPos = robot.getY();
	double thAngular = robot.getTh();
	timeRegister = 0.5 + timeRegister;
	countRegisters = 1 + countRegisters;

	ArLog::log(ArLog::Terse, "simpleMotionCommands: Time=%.2f , Pose=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Rot. Vel=%.2f, Battery=%.2fV",
		timeRegister, xPos, yPos, thAngular, robot.getVel(), robot.getRotVel(), robot.getBatteryVoltage());
	
	writeDraw(countRegisters, timeRegister, xPos, yPos, thAngular);

	return NULL;
}


void *writeDraw(int id, float time, double posX, double posY, double angTh) {

	std::ofstream outfile;
	outfile.open("RobotDraw.txt", std::ofstream::out | std::ofstream::app);

	outfile
		<< id << " "
		<< time << " "
		<< posX << " "
		<< posY << " "
		<< angTh << " "
		<< std::endl;
	outfile.close();
	return NULL;
}

void *writeHead() {
	clearOutput();
	std::ofstream outfile;
	outfile.open("RobotDraw.txt", std::ofstream::out | std::ofstream::app);

	outfile
		<< "id "
		<< "time "
		<< "x "
		<< "y "
		<< "th"
		<< std::endl;
	outfile.close();
	return NULL;
}

void *clearOutput() {
	std::ofstream outfile("RobotDraw.txt");
	outfile<<"";
	outfile.close();
	return NULL;
}

