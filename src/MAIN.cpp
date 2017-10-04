/* 
TODO: WRITE DESCRIPTION
Authors: Niclas Evestedt, Oskar Ljungqvist and Kristin Bergstrand 2014.

*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <string>
#include <sstream>
#include <time.h>
#include "GlutVizualisation.h"

//planner
#include "planner.h"
#include "CollisionThread.h"
HANDLE collisionTestSem;
HANDLE collisionTestDone;
HANDLE collisionDataAccess;
HANDLE collisionTestStart;

using namespace std;

// Plannertest
typedef struct PlannerThreadData {

	//Parameters for the planner
	int numTimeSteps;
	int numTrajS;
	int numTrajD;
	double s2final;
	double kxi;
	double maxOffsetD;
	double maxOffsetS;

	bool* changed;

	// If the program is still running or not
	bool* shutdown;

	//std::vector<Car> *pcars;

	//Parameters *userParam;

	//sentFromMainThreadStruct *parameterChangeStruct;

	// Opt traj to be sent to regulator
	Trajectory* currentTraj;

	bool* newTrajReg;

	bool* newTrajDraw;

	bool* waitForCopy;

	int controlledCar;

	clock_t* time;

} PlannerData;


int main(int argc, char **argv)
{
	Simulator       *simulator;
	MapHandler      *mapHandler;
	ObstacleMap		*obstacleMap;
	glutVizualisation *vizualiser;


	double deltaT = 0.04;	// Time to expand_tree() between updates from server
	double Ts = 0.2;		// Time between states recieved from simulation

	//Used to store best traj. and send this to server after packing.
	//list<ReferenceTrajectoryState> reference_trajectory;

	/***********************************************************************************************/
	/*** Wait for valid position before initzialising the planner and obstacle map *****************/
	/**  RIGHT NOW WE HAVE TO SEND A VALID OBSTACLE ************************************************/
	/***********************************************************************************************/

	// Development file => we have to initiate states and obstacles.
	//x , y , vel, acc, heading, steerAngle, steerAngleRate.
	State egoState(1, -16, 1.12, 0, 0, 0, 0);

	//x,y,width,height,orientation,speed x, speed y.
	RectObstacle rectObstacle = RectObstacle(Point2D(60, -10), 24, 8, 0, 0, 0);

	obstacleMap = new ObstacleMap();

	obstacleMap->initialize();

	obstacleMap->insert_obstacle(rectObstacle);

	/****************************************************************/
	/*** Set up a simulator  ****************************************/
	/****************************************************************/

	simulator = new Simulator(deltaT);

	simulator->getSimulationModel()->set_state(egoState);

	simulator->set_obstacle_map(obstacleMap);

	/****************************************************************/
	/*** Setup for Map Handler **************************************/
	/****************************************************************/

	mapHandler = new MapHandler(obstacleMap);
	mapHandler->setEgoState(&simulator->getSimulationModel()->state);

	/****************************************************************/
	/*** Setup for vizualisation ************************************/
	/****************************************************************/

	vizualiser = new glutVizualisation();
	vizualiser->setSimulationModel(simulator->getSimulationModel());
	vizualiser->setEgoState(egoState);
	vizualiser->setObstacleMap(obstacleMap);
	vizualiser->setDrawOffset(egoState.x, egoState.y);

	vizualiser->initVizualisation();

	/****************************************************************/
	/*** Main execution loop     ************************************/
	/****************************************************************/

	int simulationStep = 0;
	OptimalTrajectory *OptTraj;
	OptTraj = new OptimalTrajectory("../data/referenceCar_1195.txt");
	vizualiser->setOptimalTrajectory(OptTraj);

	/***********************TEST ATT FÅ IN PLANNER *******/
	// ID of car to control
	int controlledCarID;

	//Create the vector of cars that will use the filters
	//vector<Car> cars;

	// Trajectory for the regulator to follow
	bool* newTrajReg = new bool(false);
	bool* newTrajDraw = new bool(false);

	double s2final = 0.7;
	double maxOffsetD = 0.45; // The track is 35 cm wide
	double maxOffsetS = s2final; // To make sure that we will always have one point to zero speed.
	int nrofTrajS = 11;
	int nrofTrajD = 21;
	double kxi = 1000;

	// InitializePlanner(PlannerData, &cars, sharedMainThreadPara, &userParam, plannerNumTimeSteps, &controlledCarID, nrofTrajS, nrofTrajD, kxi, maxOffsetD, maxOffsetS, s2final, currentRef, newTrajReg, newTrajDraw, &startTime);


	PlannerThreadData plannerData;

	// Global defines
	static int numTimeSteps = 200;

	// Read Trajectory from file
	Trajectory *ref;
	ref = new Trajectory("../data/referenceCar_1195.txt");

	// Read Werling grid from file
	WerlingPoints *werPoints;
	werPoints = new WerlingPoints("../data/werling_grid_and_xy_grid.txt");

	//plannerData.pcars = cars;
	//plannerData.userParam = userParam; används ej?
	//plannerData.parameterChangeStruct = parameterChangeStruct; används ej?
	plannerData.numTrajD = nrofTrajD;
	plannerData.numTrajS = nrofTrajS;
	plannerData.kxi = kxi;
	plannerData.numTimeSteps = numTimeSteps;
	plannerData.s2final = s2final;
	plannerData.maxOffsetD = maxOffsetD;
	plannerData.maxOffsetS = maxOffsetS;

	plannerData.currentTraj = ref; // behöver sättas, den traj som regulatorn kör efter.
	plannerData.newTrajReg = newTrajReg;
	plannerData.newTrajDraw = newTrajDraw;

	plannerData.controlledCar = controlledCarID;
	//plannerData.time = time;

	plannerData.shutdown = new bool(false);
	plannerData.changed = new bool(false);
	plannerData.waitForCopy = new bool(false);

	clock_t startTime, finishTime;

	Planner* currentPlanner = new Planner(plannerData.numTimeSteps, plannerData.s2final, plannerData.kxi, plannerData.maxOffsetD, plannerData.maxOffsetS, plannerData.numTrajS, plannerData.numTrajD);

	//int controlledCarID = *(plannerData->controlledCar); Redan def innan?

	// Defines validTraj, used to keep track of which type of trajectory is used, and when its valid.
	bool validTraj;

	// I have no clue how these works (wrote a bit about states below (line 260).
	// but otherwise these are very complicated, take care if use is needed.
	matrix states;
	matrix statesTruck(6, 1);
	matrix statesTralier;
	// AW set
	statesTruck.Set(1, 1, -0.6);
	statesTruck.Set(2, 1, 0.3);
	statesTruck.Set(4, 1, 0);
	std::vector<int> ident;
	//std::map<int, Obstacle_tracking_container> obstacle;

	double v[] = { 0,0 };
	double a[] = { 0,0 };
	double vtot = 0;
	double calcTime = 0;
	double atemp, vtemp, theta;
	int index, sleeptime;
	double planningtime = 0.2; // TODO: Put this to const?
	double optdT; // Save dT from optTraj so we know it locally.

	// Create a bunch of semaphores for future use,
	collisionTestSem = CreateSemaphore(NULL, 0, plannerData.numTrajS*plannerData.numTrajD, "collisionTestSem");
	collisionDataAccess = CreateSemaphore(NULL, 1, 1, "collisionDataAccess");
	collisionTestStart = CreateSemaphore(NULL, 0, 1, "collisionTestStart");
	collisionTestDone = CreateSemaphore(NULL, 0, 1, "collisionTestDone");

	CollisionData pCollisionData;

	InitializeCollision(pCollisionData, currentPlanner->getTrajSet(), &statesTruck, &statesTralier);

	// First iteration start planning from car position! FEL!
	double x1;
	double x2;

	while (true)
	{
		clock_t start = clock();

		//Wait for go (press S)
		if (vizualiser->getRunState())
		{

			x1 = -egoState.get_x() / 100;
			x2 = -egoState.get_y() / 100;

			vtemp = egoState.get_v();
			theta = M_PI - egoState.get_heading();
			v[0] = vtemp*cos(theta);
			v[1] = vtemp*sin(theta);

			ReleaseSemaphore(collisionTestStart, 1, NULL);

			validTraj = currentPlanner->useRefInstead(x1, x2, theta, v, a, ref);
			if (!validTraj) {
				ReleaseSemaphore(collisionTestStart, 1, NULL);
				validTraj = currentPlanner->calculateNewRefQuintic(x1, x2, theta, v, a, ref, werPoints);
			}

			/*
			if (validTraj) {

				//WaitForSingleObject(optTrajSem, INFINITE);
				//plannerData.time = clock();

				//Copy trajectory, coping since optTraj will be changed during next planning cycle and
				// the regulator is accesing the trajectory regularly.
				plannerData.currentTraj = currentPlanner->getOptTraj();
				plannerData.newTrajReg = new bool(true);
				plannerData.newTrajDraw = new bool(true);
				//ReleaseSemaphore(optTrajSem, 1, NULL);

				finishTime = clock();
				calcTime = double(abs(finishTime - startTime)); // Used to decide sleeptime untill next 5Hz planning cycle.

				sleeptime = (int)(1000 * planningtime - calcTime); // Sleeptime untill next cycle (sleepcall at the bottom of the function).

																   // Calculate where the car should be when the next planning cycle is finished (not when it starts, is finished, important! ).
				optdT = currentPlanner->getOptTraj()->getdT();
				index = (int)round(planningtime / optdT);
				//std::cout << "dT for the Optimal Traj is : " << optdT << std::endl;
				//std::cout << "index for next iteration is : " << index << std::endl;

				//Checks if the car is within a 1m of the x and y position of where the next trajectory is going to be made
				if ((abs(egoState.get_x()/100 - currentPlanner->getOptTraj()->get(xrow, index)) < 1 && abs(egoState.get_y()/100 - currentPlanner->getOptTraj()->get(yrow, index)) < 1)) {
					// Plots Trajectory from where the car should be after next planning cycle.
					x1 = currentPlanner->getOptTraj()->get(xrow, index);
					x2 = currentPlanner->getOptTraj()->get(yrow, index);
					vtemp = currentPlanner->getOptTraj()->get(vrow, index);
					atemp = currentPlanner->getOptTraj()->get(arow, index);
					theta = currentPlanner->getOptTraj()->get(thetarow, index);

					v[0] = vtemp*cos(theta);
					v[1] = vtemp*sin(theta);

					a[0] = atemp*cos(theta);
					a[1] = atemp*sin(theta);
				}
				else
				{
					// If trajectory has gotten away from car (bcs u ran into wall or something weird happend, computer lag etc.)
					// reset position back to car and calculate traj from here.
					vtemp = egoState.get_v()/100;
					theta = egoState.get_heading();

					v[0] = vtemp*cos(theta);
					v[1] = vtemp*sin(theta);
					x1 = egoState.get_x()/100 + v[0] * planningtime;// Add planningtime*v to position and plot from there for more accuracy.
					x2 = egoState.get_y()/100 + v[1] * planningtime; // Add planningtime*v to position and plot from there for more accuracy.
																 // Same story here as above, whole planningtime = 5Hz untill we give next traj to Regulator, so thats from where we should plot.
					a[0] = 0;
					a[1] = 0;
				}

				sleeptime = (int)(1000 * planningtime - calcTime);

			}

			//All where invalid, try again.
			else {

				x1 = egoState.get_x()/100;
				x2 = egoState.get_y()/100;

				vtemp = egoState.get_v()/100;
				v[0] = vtemp*cos(egoState.get_heading());
				v[1] = vtemp*sin(egoState.get_heading());

				a[0] = 0;
				a[1] = 0;

				theta = egoState.get_heading();


				//To make sure we try again immediately
				finishTime = clock();

				sleeptime = -1;

				//WaitForSingleObject(optTrajSem, INFINITE);
				//plannerData.time = clock();

				ReleaseSemaphore(collisionTestStart, 1, NULL);
				// If the old trajectory didnt work then lets try to back up the car a bit.
				for (int i = 0; i < plannerData.currentTraj->cols(); i++) {
					plannerData.currentTraj->set(xrow, i, x1);
					plannerData.currentTraj->set(yrow, i, x2);
					plannerData.currentTraj->set(vrow, i, -1);
					plannerData.currentTraj->set(arow, i, 0);
					plannerData.currentTraj->set(thetarow, i, theta);
					plannerData.currentTraj->set(kapparow, i, 0);
				}
				plannerData.newTrajReg = new bool(true);
				plannerData.newTrajDraw = new bool(true);
				std::cout << "No valid Traj, Stopping the car." << std::endl;

				//ReleaseSemaphore(optTrajSem, 1, NULL);
			}
			if (sleeptime > 0) {
				Sleep(sleeptime);
			}
			*/

			//optdT = currentPlanner->getOptTraj()->getdT();
			//index = (int)round(planningtime / optdT);
			cout << "x1 =" << -100 * currentPlanner->getOptTraj()->get(xrow, 1) << "\n";
			cout << "x2 = " << -100 * currentPlanner->getOptTraj()->get(yrow, 1) << "\n";
			//vtemp = currentPlanner->getOptTraj()->get(vrow, index);
			//atemp = currentPlanner->getOptTraj()->get(arow, index);
			//theta = currentPlanner->getOptTraj()->get(thetarow, index);

			for (int i = 0; i < currentPlanner->getOptTraj()->cols(); i++)
			{
				x1 = currentPlanner->getOptTraj()->get(xrow, i);
				x2 = currentPlanner->getOptTraj()->get(yrow, i);
				theta = currentPlanner->getOptTraj()->get(thetarow, i);
				OptTraj->set_state(x1, x2, theta, simulationStep + i);
			}

			////// Simulate
			egoState = simulator->stepTrajectory(OptTraj, simulationStep % 1330);
			simulationStep++;
		}
		//Update grafic window.
		vizualiser->reDisplay();

		while ((float)((clock() - start)) / CLOCKS_PER_SEC < 0.2);
		//cout << (float)((clock() - start)) / CLOCKS_PER_SEC << "\n";
	}
}

