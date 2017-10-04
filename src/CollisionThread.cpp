#include "CollisionThread.h"
//#include "../include/globaldefines.h" AW

extern HANDLE collisionTestSem;
extern HANDLE collisionDataAccess;
extern HANDLE collisionTestStart;
extern HANDLE collisionTestDone;


#define MINX -2.0
#define MAXX 2.0 
#define MINY 0.0
#define MAXY 2.5
#define TOLERANCE 0.01

#define MAXMARGIN 0.025
#define TIME_TO_MAX_MARGIN .15

// AW, globaldefines
static double plannerTf = 2.0;
static double plannerDT = 0.01;
// Parameters used when creaeting obstacles
#define carH 0.02
#define carW 0.105 
#define truckH 0.08
#define truckW 0.24
#define semitrailerH 0.08
#define semitrailerW 0.24
#define obstacle_1H 0.1
#define obstacle_1W 0.1
#define obstacle_2H 0.05
#define obstacle_2W 0.05

DWORD WINAPI CollisionThread(LPVOID pCollisionParam) {

	CollisionData* pCollisionData;
	pCollisionData = (CollisionData *)pCollisionParam;

	GridCollision test(MINX, MAXX, MINY, MAXY, TOLERANCE, plannerDT, plannerTf, MAXMARGIN, TIME_TO_MAX_MARGIN);
	test.readTrackFile("../data/StaticObstacles.txt");

	test.setTarget(.20 , .07, 0, 0);

	clock_t startTime, finishTime;

	double v,a = 0;

	matrix truckStates;
	matrix trailerStates;

	test.reset();

	//test.addObstalce(0, 0, 0.3, 0.3, 0, 0, 0, 0);

	std::vector<Trajectory*>* trajSet = (pCollisionData->trajSet);
	bool valid;

	//std::map<int, Obstacle_tracking_container>::iterator it;

	while (true) {

		test.removeAllObstacles();

		test.reset();

		WaitForSingleObject(collisionTestStart, INFINITE);
		WaitForSingleObject(collisionDataAccess, INFINITE);
		
		
		/* Copy obstacles from planner thread*/
		truckStates = *pCollisionData->truck;
		trailerStates = *pCollisionData->trailer;

		
		/*
		for (it = pCollisionData->obstacles->begin(); it != pCollisionData->obstacles->end(); ++it) {
			
			
			
			v = sqrt(pow(it->second.vx,2) + pow(it->second.vy,2));
			test.addObstalce(it->second.x + it->second.vx*0.2,
				it->second.y + it->second.vy*0.2,
				it->second.width,
				it->second.height,
				v,
				a,
				it->second.angle,
				0);
				
		}
		*/
		ReleaseSemaphore(collisionDataAccess, 1, NULL);

		if (truckStates.Get(1, 1) != -1 && truckStates.Get(2, 1) != -1) {
			test.addObstalce(truckStates.Get(x_index_, 1),
				truckStates.Get(y_index_, 1),
				truckW,
				truckH,
				truckStates.Get(3, 1),
				0, // TODO: byt acceleration här
				truckStates.Get(4, 1), // TODO: PSI HÄR???
				0);
		}

		if (trailerStates.Get(1, 1) != -1 && trailerStates.Get(2, 1) != -1) {
			test.addObstalce(trailerStates.Get(x_index_, 1),
				trailerStates.Get(y_index_, 1),
				semitrailerW,
				semitrailerH,
				trailerStates.Get(3, 1),
				0, // TODO: byt acceleration här
				trailerStates.Get(4, 1), // TODO: PSI HÄR???
				0);
		}


		startTime = clock();

		test.markObstacles();

	

		/* Test for colission*/
	//	for (int j = 0; j <= trajSet->size(); j++)
	//	{
			for (unsigned int i = 0; i < trajSet->size(); ++i) {
				WaitForSingleObject(collisionTestSem, INFINITE);

				if (trajSet->at(i)->isValid())
				{
					valid = test.collissionTest(trajSet->at(i));
					trajSet->at(i)->setValid(!valid);
				}

			}
			ReleaseSemaphore(collisionTestDone, 1, NULL);
		//}
		finishTime = clock();
		//std::cout << "Time Collision: " << double(finishTime - startTime) / CLOCKS_PER_SEC << std::endl;

	}

	

	return 0;

}

void InitializeCollision(CollisionData& pCollisionData, std::vector<Trajectory*>* trajSet, matrix* truckState, matrix* tralierState) {
	
	HANDLE threadHandle;
	int threadid;

	pCollisionData.trajSet = trajSet;
	pCollisionData.truck = truckState;
	pCollisionData.trailer = tralierState;

	threadHandle = CreateThread(NULL,
		0, CollisionThread,
		&pCollisionData, 0, (LPDWORD)&threadid);

	/** Set full priority */
	SetThreadPriority(threadHandle, 1);
}