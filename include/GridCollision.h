#pragma once
#include "CollisionSystem.h"
#include "Grid.h"
#include <vector>
/**
* Class that implements a collisonsystem based on a grid
* @author OH, PN 2015
*/

struct GridObstacle {
	double width;
	double height;

	double radius;
	int nCircWidth;
	int nCircHeight;
	double distWidth;
	double distHeight;
	

	double xpos;
	double ypos;
	double v;
	double a;
	double psi;
	double psidot;
	int id;
};

struct GridTarget {
	int nPoints;
	double* x;
	double* y;
	double radius;
};


class GridCollision : public CollisionSystem
{
public:
	GridCollision(double ixMin, double ixMax, double iyMin, double iyMax, 
		      double iTolerance, double idT, double iTf, double iMaxMargin, double iTimeToFullMargin);
	 void init();

	 bool collissionTest(Trajectory*);
	 
	 void markObstacles();
	 int addObstalce(double x, double y, double width, double height,
			 double v, double a, double psi, double psiDot);
	 void changeObstacle(int id, double x, double y, double width, 
			     double height, double v, double a, double psi,
			     double psiDot);
	 void setTarget(double width, double height, double refPointx, double refPointy);

	 void reset();

	 void removeObstacle(int id);
	 void removeAllObstacles();
	 void printGrid();

	~GridCollision();

private:
	
	void markObstacle(GridObstacle* obstacle);
	bool testTargetInState(double x, double y, double psi, int gridNr);

	Grid** gridList;
	int nGrids;

	int nextID;
	std::vector<GridObstacle*> obstacles;

	GridTarget target;
	double targetRadius;

};

