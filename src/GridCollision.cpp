
#include "GridCollision.h"
#include <iostream>

/**
* Constructor
*/
GridCollision::GridCollision(double ixMin, double ixMax, double iyMin, 
			     double iyMax, double iTolerance, double idT, 
			     double iTf, double iMaxMargin, double iTimeToFullMargin)
	: CollisionSystem(ixMin, ixMax, iyMin, iyMax, iTolerance, idT, iTf, iMaxMargin, iTimeToFullMargin)
{

	// create one grid for each timestep
	nGrids = (int)(iTf / idT)+1;
	gridList = new Grid*[nGrids];
	int i;
	for (i = 0; i < nGrids; i++) {
		gridList[i] = new Grid(ixMin, ixMax, iyMin, iyMax, iTolerance);
	}

	nextID = 0;
	targetRadius = .05;
}

/**
* Destructor
*/
GridCollision::~GridCollision()
{
	obstacles.clear();
	int i;
	for (i = 0; i < nGrids; i++)
	{
		delete gridList[i];
	}
	delete gridList;

	delete target.x;
	delete target.y;
}
void GridCollision::init() {}

/**
* Test a trajetory for collision
*/
  
bool GridCollision::collissionTest(Trajectory* traj) 
{ 
	for (int curGrid = 0; curGrid < traj->cols(); curGrid++)
	{
		// this should not happen
		/* TODO: Throw and catch to be able to handle this*/
		if (curGrid >= nGrids) 
		{
			throw std::range_error("curGrid >= nGrids");
		}
		//TODO: no theta
		if (testTargetInState(traj->get(xrow,curGrid), traj->get(yrow, curGrid), traj->get(thetarow, curGrid), curGrid)) 
		{
			traj->setValidUntil(curGrid);
			return true;
		}
	}

	return false;
}
/**
* Test if the target in the stat given by (x,y,psi) is in collision in grid gridNR
*/
bool GridCollision::testTargetInState(double x, double y, double psi,int gridNr)
{
	// precalculate sin and cos
	double cosPsi = cos(psi);
	double sinPsi = sin(psi);

	for (int i = 0; i < target.nPoints; i++) 
	{
		double x1 = target.x[i];
		double y1 = target.y[i];
			if (gridList[gridNr]->test(x+target.x[i]*cosPsi - target.y[i] * sinPsi,
									y+ target.y[i] * cosPsi + target.x[i] * sinPsi)) 
			{
				return true;
			}
	}
	return false;
}
/**
* Simulates the obstacles forward in time
*/
void eulerForward(GridObstacle* obs, double dT)
{
	obs->xpos += obs->v*cos(obs->psi)*dT;
	obs->ypos += obs->v*sin(obs->psi)*dT;
	obs->v += obs->a*dT;
	obs->psi += obs->psidot*dT;
}
/**
* Marks one obstacle in all grids
*/
void GridCollision::markObstacle(GridObstacle * obstacle)
{
	for (int curentGrid = 0; curentGrid < nGrids; curentGrid++) {

		double margin = maxMargin * dT * (double)curentGrid / timeToFullMargin;
		if (margin > maxMargin)
			margin = maxMargin;

		//position of cicle with centrum of target as reference
		double xInRec = -obstacle->distWidth*((double)(obstacle->nCircWidth / 2));
		double yInRec = -obstacle->distHeight*((double)(obstacle->nCircHeight / 2));
		double yInRecStart = yInRec;

		// precalculate sin and cos
		double cosPsi = cos(obstacle->psi);
		double sinPsi = sin(obstacle->psi);

		int i, j;

		for (i = 0; i < obstacle->nCircWidth; i++) 
		{
			for (j = 0; j < obstacle->nCircHeight; j++) 
			{
				gridList[curentGrid]->markCircle(obstacle->xpos + cosPsi*xInRec - sinPsi*yInRec,
					obstacle->ypos + cosPsi*yInRec + sinPsi*xInRec,
					obstacle->radius + targetRadius + margin);
				yInRec += obstacle->distHeight;
			}
			xInRec += obstacle->distWidth;
			yInRec = yInRecStart;
		}

		eulerForward(obstacle, dT);
	}
}
/**
* Marks all obstacles in all grids
*/ 
void GridCollision::markObstacles()
{
	int i;
	for (i = 0; i < (int)obstacles.size(); ++i) {
		markObstacle(obstacles.at(i));
	}
}
/**
* Approximate the obstacle with circles
*/
void approximateWithCircles(GridObstacle& obstacle,double tol)
{
	int nWidth = 1;
	int nHeight = 1;

	while (1) 
	{

		double r = sqrt(pow(obstacle.width / ((double)(nWidth * 2)), 2) +
			pow(obstacle.height / ((double)(nHeight * 2)), 2));

		double eWidth = abs(r - obstacle.width / ((double)(nHeight * 2)));
		double eHeight = abs(r - obstacle.height / ((double)(nWidth * 2)));

		if (eHeight > eWidth && eHeight > tol) 
		{
			nHeight++;
		}
		else if (eWidth > tol) 
		{
			nWidth++;
		}
		else 
		{
			break;
		}
	}

	double r = sqrt(pow(obstacle.width / ((double)(nWidth * 2)), 2) +
		pow(obstacle.height / ((double)(nHeight * 2)), 2));

	obstacle.radius = r;
	obstacle.nCircHeight = nHeight;
	obstacle.nCircWidth = nWidth;
	obstacle.distWidth = obstacle.width / ((double)(nWidth));
	obstacle.distHeight = obstacle.height / ((double)(nHeight));
}
/**
* Adds one obstacle in the obstacle vector
*/
int GridCollision::addObstalce(double x, double y, double width, 
			       double height, double v, double a,
			       double psi, double psiDot)
{
	GridObstacle* newObstacle = new GridObstacle;
	//XXX: Fix so that x and y is translated correctly from global to local coordinates. 
	
	newObstacle->xpos = x; // marcCircle translates to local
	newObstacle->ypos = y;
	newObstacle->width = width;
	newObstacle->height = height;
	newObstacle->v = v;
	newObstacle->a = a;
	newObstacle->psi = psi;
	newObstacle->psidot = psiDot;
	newObstacle->id = nextID;
	++nextID;
	approximateWithCircles(*newObstacle,tolerance);
	obstacles.push_back(newObstacle);
	return newObstacle->id;
}
/**
* Sets target that is to be tested for collision
*/
void GridCollision::setTarget(double width, double height, double refPointx, double refPointy)
{
	GridObstacle temp;
	temp.width = width;
	temp.height = height;
	approximateWithCircles(temp, tolerance);

	target.radius = temp.radius;

	targetRadius = temp.radius;

	target.nPoints = temp.nCircHeight*temp.nCircWidth;
	target.x = new double[target.nPoints];
	target.y = new double[target.nPoints];


	double xInRec = -temp.distWidth*((double)(temp.nCircWidth / 2));
	double yInRec = -temp.distHeight*((double)(temp.nCircHeight / 2));
	double yInRecStart = yInRec;

	int i, j;
	for (i = 0; i < temp.nCircWidth; i++) 
	{
		for (j = 0; j < temp.nCircHeight; j++) 
		{
			target.x[i*temp.nCircHeight+j] = xInRec + refPointx;
			target.y[i*temp.nCircHeight+j] = yInRec + refPointx;
			yInRec += temp.distHeight;
		}

		xInRec += temp.distWidth;
		yInRec = yInRecStart;
	}

	return;
}
/**
* Resets all grids for next timestep
*/ 
void GridCollision::reset()
{
	for (int i = 0; i < nGrids;i++) 
	{
		double margin = maxMargin * dT * (double)i / timeToFullMargin;
		if (margin > maxMargin)
			margin = maxMargin;
		gridList[i]->reset(track, targetRadius + margin);
	}
}


/**
* Moves the object with id
*/
void GridCollision::changeObstacle(int id, double x, double y, double width, 
				   double height, double v, double a, 
				   double psi, double psiDot)
{
	GridObstacle* currentObstacle = NULL;
	int i;
	for (i = 0; i < (int)obstacles.size(); ++i) 
	{
		if (obstacles.at(i)->id == id) 
		{
			currentObstacle = obstacles.at(i);
			break;
		}
	}
	if (currentObstacle == NULL) 
	{
		throw std::range_error("Obstacle not found in GridCollision::changeObstacle, requested ID: " + id);
	}
	currentObstacle->xpos = x;
	currentObstacle->ypos = y;
	currentObstacle->width = width;
	currentObstacle->height = height;
	currentObstacle->v = v;
	currentObstacle->a = a;
	currentObstacle->psi = psi;
	currentObstacle->psidot = psiDot;
}
/**
* Removes the object with id
*/
void GridCollision::removeObstacle(int id)
{
	std::vector<GridObstacle*>::iterator it;
	for (it = obstacles.begin(); it != obstacles.end(); ++it) {
		if ((*it)->id == id) {
			obstacles.erase(it);
			return;
		}
	}
	//If this point is reached, we did not find a element to erase
	throw std::range_error("Obstacle not found in GridCollision::removeObstacle, requested ID: " + id);
}
/**
* Removes all obstacles
*/
void GridCollision::removeAllObstacles() {
	obstacles.clear();
}

/**
* Prints the grids
*/
void GridCollision::printGrid()
{
	int i;
	for (i = 0; i < nGrids; i++)
	{
		gridList[i]->printGrid(i);
	}

}