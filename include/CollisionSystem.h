#pragma once

/**
* Virtual class to be used as base for collision system
* @author OH, PN 2015
*/

#include <vector>
#include <string>
#include "Trajectory.h"
//#include "../include/stdafx.h" AW
#include <fstream>
#include <iostream>

struct trackRectangle {
	double x;
	double y;
	double height;
	double width;
};
struct trackCircle {
	double x;
	double y;
	double r;
	int toFill; // 0 for middle, 1 for upper left 2 for upper right 3 for lower right...
};
struct ColissionTrack {
	std::vector<trackRectangle*>  rectangleList;

	std::vector<trackCircle*>  circleList;
};

class CollisionSystem 
{
public:
	~CollisionSystem();

	void addTrackCircle(double x, double y, double r, int toFill);
	void addTrackRectangle(double x, double y, double width, double height);
	virtual int addObstalce(double x, double y, double width, double height, double v, double a, double psi, double psiDot)=0;
	virtual void setTarget(double width, double height, double refPointx, double refPointy) = 0;
	void readTrackFile(std::string fName);

protected:

	virtual void init()=0;
	virtual bool collissionTest(Trajectory* traj)=0;
	CollisionSystem(double ixMin, double ixMax, double iyMin, double iyMax, double iTolerance,
		double idT, double iTf, double iMaxMargin, double iTimeToFullMargin);
	double xMin;
	double xMax;
	double yMin;
	double yMax;
	double tolerance;
	double dT;
	double Tf;

	double maxMargin;
	double timeToFullMargin;

	ColissionTrack track;
};

