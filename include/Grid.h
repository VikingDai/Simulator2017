#pragma once
/**
* Class that implements a grid that can be used in a collision system
* @author OH, PN 2015
*/

#include <algorithm>
#include <stdexcept>
#include "CollisionSystem.h"

/*
           ________________________
        ^ |                        |
 y      | |                        |
 row    | |                        |
 height | |                        |
        | |                        |
        - |________________________|
		      |-------------->
		       x, col, width
*/

class Grid
{
public:
	Grid(double ixMin, double ixMax, double iyMin, double iyMax,
		double iTolerance);
	~Grid();

	void printGrid(int id);

	void markCircle(double xpos, double ypos, double radius);
	void reset(const ColissionTrack& track, double targetRadius);

	bool test(double x, double y);
private:
	void setElement(unsigned int row, unsigned int col, bool value);
	bool getElement(unsigned int row, unsigned int col);

	void setElement(double xpos, double ypos, bool value);
	bool getElement(double xpos, double ypos);

	void Grid::markRectangle(const trackRectangle& rec, double targetRadius);
	void Grid::markTrakCircle(const trackCircle& circ, double targetRadius);
	void Grid::markTrack(const ColissionTrack& track, double targetRadius);

	bool* grid;
	unsigned int colMax;
	unsigned int rowMax;
	double squareXSize;
	double squareYSize;

	double xMin;
	double xMax;
	double yMin;
	double yMax;
	double tolerance;
};

