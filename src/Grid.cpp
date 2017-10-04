
#include "Grid.h"
#include <string.h>
#include <sstream>

/**
* Constructor 
*/
Grid::Grid(double ixMin, double ixMax, double iyMin, double iyMax,
				double iTolerance)
	: xMin(ixMin), xMax(ixMax), yMin(iyMin), yMax(iyMax), tolerance(iTolerance)
{
	colMax = (int)((xMax - xMin) / tolerance);
	rowMax = (int)((yMax - yMin) / tolerance);
	grid = new bool[rowMax*colMax];
	std::fill(grid, grid + rowMax*colMax, false);
	squareXSize = tolerance;
	squareYSize = tolerance;
}

/**
* Destructor
*/
Grid::~Grid()
{
	delete grid;
}

/**
* Test a position (x,y) for collsion
*/
bool Grid::test(double x, double y)
{
	unsigned int row = (int)((y - yMin) / squareYSize);
	unsigned int col = (int)((x - xMin) / squareXSize);

	if (row >= rowMax || row < 0) 
	{
		return true;
	}
	else if (col >= colMax ||  col <0 ) 
	{
		return true;
	}

	return this->getElement(row, col);
}


/**
* Resets the grid and marks the track
*/
void Grid::reset(const ColissionTrack& track, double targetRadius)
{
	std::fill(grid, grid + rowMax*colMax, false);
	this->markTrack(track, targetRadius);
}

/**
* Returns the element at (row,col)
*/
bool Grid::getElement(unsigned int row, unsigned int col) {

	if (row >= rowMax) 
	{
		throw std::range_error("GridCollision::getElement, row out of range. Requested: " + row);
	}
	else if (col >= colMax) 
	{
		throw std::range_error("GridCollision::getElement, col out of range. Requested: " + col);
	}

	return grid[colMax*row + col];
}

/**
* Returns the element at (xpos, ypos) 
*/
bool Grid::getElement(double xpos, double ypos) 
{
	unsigned int row = (int)((ypos - yMin) / squareYSize);
	unsigned int col = (int)((xpos - xMin) / squareXSize);

	if (row >= rowMax) 
	{
		throw std::range_error("GridCollision::getElement, row out of range. Requested: " + row);
	}
	else if (col >= colMax) 
	{
		throw std::range_error("GridCollision::getElement, col out of range. Requested: " + col);
	}

	return grid[colMax*row + col];
}

/**
* Sets the element at (xpos, ypos) as value
*/
void Grid::setElement(double xpos, double ypos, bool value) 
{
	unsigned int row = (int)((ypos - yMin) / squareYSize);
	unsigned int col = (int)((xpos - xMin) / squareXSize);

	if (row >= rowMax) 
	{
		throw std::range_error("GridCollision::setElement, row out of range. Requested: " + row);
	}
	else if (col >= colMax) 
	{
		throw std::range_error("GridCollision::setElement, col out of range. Requested: " + col);
	}

	grid[colMax*row + col] = value;
}
/**
* Sets the element at (row, col) as value
*/
void Grid::setElement(unsigned int row, unsigned int col,
	bool value)
{

	if (row >= rowMax) {
		throw std::range_error("GridCollision::setElement, row out of range. Requested: " + row);
	}
	else if (col >= colMax) {
		throw std::range_error("GridCollision::setElement, col out of range. Requested: " + col);
	}

	grid[colMax*row + col] = value;
}

/**
* Marks a circle
*/
void Grid::markCircle(double xpos, double ypos, double radius) 
{

	//XXX: Check if circle is inside grid
	double r2 = pow(radius, 2);
	double xposLocal = xpos - xMin;
	double yposLocal = ypos - yMin;

	unsigned int rowIt, colIt;
	int minRowCircle, maxRowCircle, minColCircle, maxColCircle;
	double dist;
	minRowCircle =  (int)floor((yposLocal - radius) / squareXSize);
	minColCircle = (int)floor((xposLocal - radius) / squareYSize);
	maxRowCircle = (int)ceil((yposLocal + radius) / squareXSize);
	maxColCircle = (int)ceil((xposLocal + radius) / squareYSize);


	if (minRowCircle < 1) 
	{
		minRowCircle = 1;
	}
	if (maxRowCircle > (int)rowMax - 1) 
	{
		maxRowCircle = rowMax - 1;
	}
	if (minColCircle < 1) 
	{
		minColCircle = 1;
	}
	if (maxColCircle > (int)colMax - 1) 
	{
		maxColCircle = colMax - 1;
	}

	for (rowIt = minRowCircle; (int)rowIt <= maxRowCircle; ++rowIt) 
	{
		for (colIt = minColCircle; (int)colIt <= maxColCircle; ++colIt) 
		{
			dist = pow(xposLocal - colIt*squareXSize, 2) +
				pow(yposLocal - rowIt*squareYSize, 2);
			if (dist < r2) 
			{
				this->setElement(rowIt, colIt, true);
				this->setElement(rowIt - 1, colIt, true);
				this->setElement(rowIt - 1, colIt - 1, true);
				this->setElement(rowIt, colIt - 1, true);
			}
		}
	}
}

/**
* Prints the grid to a file
*/
void Grid::printGrid(int id) 
{
	FILE* pFile;

	char fileName[80] = "grid";


	std::ostringstream idOss;
	idOss << id;
	strcat_s(fileName, idOss.str().c_str());
	strcat_s(fileName, ".txt");

	fopen_s(&pFile,fileName, "w");


	unsigned int i, j;
	for (i = 0; i < rowMax; ++i)
	{
		for (j = 0; j < colMax; ++j) 
		{
			fprintf(pFile, "%d ",
				this->getElement((unsigned int)i,
					(unsigned int)j));
		}
		fprintf(pFile, "\n");
	}
	fclose(pFile);
}

/**
* Marks a rectangle
*/
void Grid::markRectangle(const trackRectangle& rec, double targetRadius)
{
	double xPosMin = rec.x - xMin - targetRadius;
	double xPosMax = rec.x + rec.width - xMin + targetRadius ;
	double yPosMin = rec.y - yMin - targetRadius ;
	double yPosMax = rec.y + rec.height - yMin + targetRadius ;

	int minRowRec = (unsigned int)floor((yPosMin) / squareXSize);
	int minColRec = (unsigned int)floor((xPosMin) / squareYSize);
	int maxRowRec = (unsigned int)floor((yPosMax) / squareXSize);
	int maxColRec = (unsigned int)floor((xPosMax) / squareYSize);

	if (minRowRec < 0) 
	{
		minRowRec = 0;
	}
	if (maxRowRec > (int)rowMax -1) 
	{
		maxRowRec = rowMax-1;
	}
	if (minColRec < 0) 
	{
		minColRec = 0;
	}
	if (maxColRec > (int)colMax-1) 
	{
		maxColRec = colMax-1;
	}
	unsigned int i, j;
	for (i = minRowRec; (int)i <= maxRowRec; ++i) 
	{
		for (j = minColRec; (int)j <= maxColRec; ++j) 
		{
				this->setElement(i, j, true);
		}
	}
}
/**
* Marks a track circle
*/
void Grid::markTrakCircle(const trackCircle& circ, double targetRadius)
{
	if (circ.toFill == 0) 
	{
		this->markCircle(circ.x, circ.y, circ.r + targetRadius);
	}
	else 
	{
		targetRadius += tolerance;

		double r = circ.r - targetRadius;
		double r2 = pow(r, 2);
		double xposLocal = circ.x - xMin;
		double yposLocal = circ.y - yMin;

	   unsigned int rowIt, colIt;
		int minRowCircle, maxRowCircle, minColCircle, maxColCircle;
		double dist;

		if (circ.toFill < 3) 
		{ // upper two corners
			maxRowCircle = (unsigned int)ceil((yposLocal+r) / squareYSize);
			minRowCircle = (unsigned int)floor((yposLocal) / squareYSize);
			if (circ.toFill == 1) 
			{ // left
				minColCircle = (unsigned int)floor((xposLocal - r) / squareXSize);
				maxColCircle = (unsigned int)ceil((xposLocal) / squareXSize);
				
			}
			else 
			{ // right
				minColCircle = (unsigned int)floor((xposLocal) / squareXSize);
				maxColCircle = (unsigned int)ceil((xposLocal + r) / squareXSize);
			}
		}
		else 
		{ // lower two corners
			maxRowCircle = (unsigned int)ceil((yposLocal) / squareYSize);
			minRowCircle = (unsigned int)floor((yposLocal - r) / squareYSize);
			if (circ.toFill == 3) 
			{ // right
				minColCircle = (unsigned int)floor((xposLocal) / squareXSize);
				maxColCircle = (unsigned int)ceil((xposLocal + r) / squareXSize);
			}else 
			{ // left
				minColCircle = (unsigned int)floor((xposLocal - r) / squareXSize);
				maxColCircle = (unsigned int)ceil((xposLocal) / squareXSize);
			}
		}
		
		if (minRowCircle < 1) 
		{
			minRowCircle = 1;
		}
		if (maxRowCircle > (int)rowMax - 1) 
		{
			maxRowCircle = rowMax - 1;
		}
		if (minColCircle < 1) 
		{
			minColCircle = 1;
		}
		if (maxColCircle > (int)colMax - 1) 
		{
			maxColCircle = colMax - 1;
		}

		for (rowIt = minRowCircle; (int)rowIt <=maxRowCircle; ++rowIt) 
		{
			for (colIt = minColCircle; (int)colIt <= maxColCircle; ++colIt) 
			{
				dist = pow(xposLocal - rowIt *squareXSize, 2) +
					pow(yposLocal - colIt*squareYSize, 2);
				if (dist > r2) 
				{
					this->setElement(rowIt, colIt, true);
					this->setElement(rowIt - 1, colIt, true);
					this->setElement(rowIt - 1, colIt - 1, true);
					this->setElement(rowIt, colIt - 1, true);
				}
			}
		}
	}
}
/**
* Marks a complete track of rectangles and circles
*/
void Grid::markTrack(const ColissionTrack& track, double targetRadius)
{
	unsigned int i;
	for (i = 0;i < track.circleList.size();i++) 
		this->markTrakCircle(*track.circleList.at(i), targetRadius);

	for (i = 0;i < track.rectangleList.size();i++)
		this->markRectangle(*track.rectangleList.at(i), targetRadius); 

}