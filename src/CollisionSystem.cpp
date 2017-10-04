
#include "CollisionSystem.h"

/**
* Constructor
*/
CollisionSystem::CollisionSystem(double ixMin, double ixMax, double iyMin,
	double iyMax, double iTolerance, double idT, double iTf, double iMaxMargin, double iTimeToFullMargin)
	: xMin(ixMin), xMax(ixMax), yMin(iyMin), yMax(iyMax), tolerance(iTolerance), dT(idT), Tf(iTf),
	maxMargin(iMaxMargin), timeToFullMargin(iTimeToFullMargin) 
{}
/**
* Destructor
*/
CollisionSystem::~CollisionSystem()
{
}

/**
* Adds a circle to the objects being tested for collsioin
*/
void CollisionSystem::addTrackCircle(double x, double y, double r, int toFill)
{
	trackCircle* circ = new trackCircle;
	circ->r = r;
	circ->x = x;
	circ->y = y;
	circ->toFill = toFill;
	track.circleList.push_back(circ);
}

/**
* Adds a rectangle to the objects being tested for collsioin
*/
void CollisionSystem::addTrackRectangle(double x, double y, double width, double height)
{
	trackRectangle* rec = new trackRectangle;
	rec->x = x;
	rec->y = y;
	rec->height = height;
	rec->width = width;
	track.rectangleList.push_back(rec);
}

/**
* Reads a track from file to be tested for collision
*/
void CollisionSystem::readTrackFile(std::string fName)
{
	char line[256];

	/* Open the file to read from */
	std::fstream inputFile;
	inputFile.open(fName.c_str(), std::fstream::in);

	if (!inputFile.is_open())
	{
		std::cout << "Couldn't open trackFile collisionsystem " << fName << "\n";
		return;
	}

	//Read first line, discard info since first line is descriptors
	inputFile.getline(line, 256);

	double x, y, w, h;
	int type;

	/* Read lines until eof */
	while (!inputFile.eof())
	{
		inputFile.getline(line, 256);

		int nrOfWritten = sscanf_s(line, "%lf %lf %lf %lf %d", &x, &y, &w, &h, &type);
		
		switch (type)
		{
		case 5:
			addTrackRectangle(x, y, w, h);
		default:
			break;
		}

	}


	inputFile.close();
}