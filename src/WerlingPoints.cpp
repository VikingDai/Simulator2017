#include "WerlingPoints.h"

// Constructor for reading from file
WerlingPoints::WerlingPoints(std::string fName)
{
	char line[256];
	double s, d, x, y;
	int nrOfWritten;

	// Open the file to read from 
	std::fstream inputFile;
	inputFile.open(fName.c_str(), std::fstream::in);

	if (!inputFile.is_open())
	{
		throw std::runtime_error("Can not open file: " + fName);
	}

	int i = 0;
	while (!inputFile.eof()) {
		inputFile.getline(line, 256);
		++i;
	}

	inputFile.close();
	inputFile.open(fName.c_str(), std::fstream::in);

	i = 0;
	// Read lines until eof 
	while (!inputFile.eof())
	{
		inputFile.getline(line, 256);
		nrOfWritten = sscanf(line, "%lf", &s);
		inputFile.getline(line, 256);
		nrOfWritten = sscanf(line, "%lf", &d);
		inputFile.getline(line, 256);
		nrOfWritten = sscanf(line, "%lf", &x);
		inputFile.getline(line, 256);
		nrOfWritten = sscanf(line, "%lf", &y);
		if (nrOfWritten != 1) {
			std::cout << "Wrong number of elements in Werling Points File\n";
		}
		else {
			insert(s, d, x, y);
			++i;
		}
	}

	inputFile.close();
}


void WerlingPoints::insert(const double s, const double d, const double x, const double y)
{
	double* toInsert = new double[4];
	toInsert[0] = s;
	toInsert[1] = d;
	toInsert[2] = x;
	toInsert[3] = y;
	werlingPointsMat.push_back(toInsert);
}

void WerlingPoints::setTotalRefLength(const Trajectory* ref)
{
	double sum = 0;
	double lengthx, lengthy;
	int refIndex = ref->cols();

	for (int i = 0; i < refIndex-1; i++) {
		lengthx = ref->get(0,i+1) - ref->get(0,i);
		lengthy = ref->get(1,i+1) - ref->get(1,i);

		sum += sqrt(pow(lengthx,2) + pow(lengthy,2));
	}

	sMax = sum;
}

double WerlingPoints::getTotalRefLength() const 
{
	return sMax;
}

std::vector<double*> WerlingPoints::getPointsInWindow(double sBegin, double sEnd) const
{
	std::vector<double*> tmpVec;
	std::vector<double*> tmpMat = werlingPointsMat;
	std::vector<double*>::iterator it;
	int i = 0;
	
	for (it = tmpMat.begin(); it != tmpMat.end(); ++it)
	{
		if (sBegin > sMax && (*it)[0] >= sBegin-sMax && (*it)[0] <= sEnd-sMax) {
			tmpVec.push_back(*it);
			i++;
		}

		else if (sEnd > sMax && ((*it)[0] >= sBegin || (*it)[0] <= sEnd-sMax)) {
			tmpVec.push_back(*it);
			i++;
		}
		else if ((*it)[0] >= sBegin && (*it)[0] <= sEnd) {
			tmpVec.push_back(*it);
			i++;
		}

		//else if ((*it)[0] >= (int)std::floor(sBegin) % (int)std::floor(sMax) && (*it)[0] <= (int)std::floor(sEnd) % (int)std::floor(sMax)) {
		//	tmpVec.push_back(*it);
		//	i++;
		//}
		
			
	}
	

	return tmpVec;
}

