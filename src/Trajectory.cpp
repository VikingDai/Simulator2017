#include "Trajectory.h"

Trajectory::Trajectory(double itf, double idT, int rows)
{
	dT = idT;
	tf = itf;
	colMax = (int)ceil(tf / dT);
	rowMax = rows;
	traj = new double[rowMax * colMax];
	valid = true;
	std::fill(traj, traj + rowMax * colMax, 0);

}

Trajectory::Trajectory(int rows, int cols)
{
	colMax = cols;
	rowMax = rows;
	valid = true;
	traj = new double[rowMax * colMax];
	std::fill(traj, traj + rowMax * colMax, 0);
	dT = 0.01;
	tf = 2;
}

Trajectory::Trajectory(const Trajectory& other)
{
	rowMax = other.rowMax;
	colMax = other.colMax;
	valid = other.valid;
	dT = other.dT; 
	tf = other.tf; 
	traj = new double[other.rows()*other.cols()];
	for (int row = 0; row < rowMax; ++row) {
		for (int col = 0; col < colMax; ++col) {
			traj[row*colMax + col] = other.get(row, col);
		}
	}
}

Trajectory::Trajectory(std::string fName)
{
	char line[256];
	double x, y, v, a, psi, kappa, t;

	/* Open the file to read from */
	std::fstream inputFile;
	inputFile.open(fName.c_str(), std::fstream::in);

	if (!inputFile.is_open())
	{
		throw std::runtime_error("Can not open file: " + fName);
	}

	//Read first line, discard info since first line is descriptors
	inputFile.getline(line, 256);
	inputFile.getline(line, 256);

	int i = 0;
	while (!inputFile.eof()) {
		inputFile.getline(line, 256);
		++i;
	}

	inputFile.close();
	inputFile.open(fName.c_str(), std::fstream::in);
	//Read first line, discard info since first line is descriptors
	inputFile.getline(line, 256);
	inputFile.getline(line, 256);

	rowMax = 7;
	colMax = i - 1;

	valid = true;
	traj = new double[rowMax * colMax];
	std::fill(traj, traj + rowMax * colMax, 0);
	dT = 0.01;		// TODO: read sampling time of states from file instead
	tf = 2;			// ???

	i = 0;
	/* Read lines until eof */
	while (!inputFile.eof())
	{
		inputFile.getline(line, 256);

		int nrOfWritten = sscanf(line, "%lf %lf %lf %lf %lf %lf %lf", &x, &y, &v, &a, &psi, &kappa, &t);
		if (nrOfWritten != 7) {
			std::cout << "Wrong number of elements in reference\n";
		}
		else {
			set(0, i, x);
			set(1, i, y);
			set(2, i, v);
			set(3, i, a);
			set(4, i, psi);
			set(5, i, kappa);
			set(6, i, t);
			++i;
		}
	}

	inputFile.close();
}


Trajectory& Trajectory::operator=(const Trajectory& other) {
	valid = other.valid;
	dT = other.dT;
	tf = other.tf;
	if (rowMax == 0 && colMax == 0) {
		traj = new double[other.rowMax * other.colMax];
		rowMax = other.rowMax;
		colMax = other.colMax;		
	}
	else if (rowMax != other.rows() || colMax != other.cols()) {
		throw std::range_error("Cannot copy trajectory, size mismatch");
	}

	for (int row = 0; row < rowMax; ++row) {
		for (int col = 0; col < colMax; ++col) {
			this->set(row, col, other.get(row, col));
		}
	}
	return *this;
}

void Trajectory::setValidUntil(int value)
{
	validUntil = value;
}

int Trajectory::getValidUntil() const
{
	return validUntil;
}

Trajectory::~Trajectory()
{
	delete traj;
}

void Trajectory::set(int row, int col, double value)
{
	if (row >= rowMax || col >= colMax) {
		throw std::range_error("Out of range in Trajectory::set");
	}
	traj[row*colMax + col] = value;
}

double Trajectory::get(int row, int col) const
{
	if (row >= rowMax || col >= colMax) {
		throw std::range_error("Out of range in Trajectory::get");
	}
	return traj[row*colMax + col];
}

void Trajectory::setColumn(const Trajectory& other, int column)
{
	if (column >= colMax || (other.rows() < this->rows()))
	{
		throw std::range_error("Out of range in Trajectory::setColumn");
	}
	for (int i = 0; i < this->rows(); ++i)
	{
		this->set(i, column, other.get(i, 0));
	}
}

Trajectory Trajectory::getColumn(const int column) const
{
	Trajectory new_traj{ this->rows(), 1 };
	for (int i = 0; i < this->rows(); ++i)
	{
		new_traj.set(i, 0, this->get(i, column));
	}
	return new_traj;
}

void Trajectory::setValid(bool value)
{
	//if (row >= rowMax || col >= colMax) {
	//	throw std::range_error("Out of range in Trajectory::set");
	//}
	valid = value;
	//traj[row*colMax + col] = value;
}

void Trajectory::setdTtf(double dTinput, double tfInput) {
	dT = dTinput;
	tf = tfInput;
}

void Trajectory::addWerling(const Trajectory* other, int numWerling, int trajSize) {
	for (int i = 0; i < rowMax; i++) {
		for (int j = 0; j < trajSize ; j++) {      // TODO: instead of colmax have colmax/(numWerlingSteps+1-numWerling)
													  //set(i,j, other.get(i,j-trajSize*(numWerling-1)));
			set(i, j + trajSize*(numWerling - 1), other->get(i, j));
		}
	}
}

void Trajectory::LPFilter(access_traj row) {
	double u = 1;
	double* yn = new double(colMax);
	yn[0] = this->get(row, 0);
	for (int i = 1; i < colMax; ++i) {
		yn[i] = this->get(row, i) + u*this->get(row, i - 1);
	}
	for (int i = 0; i < colMax; ++i) {
		this->set(row, i, yn[i]);
	}
	//delete yn;
}
