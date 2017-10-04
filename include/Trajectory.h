#pragma once

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>
#include <string>
#include <fstream>
#include <iostream>

/*! Enum for different rows of a trajectory */
enum access_traj { xrow, yrow, vrow, arow, thetarow, kapparow , trow};

class Trajectory
{
public:
	/** @brief Default constructor
 	*/
	Trajectory() = default;

	/** @brief Constructor
	*
 	*  @param itf Final time
 	*  @param idT Sample interval
 	*  @param cols Number of columns
 	*/
	Trajectory(double itf, double idT, int cols);

	/** @brief Constructor
	*
 	*  @param rows Number of rows.
 	*  @param cols Number of cols.
 	*/
	Trajectory(int rows, int cols);

	/** @brief Copy contructor
	*
 	*  @param other Trajectory to copy
 	*/
	Trajectory(const Trajectory& other);

	/** @brief Read file constructor
	*
	*  @param fName String with path to txt-file
	*/
	Trajectory(std::string fName);

	/** @brief Destructor
 	*/
	~Trajectory();

	/** @brief Set element in trajectory. Will throw ::std::out_of_range if element
	*		does not exists.
	*
 	*  @param row Row
 	*  @param col Column
 	*  @param value Value
 	*  @return Void
 	*/
	void set(int row, int col, double value);

	/** @brief Set a column of a trajectory to values specified in another trajectory
	*
	*  @param other
	*  @param column
	*  @return Void
	*/
	void Trajectory::setColumn(const Trajectory& other, int column);

	/** @brief Get value on position. Will thorw ::std::out_of_range if element does not
	*			exists.
	*
 	*  @param row Row
 	*  @param col column
 	*  @return Value
 	*/
	double get(int row, int col) const;

	/** @brief Get an entire column from the trajectory as a trajectory
	*
	*  @param other
	*  @param columns
	*  @return Trajectory with same number of rows as 'other'
	*/
	Trajectory getColumn(const int column) const;

	/** @brief Get number of columns.
	*
 	*  @param Void
 	*  @return Number of columns
 	*/
	int cols() const { return colMax; };

	/** @brief Get number of rows.
	*
 	*  @param Void
 	*  @return Number of rows.
 	*/
	int rows() const { return rowMax; };

	/** @brief Return true if the trajectory is valid, false otherwise.
	*
 	*  @param Void
 	*  @return bool True if trajectory is valid, false otherwise.
 	*/
	bool isValid() { return valid; };

	/** @brief Set valid to value.
	*
 	*  @param value True or false
 	*  @return Void
 	*/
	void setValid(bool value);

	/** @brief LP-filter a row in the trajectory. Results are stored in the trajectory
	*			on the corresponding row.
	*
 	*  @param Row to be filtered.
 	*  @return Void
 	*/
	void LPFilter(access_traj row);


	Trajectory& operator=(const Trajectory&);

	/** @brief Get sampletime of the trajectory.
	*
 	*  @param Void
 	*  @return Sampletime
 	*/
	double getdT() const { return dT; };

	void setdTtf(double dT,double tf);

	void addWerling(const Trajectory* other, int numWerling, int trajSize);

	void setdT(double idT) {dT = idT;};

	/** @brief Get final time of the trajectory.
	*
 	*  @param Void
 	*  @return Final time.
 	*/
	double getTf() const { return tf; };

	/** @brief Set for how long the trajectory is valid. Used in collision check.
	*
	*  \see Trajectory::getVaildUntil
 	*  @param value Number of samples before the trajectory is in collision.
 	*  @return Void.
 	*/
	void setValidUntil(int value);

	/** @brief Get the number of samples before the trajectory is in collision.
	*
	*  \see Trajectory::setVaildUntil()
 	*  @param Void
 	*  @return Vaild until
 	*/
	int getValidUntil() const;

private:
	double* traj;
	int colMax = 0;
	int rowMax = 0;
	bool valid;
	double dT;
	double tf;
	int validUntil = 0;
	
};


#endif
