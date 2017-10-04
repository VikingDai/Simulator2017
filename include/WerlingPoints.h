#pragma once

#include "Trajectory.h"

#include <vector>
#include <iostream>
#include <array>
#include <string>
#include <fstream>

class WerlingPoints
{
public:
	/** @brief Default constructor
	*/
	WerlingPoints() = default;

	
	WerlingPoints(const WerlingPoints& other) = default;

	/** @brief Read file constructor
	*
	*  @param fName String with path to txt-file
	*/
	WerlingPoints(std::string fName);

	/** @brief Destructor
	*/
	~WerlingPoints() = default;

	/** @brief Set element in WerlingPoints. Will throw ::std::out_of_range if element
	*		does not exists.
	*
	*  @param row Row
	*  @param col Column
	*  @param value Value
	*  @return Void
	*/
	void insert(double s, double d, double x, double y);

	/** @brief Get value on position. Will thorw ::std::out_of_range if element does not
	*			exists.
	*
	*  @param row Row
	*  @param col column
	*  @return Value
	*/
	std::vector<double*> getPointsInWindow(double sBegin, double sEnd) const;

	void setTotalRefLength(const Trajectory* ref);

	double getTotalRefLength() const;

private:
	std::vector<double*> werlingPointsMat = std::vector<double*>();
	double sMax = 13.2599997911774;

};