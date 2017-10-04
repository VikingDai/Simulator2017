/* 
	Racetrack
	Automatic Control LiTH

	Filename: RTError.cpp

	Description:
		This file contains the implementation for the RTError class

	Version:	Date:		Changes:							
		1.0		2011-06-xx	First version
		1.1		2011-07-04	Minor changes, added error boolean, removed the FC dependency

	Author/s:
	Isak Nielsen
	Johan Kihlberg

*/

#include "RTError.h"

using namespace std;

/* Function declaration */

/* Constructor with information about description and fName */
RTError::RTError(const string dscrp, const string fName)
{
	description = dscrp;
	functionName = fName;
	error = true;
}

/* Prints error information */
void RTError::PrintError()
{
	cout << "Error description: " << description << endl;
	cout << "In function: " << functionName << endl << endl;
}

/* Sets values to the private members */
void RTError::SetDescription(const string dscrp,const string fName)
{
	description = dscrp;
	functionName = fName;
	error = true;
}

string RTError::GetDescription() const
{
	return description;
}

string RTError::GetFunction() const
{
	return functionName;
}
