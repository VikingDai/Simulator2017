/** 
	Racetrack
	Automatic Control LiTH

	Filename: RTError.h

	Description:
		This file contains the definitions for the RTError class

	Version:	Date:		Changes:							
		1.0		2011-06-xx	First version
		1.1		2011-07-04	Minor changes, added error boolean, removed the FC dependency

	Author/s:
	Isak Nielsen
	Johan Kihlberg

*/


#ifndef RTERROR_H_
#define RTERROR_H_

#include <string>
#include <iostream>

class RTError
{
public:
	/** Default constructor, no error */
	RTError(){error = false;};

	/** Constructor with a given error description and function name */
	RTError(const std::string dscrp, const std::string fName);
	
	/* Operators */
	//bool operator==(const RTError rte);
	//bool operator!=(const RTError rte);

	/** Print the error to the console */
	void PrintError();

	/** Set the description, dscrp is what error appeared and fName is the function
		where it happened */
	void SetDescription(const std::string dscrp, const std::string fName);

	/** Return the Description of the error */
	std::string GetDescription() const;

	/** Return the function where the error appeared*/
	std::string GetFunction() const;

	/** Return if there has been an error or everything went ok */
	bool Error() { return error; }
	

private:
	/** String to the description of the error */
	std::string description;
	
	/** String to the function name where the error appeared */
	std::string functionName;

	/** Boolean if there has been an error or not */
	bool error;
};

#endif