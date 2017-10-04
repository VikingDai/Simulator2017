/** 
	OSARR with Racetrack
	Automatic Control LiTH

	Filename: globaldefines.h

	Description:
		This file contains some global definitions that are used in multiple files in the
		project.

	Version:	Date:		Changes:							
		1.0		2011-10-19	First version

	Author/s:
	Isak Nielsen

*/

// RIGHT NOW WE ONLY USE THE SIZE OF THE CAR IN RRT-project.

#ifndef GLOBALDEFINES_H_
#define GLOBALDEFINES_H_

#include "RTError.h"
#include "matrix.h"

#include <string>
#include <vector>
#include <map>
#include <math.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <conio.h>

using namespace std;

/* Defines for wich number of the state that corresponds to position etc. */
#define x_index_ 1
#define y_index_ 2
#define v_index_ 3
#define th_index_ 4
#define thd_index_ 5

#define nr_of_states_ 5

#define ug_index_ 1
#define us_index_ 2

#define nr_of_control_ 2
#define filter_converge_value 100

static const float pi = 3.141592653589793238462643383279f;

static const std::string datadir = "";

/* Length of the car from centre to front, centre to rear and width */
static const float f_c = 0.05f;
static const float c_b = 0.05f;
static const float w=0.025f;

/* The line where the lap times are measured from. y < yLine must hold */
static const float xLine = 0.487f;
static const float yLine = 0.51f;

static const float xLine1 = 0.487f;
static const float yLine1 = 0.51f;
static const float xLine2 = -1.16f;
static const float yLine2 = 1.77f;
static const float xLine3 = 1.16f;
static const float yLine3 = 1.77f;

/* Standard values for dual rates and expo */
static const int steerexpstd=30;
static const int steerdrstd=55;       
static const int throttledrstd=40; 

/** Start position for simulated car */
static const float Xstart = 0.222f;	
static const float Ystart = 0.17f;

/*TrackHeight is the distance between the track and the calibration plane*/
static const float TrackHeight=-0.018f;







/** The distance between the two calibration markers */
#define worldscale 2.36

/** The width of the car, the distance in meters between the two back markers */
#define carWidth 0.033
//#define carWidth 0.09

//#define carWidth 0.03

/** The height of the car, the distance between the front 
	and between the two back markers */

#define carHeight 0.088
//#define carHeight 0.25

/** The distance between the front and the back ident marker, 
or the distance between the two backs and the front ident marker */

#define carIdent 0.062
//#define carIdent 0.17

/** Windows height along the x-axis */
#define window_width	800//1232.0

/** Windows height along the y-axis */
#define window_height	500//800.0

/** The border that should be included outside the field of view */
#define window_border	50//100.0

/** Defines for Modes: manual, auto, iocontroller */
#define c_manual 2
#define c_auto 1
#define c_iocontroller 3

/** Defines simulator */
#define simModel2012 1
#define simModel2011 2




/** Simple struct to hold a coordinate position, 
	contains just a simple x and y position */
struct coord
{
	/** Default constructor, initializes the x and y coordinates to 0 */
	coord() { xPos = 0; yPos = 0; }

	/** Constructor that sets the coordinates to the input values */
	coord(float ixPos, float iyPos) { xPos = ixPos; yPos = iyPos; }

	/** x position for the coordinate */
	float xPos;

	/** y position for the coordinate */
	float yPos;
};



#endif 