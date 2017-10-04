#pragma once

#include <set>
#include <list>
#include <limits>

#include "Geometry.h"

using namespace std;

typedef enum{feasable, nonFeasable, partlyFeasable 
}pathFeasability;

typedef enum{positionConstrained, headingConstrained
}WayPointType;

typedef enum{forwardDirection, reverseDirection, stopDrive
}Direction;

typedef enum{Safety_Behaviour, Racing_Behaviour
}Behaviour;

class Sample
{
	public:
		Point2D samplePoint;
		Direction direction; 
		double velocity;
		bool active;

		Sample()
		{
			samplePoint = Point2D();
			direction	= Direction::forwardDirection;
			active = false;
		}
		Sample(Point2D samplePoint_in, Direction direction_in)
		{
			samplePoint = samplePoint_in;
			direction	= direction_in;
			active = true;
		}
		Sample(Point2D samplePoint_in, Direction direction_in, double v_in)
		{
			samplePoint = samplePoint_in;
			direction	= direction_in;
			velocity	= v_in;
			active = true;
		}
};

class State
{
	// States for the car /AW
	public:
		double x ,y;			// Position of model
		double v, a;			// Velocity and acceleration
		double heading;		// psi
		double kappa;
		double t;			// Time
		/*
		double beta2, beta3;
		double v;				// Velocity of model
		double a;				// Acceleration of model
		double heading;			// Heading of model
		double steerAngle;		// Heading of model
		double steerAngleRate;  // Heading of model
		*/

	State();
	State(const State& stateIn);
	State(double x_in, double y_in, double v_in, double a_in, double heading_in, double kappa_in, double t_in);
	void operator=(const State& other);
	double get_x() { return x; };
	double get_y() { return y; };
	double get_v() { return v; };
	double get_heading() { return heading; };
};


