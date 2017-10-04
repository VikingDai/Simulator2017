#include <cstddef> //Defines NULL
#include "PlannerTypes.h"

using namespace std;

State::State()
{
  x				= 0;
  y				= 0;
  v				= 0;
  a				= 0;
  heading		= 0;
  kappa			= 0;
  t				= 0;
}

State::State(double x_in, double y_in, double v_in, double a_in, double heading_in, double kappa_in, double t_in)
{
  x				= x_in;
  y				= y_in;
  v				= v_in;
  a				= a_in;
  heading		= heading_in;
  kappa			= kappa_in;
  t				= t_in;
}

State::State(const State& stateIn)
{
  x				= stateIn.x;
  y				= stateIn.y;
  v				= stateIn.v;
  a				= stateIn.a;
  heading		= stateIn.heading;
  kappa			= stateIn.kappa;
  t				= stateIn.t;

}

void State::operator=(const State&  other)
{
  x				= other.x;
  y				= other.y;
  v				= other.v;
  a				= other.a;
  heading		= other.heading;
  kappa			= other.kappa;
  t				= other.t;
}

