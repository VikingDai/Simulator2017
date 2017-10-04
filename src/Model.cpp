#include "Model.h"
using namespace std;

/****************************************************************/
/*** Model class: Implementation of the system model that will  */
/*** be controled                      **************************/
/****************************************************************/

/****************************************************************/
/*** Constructor Destructor  ************************************/
/****************************************************************/
Model::Model(int ID)
{
	debugLogg = false;
	state = State();
	modelID = ID;
}

/****************************************************************/
/*** Set methods ************************************************/
/****************************************************************/
void	Model::set_state(double x_in, double y_in, double v_in, double a_in, double heading_in, double steerAngle_in, double steerAngleRate_in)
{
	state = State(x_in, y_in, v_in, a_in, heading_in, steerAngle_in, steerAngleRate_in);
}

void	Model::set_state(State& stateIn)
{
	state = stateIn;
}

