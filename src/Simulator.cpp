#include "Simulator.h"

Simulator::Simulator(double Ts_in)
{
    Ts = Ts_in;

    simulationModel = new Model(1);

}

/*
void Simulator::step()
{	

    ControlCommand c;
    double vRef;

    simulationModel->controller->calcControl(c, vRef);

    simulationModel->state.v = vRef;

    if(simulationModel->controller->referenceControl.empty())
        simulationModel->state.v =  0;

    simulationModel->step(Ts, c.steerAngleCommand, 0);

}
*/

State Simulator::stepTrajectory(OptimalTrajectory *optimalTrajectoryIn, int index)
{
	return optimalTrajectoryIn->get_optimal_trajectory_state(index);
}
