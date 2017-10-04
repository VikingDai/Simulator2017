#include "Model.h"
#include "ObstacleMap.h"
#include "OptimalTrajectory.h"

class Simulator
{
public:
  
    double Ts;
    
    
    Simulator(double Ts_in);
    ~Simulator();
    
    void step();

    Model* getSimulationModel()
    {
        return simulationModel;
    }
    ObstacleMap* getObstacleMap()
    {
        return obstacleMap;
    }

	void set_obstacle_map(ObstacleMap *obstacleMapIn){obstacleMap=obstacleMapIn;}
    
	State stepTrajectory(OptimalTrajectory *optimalTrajectoryIn, int index);

private:
    Model       *simulationModel;
    ObstacleMap *obstacleMap;
    
};
