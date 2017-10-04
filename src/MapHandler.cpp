#include "MapHandler.h"

MapHandler::MapHandler(ObstacleMap *oMapIn)
{
	mapType = MapType::LOCAL_OMAP;
	oMap = oMapIn;
}

/*MapHandler::MapHandler(mock_lcm::grid_map  *lcmMapIn)
{
mapType = MapType::LCM_COSTMAP;
lcmMap = lcmMapIn;

Eigen::Matrix<double,Eigen::Dynamic,2> circleCenters_(1,2);
circleCenters_(0,0) = 0.0;
circleCenters_(0,1) = 0.0;
modelTemplate       = CollisionCheckTemplate(circleCenters_);
costChecker         = CostChecker(modelTemplate);
costChecker.setCostMap(lcmMap);

}*/


/*double MapHandler::getCostLCMCost(State &state)
{

//Convert to local map coordinates
Point2D mapStateP;
Point2D checkStateP(state.x, state.y);
Point2D egoStateP(egoState->x, egoState->y);

mapStateP = checkStateP - egoStateP;
//mapStateP = mapStateP.rotate(-egoState->heading);

if(costChecker.checkCost(mapStateP.x, mapStateP.y, 0) != 0)
std::cout << costChecker.checkCost(mapStateP.x, mapStateP.y, 0) << " x: " << mapStateP.x << " y: " << mapStateP.y << std::endl;

return costChecker.checkCost(mapStateP.y, mapStateP.x, state.heading);
}*/

