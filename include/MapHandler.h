#pragma once

#include "ObstacleMap.h"
#include "PlannerTypes.h"
//#include "collision_checking/CostMap.hpp"

//#include "msg/mock_lcm/drive_map.hpp"

enum MapType{LOCAL_OMAP, LCM_COSTMAP};

class MapHandler
{
public:


    MapHandler(ObstacleMap *oMapIn);
    //MapHandler(mock_lcm::grid_map *lcmMapIn);
    ~MapHandler();

   /* mock_lcm::grid_map* getLCMMap()
    {
        return lcmMap;
    }*/

    void setEgoState(State *egoStateIn)
    {
        egoState = egoStateIn;
    }

private:

    ObstacleMap         *oMap;
   // mock_lcm::grid_map  *lcmMap;
    State               *egoState;

   // CollisionCheckTemplate modelTemplate;
   // CostChecker            costChecker;

    //double getCostLCMCost(State &state);

    MapType mapType;

};
