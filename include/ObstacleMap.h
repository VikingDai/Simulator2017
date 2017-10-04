#pragma once

#include <iostream>
#include <list>

#include "Geometry.h"



#include "ObstacleMap.h"

using namespace std;
class RectObstacle
{
	public:
		Point2D pos;
		double width, height, diagonal;
		double orientation;
		double vx;
		double vy;
		
		vector<LineSegment> perimiter;

		RectObstacle();
		RectObstacle(Point2D pos_in, double width, double height, double velocity_x, double velocity_y);
		RectObstacle(Point2D pos_in, double width, double height, double angle, double velocity_x, double velocity_y);
		RectObstacle(Point2D pos_in_LL,Point2D pos_in_HR, double velocity_x, double velocity_y);
};


 class ObstacleMap 
 {

public:
  
	/****************************************************************/
	/*** Public varibles ********************************************/
	/****************************************************************/
    list<RectObstacle> staticObstacles;
	list<RectObstacle> dynamicObstacles;
	
	/****************************************************************/
	/*** Constructor destructor *************************************/
	/****************************************************************/
    ObstacleMap();   
    ~ObstacleMap();

	/****************************************************************/
	/*** Public methods *********************************************/
	/****************************************************************/
    void initialize();  

	void insert_obstacle(RectObstacle rect_obstacle_in);
	list<RectObstacle> predic_position_of_dynamic_obstacle(double time);
	void step(double Ts);

	void insert_obstacles(list<RectObstacle> rect_obstacles_in);
	
        
};
