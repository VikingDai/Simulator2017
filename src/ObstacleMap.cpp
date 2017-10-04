#include <iostream>
#include <list>

#include "ObstacleMap.h"

using namespace std;

RectObstacle::RectObstacle()
{
	pos			= Point2D();
	width		= 1;
	height		= 1;
	diagonal	= sqrt(width*width + height*height);
	orientation	= 0;
	perimiter.push_back(LineSegment(Point2D(-0.5, -0.5), Point2D(-0.5,  0.5)));
	perimiter.push_back(LineSegment(Point2D(-0.5,  0.5), Point2D( 0.5,  0.5)));
	perimiter.push_back(LineSegment(Point2D( 0.5,  0.5), Point2D( 0.5, -0.5)));
	perimiter.push_back(LineSegment(Point2D( 0.5, -0.5), Point2D(-0.5, -0.5)));

}
RectObstacle::RectObstacle(Point2D pos_in, double width_in, double height_in, double velocity_x, double velocity_y)
{
	pos			= pos_in;
	width		= width_in;
	height		= height_in;
	diagonal	= sqrt(width*width + height*height);
	orientation = 0;
	vx=velocity_x;
	vy=velocity_y;

	perimiter.push_back(LineSegment(Point2D(pos.x-width/2, pos.y-height/2), Point2D(pos.x-width/2, pos.y+height/2)));
	perimiter.push_back(LineSegment(Point2D(pos.x-width/2, pos.y+height/2), Point2D(pos.x+width/2, pos.y+height/2)));
	perimiter.push_back(LineSegment(Point2D(pos.x+width/2, pos.y+height/2), Point2D(pos.x+width/2, pos.y-height/2)));
	perimiter.push_back(LineSegment(Point2D(pos.x+width/2, pos.y-height/2), Point2D(pos.x-width/2, pos.y-height/2)));
}

RectObstacle::RectObstacle(Point2D pos_in, double width_in, double height_in, double angle, double velocity_x, double velocity_y)
{
	pos			= pos_in;
	width		= width_in;
	height		= height_in;
	diagonal	= sqrt(width*width + height*height);
	orientation = angle;
	vx=velocity_x;
	vy=velocity_y;

	Point2D p1 = Point2D(-width/2, -height/2).rotate(orientation);
	Point2D p2 = Point2D(-width/2, height/2).rotate(orientation);
	Point2D p3 = Point2D(width/2, height/2).rotate(orientation);
	Point2D p4 = Point2D(width/2, -height/2).rotate(orientation);

	perimiter.push_back(LineSegment(pos+p1, pos-p2));
	perimiter.push_back(LineSegment(pos+p2, pos+p3));
	perimiter.push_back(LineSegment(pos+p3, pos+p4));
	perimiter.push_back(LineSegment(pos+p4, pos-p1));

}

RectObstacle::RectObstacle(Point2D pos_in_LL,Point2D pos_in_HR, double velocity_x, double velocity_y)
{
	double x=-(pos_in_LL.x+pos_in_HR.x)/2;	
	double y=-(pos_in_LL.y+pos_in_HR.y)/2;	
	pos=Point2D(x,y);
	orientation=0;
	vx=velocity_x;
	vy=velocity_y;

	width		= pos_in_HR.x-pos_in_LL.x;
	height		= pos_in_HR.y-pos_in_LL.y;
	diagonal	= sqrt(width*width + height*height);

	perimiter.push_back(LineSegment(Point2D(pos.x-width/2, pos.y-height/2), Point2D(pos.x-width/2, pos.y+height/2)));
	perimiter.push_back(LineSegment(Point2D(pos.x-width/2, pos.y+height/2), Point2D(pos.x+width/2, pos.y+height/2)));
	perimiter.push_back(LineSegment(Point2D(pos.x+width/2, pos.y+height/2), Point2D(pos.x+width/2, pos.y-height/2)));
	perimiter.push_back(LineSegment(Point2D(pos.x+width/2, pos.y-height/2), Point2D(pos.x-width/2, pos.y-height/2)));
}

ObstacleMap::ObstacleMap()
{
}

ObstacleMap::~ObstacleMap()
{
}

void ObstacleMap::initialize()
{

	//////////////////////////////////////////////////////// MUST CHANGE /////////////////////////////////////////////////////
	//Racetrack
	//Outer boundaries
	staticObstacles.push_back(RectObstacle(Point2D(-180,-10), Point2D(180,0),0,0));
	staticObstacles.push_back(RectObstacle(Point2D(-180,0), Point2D(-173,230),0,0));
	staticObstacles.push_back(RectObstacle(Point2D(-180,226), Point2D(180,240),0,0));
	staticObstacles.push_back(RectObstacle(Point2D(173,-10), Point2D(180,240),0,0));


	//Inner boundaries
	//Real values: staticObstacles.push_back(RectObstacle(Point2D(-127,46), Point2D(-114,180)));
	//Real values: staticObstacles.push_back(RectObstacle(Point2D(-127,46), Point2D(127,57)));
	staticObstacles.push_back(RectObstacle(Point2D(-124,46), Point2D(-114,180),0,0));
	staticObstacles.push_back(RectObstacle(Point2D(-124,46), Point2D(127,57),0,0));

	//Real values: staticObstacles.push_back(RectObstacle(Point2D(-67,107), Point2D(-54,230)));
	staticObstacles.push_back(RectObstacle(Point2D(-67,112), Point2D(-54,230),0,0));

	staticObstacles.push_back(RectObstacle(Point2D(-7,46), Point2D(6,180),0,0));

	//Real values: staticObstacles.push_back(RectObstacle(Point2D(-7,167), Point2D(127,180)));
	staticObstacles.push_back(RectObstacle(Point2D(-7,167), Point2D(122,180),0,0));
	staticObstacles.push_back(RectObstacle(Point2D(53,107), Point2D(173,120),0,0));

}

void ObstacleMap::insert_obstacle(RectObstacle rect_obstacle_in)
{
		dynamicObstacles.push_back(rect_obstacle_in);
		cout << "Speed of obstacle in x: " << dynamicObstacles.front().vx << " in y:" << dynamicObstacles.front().vy << endl;
}


void ObstacleMap::step(double Ts){

	list<RectObstacle>::iterator obstacle;

	for (obstacle = dynamicObstacles.begin(); obstacle != dynamicObstacles.end(); obstacle++)
	{
		Point2D pos;
		//Udate obstacles
		pos.x = obstacle->pos.x;
		pos.y = obstacle->pos.y;

		double delta_x=Ts*obstacle->vx;
		double delta_y=Ts*obstacle->vy;

		pos.x+=delta_x;
		pos.y+=delta_y;

		obstacle->pos=pos;


		obstacle->perimiter.clear();
		double width		= obstacle->width;
		double height		= obstacle->height;
		double diagonal	= sqrt(width*width + height*height);

		Point2D p1 = Point2D(-width/2, -height/2).rotate(obstacle->orientation);
		Point2D p2 = Point2D(-width/2, height/2).rotate(obstacle->orientation);
		Point2D p3 = Point2D( width/2, height/2).rotate(obstacle->orientation);
		Point2D p4 = Point2D( width/2, -height/2).rotate(obstacle->orientation);

		obstacle->perimiter.push_back(LineSegment(pos+p1, pos-p2));
		obstacle->perimiter.push_back(LineSegment(pos+p2, pos+p3));
		obstacle->perimiter.push_back(LineSegment(pos+p3, pos+p4));
		obstacle->perimiter.push_back(LineSegment(pos+p4, pos-p1));

	}	
}


list<RectObstacle> ObstacleMap::predic_position_of_dynamic_obstacle(double time){

	list<RectObstacle> list_rec_obstacle_prediction;
	list<RectObstacle>::iterator obstacle;

	for (obstacle = dynamicObstacles.begin(); obstacle != dynamicObstacles.end(); obstacle++)
	{
		Point2D pos;
		pos.x = obstacle->pos.x;
		pos.y = obstacle->pos.y;

		double delta_x=time*obstacle->vx;
		double delta_y=time*obstacle->vy;

		pos.x+=delta_x;
		pos.y+=delta_y;

		RectObstacle temp_obstacle;

		temp_obstacle.pos=pos;

		double width		= obstacle->width;
		double height		= obstacle->height;
		double diagonal	= sqrt(width*width + height*height);

		temp_obstacle.width=width;
		temp_obstacle.height=height;
		temp_obstacle.diagonal=diagonal;
		temp_obstacle.vx=obstacle->vx;
		temp_obstacle.vy=obstacle->vy;
		temp_obstacle.orientation=obstacle->orientation;

		Point2D p1 = Point2D(-width/2, height/2).rotate(temp_obstacle.orientation);
		Point2D p2 = Point2D(-width/2, height/2).rotate(temp_obstacle.orientation);
		Point2D p3 = Point2D( width/2, height/2).rotate(temp_obstacle.orientation);
		Point2D p4 = Point2D( width/2, height/2).rotate(temp_obstacle.orientation);

		temp_obstacle.perimiter.push_back(LineSegment(pos+p1, pos-p2));
		temp_obstacle.perimiter.push_back(LineSegment(pos+p2, pos+p3));
		temp_obstacle.perimiter.push_back(LineSegment(pos+p3, pos+p4));
		temp_obstacle.perimiter.push_back(LineSegment(pos+p4, pos-p1));

		list_rec_obstacle_prediction.push_back(temp_obstacle);
	}	

	return list_rec_obstacle_prediction;
}

void ObstacleMap::insert_obstacles(list<RectObstacle> rect_obstacles_in)
{
	dynamicObstacles.clear();
	dynamicObstacles.resize(rect_obstacles_in.size());
	dynamicObstacles = rect_obstacles_in;
}