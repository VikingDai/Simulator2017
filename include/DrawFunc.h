#ifndef __DRAW_FUNC_INCLUDED__
#define __DRAW_FUNC_INCLUDED__

#ifdef __GNUG__

#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/freeglut.h>

#else 

#include <GL/freeglut.h>
#include <GL/gl.h>
#include <GL/glu.h>

#endif

#include <iostream>
#include <vector>
#include <math.h>

#include "Model.h"
//#include "msg/mock_lcm/grid_map.hpp"

using namespace std;


void drawCircle(double radius, double x, double y, double z, int step, GLfloat color[]);
void drawSolidCircle(double radius, double x, double y, double z, int step, GLfloat color[]);
void drawTrianlge(double heading, double x, double y, double base, GLfloat color[]);
void drawLine(double x, double y, double z, double xe, double ye, double ze, GLfloat color[]);
void drawRoadCenter(vector<Point2D> &centerOfRoad, double SCENE_SCALE, double drawOffset_x, double drawOffset_y);
void putText(string ss, int x, int y, int width, int height);
void putText3D(string s, int x, int y, int z);
void drawRectObstacle(list<RectObstacle> *staticObstacles, double SCENE_SCALE, double drawOffset_x, double drawOffset_y);
//void drawCostMap(mock_lcm::grid_map  *lcmMap, State egoState, double SCENE_SCALE, double drawOffset_x, double drawOffset_y);
void drawOptimalTrajectorylist(std::vector<State> state_for_opt_traj_vector, double scale, double drawOffset_x, double drawOffset_y, GLfloat color[], GLfloat color2[]);
#endif
