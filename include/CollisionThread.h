#pragma once

#include <Windows.h>
#include "Trajectory.h"
#include "Grid.h"
#include "GridCollision.h"
#include <time.h>
#include <iostream>
#include "matrix.h" // AW
//#include "../include/car.h" AW

typedef struct CollisionData {
	//std::map<int, Obstacle_tracking_container>* obstacles;
	matrix* truck;
	matrix* trailer;
	std::vector<Trajectory*>* trajSet;
} CollisionData;

void InitializeCollision(CollisionData& pCollisionData, std::vector<Trajectory*>* trajSet, matrix* tralierState, matrix* truckState);

DWORD WINAPI CollisionThread(LPVOID pCollisionParam);