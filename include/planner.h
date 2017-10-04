/** 
	Planner
	Automatic Control LiTH

	Filename: Planner.h

	Description:
		

	Version:	Date:		  Changes:							
		1.0		2015 HT		  First version
	@author PN, IK 2015
	

*/
#ifndef PLANNER_H_
#define PLANNER_H_
#define _USE_MATH_DEFINES
/* Maxmimum curvature trajectories can have before becomming invalid */
#define MAX_CURVATURE 6.0f

#include "Trajectory.h"
#include "WerlingPoints.h"
#include "globaldefines.h"
#include <vector>
#include <math.h>
#include <list>
#include <deque>
#include <Windows.h>
#include <iostream> 
#include <time.h>

/* Extern handle for sempahore handling colission testing */
extern HANDLE collisionTestSem;
extern HANDLE collisionTestDone;

class Planner
{
public:
	Planner(int inumTimeSteps, double is2final, double ikxi, double imaxOffsetD, double imaxOffsetS, int inumTrajS = 11, int inumTrajD = 11, double id1final = 0, double id2final = 0, double id3final = 0, double is3final = 0);
	~Planner();

	int findRefPos(double x1, double x2, Trajectory* ref);
	/** @brief Transform all combinations of d ans s trajectories
	*		stored in trajSetD and trajSetS to global coordinates.
 	*  @param ref Matrix of reference trajectory
 	*  @return Void
 	*/


	int Planner::findRefPosSmart(double x1, double x2, Trajectory* ref, int guess);

 	/** @brief Get pointer to the vector containg pointers to all trajectories.
 	*  @return vector<Trajectory*>* containg all trajectories.
 	*/
	std::vector<Trajectory*>* getTrajSet() {return &trajSet;};
	/** @brief Get pointer to the vector containg pointers to all lateral trajectories.
 	*  @return vector<Trajectory*>* containg all lateral trajectories.
 	*/
	std::vector<Trajectory*> getTrajSetD() {return trajSetD;};
	/** @brief Get pointer to the vector containg pointers to all longitudinal trajectories.
 	*  @return vector<Trajectory*>* containg all longitudinal trajectories.
 	*/
	std::vector<Trajectory*> getTrajSetS() {return trajSetS;};

	/** @brief Get pointer to the current optimal trajectory.
 	*  @return Trajectory* containg the optimal trajectory.
 	*/
	Trajectory* getOptTraj() {return optTraj;};

	/** @brief Transform all combinations of d ans s trajectories
	*		stored in trajSetD and trajSetS to global coordinates.
 	*  @param ref Matrix of reference trajectory
 	*  @return Void
 	*/
	void transformAllToGlobal(Trajectory* ref);

	/** @brief Generate trajectories with constant velocity for s and d, stored in 
	*		trajSetD and trajSetS respectivly.
 	*  @return Void
 	*/
	void generateSetConstantVelocity();
	
	/** @brief Generate trajectories with fixed stopping point for s and d, stored in 
	*		trajSetD and trajSetS respectivly.
 	*  @return Void
 	*/
	void generateSetQuintic();

	/** @brief Transform all combinations of d ans s trajectories
	*		stored in trajSetD and trajSetS to global coordinates.
	*  @param x1 Starting position in x
	*  @param x2 Starting position in y
	*  @param t Current time, currenlty not in use
	*  @param v Speed in x- and y-direction
	*  @param a Acceleration in x- and y-direction
 	*  @param ref Trajectory contaning the reference trajectory
 	*  @return Void
 	*/
	void globalToFrenet(double x1, double x2, double t, double v[], double a[], Trajectory* ref);

	/** @brief Compute the angle (in global cooridnates) for every time step. Results are stored
	*		in the trajectory in the 'angle' row, index by ::access_traj.
	*  @param traj Pointer to the trajectory which angles should be calculate for,
	Trajectory with D and S aswell as the ref.
	*  @return Void
	*/

	void computeAnglesAccurate(Trajectory* traj, Trajectory* trajD, Trajectory* trajS, Trajectory* ref);

	/** @brief Looping through all trajectories, checking wheter the maximum value 
	*		exceeds ::MAX_CURVATURE. 
 	*  @param Void
 	*  @return Void
 	*/
	void checkCurvature(Trajectory* ref);

	// Does a more timeefficient check than findRefPos.
	// it check for a position on the reference that is closer to the current posiiton then the Index is.

	/** @brief Compute the curvature for a specific trajectory, results are stored in
	*		the trjaectory in kapparow defined by ::access_traj. Will abort if value exceeds
	*		::MAX_CURVATURE. If abortet, -10 will be the first element in kapparow.
	*  @param traj Pointer to the trajectory
	*  @return Void
	*/
	void computeCurvatureAccurate(Trajectory* traj, Trajectory* trajD, Trajectory* trajS, Trajectory* ref);

	/** @brief Choose the optimal trajectory from the current set. Sets the pointer ::optTraj
	*		accordingly if at least on trajectory is marked vaild. Return true on success, false otherwise.
 	*  @param Void
 	*  @return bool True if a valid trajectory was found, false otherwise
 	*/
	bool chooseOptTraj();

	bool chooseOptTrajWerling();

	/** @brief Compute the acceleration for every point in the trajectory. Stored in the trajectories
	*		row arow, defined by ::access_traj.
 	*  @param 
 	*  @return 
 	*/
	void computeAccAccurate(Trajectory* traj, Trajectory* trajD, Trajectory* trajS, Trajectory* ref);


	/** @brief Main function called by the main loop. Will generate and choose and optimal
	*		trajectory with constant velocity. Returns true on success, false otherwise.
	*  @param x1 Starting position in x.
	*  @param x2 Starting position in y.
	*  @param t Starting time.
	*  @param v Current velocity.
	*  @param a Current acceleration
	*  @param ref Reference trajectory.
	*  @return bool True if successful, false otherwise
	*/
	bool Planner::useRefInstead(double x1, double x2, double t, double v[], double a[], Trajectory* ref);


	/** @brief Main function called by the main loop. Will generate and choose and optimal 
	*		trajectory with constant velocity. Returns true on success, false otherwise.
 	*  @param x1 Starting position in x.
 	*  @param x2 Starting position in y.
 	*  @param t Starting time.
 	*  @param v Current velocity.
 	*  @param a Current acceleration
 	*  @param ref Reference trajectory.
 	*  @return bool True if successful, false otherwise
 	*/
	//bool calculateNewRefQuartic(double x1, double x2, double theta, double v[], double a[], Trajectory* ref, WerlingPoints * werPoints);
	
	/** @brief Main function called by the main loop. Will generate and choose and optimal 
	*		trajectory with fixed longitudinal position. Returns true on success, false otherwise.
 	*  @param x1 Starting position in x.
 	*  @param x2 Starting position in y.
 	*  @param t Starting time.
 	*  @param v Current velocity.
 	*  @param a Current acceleration
 	*  @param ref Reference trajectory.
 	*  @return bool True if successful, false otherwise
 	*/
	bool calculateNewRefQuintic(double x1, double x2, double theta, double v[], double a[], Trajectory* ref, WerlingPoints * werPoints);

	bool calculateNewRefMultWerling(double x1, double x2, double theta, double v[], double a[], Trajectory * ref, WerlingPoints * werPoints);

	/** @brief Print all trajectories in global coordinates to file. Used for debugging.
	*		See MATLAB script to plot.
 	*  @param Void
 	*  @return Void
 	*/
	void printTrajectories();

	void generateQuinticSetWerling(WerlingPoints* werPoints);

	void transformAllToGlobalWerling(Trajectory* ref);

	void checkValidity(Trajectory* ref);

	/** @brief Compute velocity for the optimal trajectory stored in ::optTraj. Results are stored
	*		in ::optTraj in vrow, defined by ::access_traj.
	*  @param Traj Trajectory with the distance from starting point to every other
	*		point in the reference trajectory.
	*  @return Void
	*/
	void Planner::computeVelAccurate(Trajectory* Traj, Trajectory* trajD, Trajectory* trajS, Trajectory* ref);

private:
	/* Pointer to the current optimal trajecory. Note the using this pointer after a new
		planning cycle has been started is illegal. */
	Trajectory* optTraj;

	double dT = 0.01;
	double tf = 2;
	int numTimeSteps;

	double maxOffsetD;
	double maxOffsetS;

	int numTrajS;
	int numTrajD;
	double kxi;

	int guessToFind = 0;

	double d1Init;
	double d2Init;
	double d3Init;

	double s1Init;
	double s2Init;
	double s3Init;

	double d1final;
	double d2final;
	double d3final;

	double s1final = 0.5;
	double s2final;
	double s3final;

	int optTrajIndex;
	int accuracyTune;

	int numWerlingSteps;
	double werlingWindowEnd;
	double werlingWindowStart;

	std::vector<double> J;
	std::vector<double> JWerling;
	std::vector<double> angles;

	/** @brief List with all longitudinal trajektories */
	std::vector<Trajectory*> trajSetD;
	/** @brief List with all the lateral trajektories. */
	std::vector<Trajectory*> trajSetS;
	
	/** @brief List with all trajektories in global coordinates. */
	std::vector<Trajectory*> trajSet;

	std::vector<Trajectory*> trajSetDWerling;
	/** @brief List with all the lateral trajektories. */
	std::vector<Trajectory*> trajSetSWerling;

	/** @brief List with all trajektories in global coordinates. */
	std::vector<Trajectory*> trajSetWerling;

	/** @brief  Sum the length of reference from element 0 to refIndex
	*	TODO: Check if x1 and x2 should be used
 	*  @param ref Matrix of reference trajectory
 	*  @param refIndex Index to which length should be calculated
 	*  @return Sum of the lengths to refIndex
 	*/
	double sumRefLength(Trajectory* ref, int refIndex);

	/** @brief 
 	*  @param 
 	*  @return 
 	*/
	double calcCost(Trajectory* d, Trajectory* s);

	/** @brief 
 	*  @param 
 	*  @return 
 	*/
	double compdist(double x, double y);
		
	/** @brief  Generate quintic trajectory, starting at 
	*		[x10,x20,x30] and ending at [x1f,x2f,x3f]. Will be
	*		stored in traj.
	*
 	*  @param x10 Starting position.
 	*  @param x20 Starting velocity.
 	*  @param x30 Starting acceleration.
 	*  @param x1f Ending position.
 	*  @param x2f Ending velocity.
 	*  @param x3f Ending acceleration.
 	*  @param traj Pointer to where the trajectory will be stored.
 	*  @return Void.
 	*/
	void generateQuinticTrajectory(double x10, double x20, double x30, double x1f, double x2f, double x3f,Trajectory* traj, double sFinal1);
	
	/** @brief  Generate quartic trajectory, starting at 
	*		[x10,x20,x30] and ending at [x2f,x3f]. Will be
	*		stored in traj.
	*
 	*  @param x10 Starting position.
 	*  @param x20 Starting velocity.
 	*  @param x30 Starting acceleration.
 	*  @param x2f Ending velocity.
 	*  @param x3f Ending acceleration.
 	*  @param traj Pointer to where the trajectory will be stored.
 	*  @return Void.
 	*/
	void generateQuarticTrajectory(double x10, double x20, double x30, double x2f, double x3f, Trajectory* traj);
	
	/** @brief  Generate set of quintic trajectories, starting at 
	*		[x10,x20,x30] and ending at [x1f + offset,x2f,x3f]. 
	*		Offset is stored as a class variabel. Will be
	*		stored in ::trajSetD.
	*
 	*  @param x10 Starting position.
 	*  @param x20 Starting velocity.
 	*  @param x30 Starting acceleration.
 	*  @param x1f Ending position.
 	*  @param x2f Ending velocity.
 	*  @param x3f Ending acceleration.
 	*  @param traj Pointer to where the trajectory will be stored.
 	*  @return Void.
 	*/
	void generateQuinticSet(double x10, double x20, double x30, double x1f, double x2f, double x3f, int numTraj, std::vector<Trajectory*>& trajSet);
		
	/** @brief  Generate set of quartic trajectories, starting at 
	*		[x10,x20,x30] and ending at [x2f + offset,x3f].
	*		Offset is stored as a class variable. Will be
	*		stored in ::trajSetS.
	*
 	*  @param x10 Starting position.
 	*  @param x20 Starting velocity.
 	*  @param x30 Starting acceleration.
 	*  @param x2f Ending velocity.
 	*  @param x3f Ending acceleration.
 	*  @param traj Pointer to where the trajectory will be stored.
 	*  @return Void.
 	*/
	void generateQuarticSet(double x10, double x20, double x30, double x2f, double x3f);

	/** @brief  Transform a trajectory to global coordinates.
	*
 	*  @param d Trajectory in lateral.
 	*  @param s Trajectory in longitudunal.
 	*  @param refDistance Vector contating distance on ref.
 	*  @param ref Vector contaning the reference.
 	*  @param Matrix of the trajectory in global coordinates.
	*  @return Void
 	*/

	void Planner::transformToGlobal(Trajectory* d, Trajectory* s, std::vector<double>* refDistance, Trajectory* ref, Trajectory* x);

	/** @brief  Find first element with value of element strict
	*			greater than value.
	*
 	*  @param vec Vector to search.
 	*  @param value Value to find
 	*  @return Index
 	*/
	int find(std::vector<double>* vec, double value);

	/** @brief  Find first element with value of element strict
	*			greater than value, starting at a guess.
	*
 	*  @param vec Vector to search.
 	*  @param value Value to find
 	*  @param guess Initial guess
 	*  @return Index
 	*/
	int find(std::vector<double>* vec, double value, int guess);

	/** @brief  Calculated the norm of a 2-dim vector.
	*
 	*  @param vec 2-dim vector.
 	*  @return Norm.
 	*/
	double norm(double vec[]);

	void generateWerlingSteps(double x1, double x2, double theta, double v[], double a[], Trajectory* ref, WerlingPoints * werPoints);


};

#endif