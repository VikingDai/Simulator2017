
#ifndef OptimalTrajectory_H_
#define OptimalTrajectory_H_

#include <iostream>
#include <vector>
#include <queue>
#include <list>
#include <set>
#include <string>
#include <fstream>
//#include "matrix.h"
#include "PlannerTypes.h"
#define M_PI 3.14159265358979323846

// States for the car /AW
typedef struct Optimal_trajectory_info 
{
	float x;
	float y;
	float v;
	float a;
	float heading;
	float kappa;
	float t;
} RefInfo;


using namespace std;


class OptimalTrajectory
{
	public:
	//Defalut constructor
	OptimalTrajectory();

	//Constructor that specifies which file the reference information can be read
	OptimalTrajectory(std::string fName);

	~OptimalTrajectory();

	void read_optimal_trajectory(string fName);

	/** Function that reads information from the file with the reference trajectory.
		The information must be on the form x y v th thd uh us and the points should
		be ordered */
	Optimal_trajectory_info* get_optimal_trajectory_position(int i) 
								{ return optimal_trajectory_vector[i];}

	/** Function that cleares the integral */
	void clear_integral() {sum_of_integration = 0;}

	int get_optimal_vector_size() { return optimal_trajectory_vector.size();}

	int find_index(double x, double y);

	void get_optimal_pos(State& state, State& zhat); //state should be Matrix&

	std::vector<State> get_optimal_trajectory_state_vector(){return optimal_trajectory_state_vector;}

	State get_optimal_trajectory_state(int i) 
								{ return optimal_trajectory_state_vector[i];}

	float integration_from_to(int from_index,int to_index);

	int get_index_from_state_add_length(int index,float length);

	//AW
	void set_state(double x, double y, double heading, int index);

private:
	/** Pointer to the frameCounter */
	std::vector<Optimal_trajectory_info*> optimal_trajectory_vector;

	Optimal_trajectory_info start_point;

	std::vector<State> optimal_trajectory_state_vector;

	/** Used to get integral in the reference error */
	float sum_of_integration;

	/** Check if it is the first run (then searched all the track) */
	bool first_run;

	//position in the ref_vec given state.
	int index;


};

#endif