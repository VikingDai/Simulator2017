#include "OptimalTrajectory.h"


/* Default constructor */
OptimalTrajectory::OptimalTrajectory()
{
	first_run = true;
	//deltaIndex = 40;
	sum_of_integration = 0;
	index = 0;
	//frameCount = fCount;
	//refFile.open(datadir+"ref.txt",fstream::out);
	/* Print header */
	//char tmpWrite[256];
	//sprintf_s(tmpWrite,"Measures from reference interface\nframeCount r_e v th_e intr_e\n");
	//refFile.write((const char*)tmpWrite,strlen(tmpWrite));
	//return;
}

/* Constructor that specifies which file the reference information can be read */
OptimalTrajectory::OptimalTrajectory(string fName)
{
	first_run = true;
	//deltaIndex = 40;
	read_optimal_trajectory(fName);
	sum_of_integration = 0;
	index = 0;
	//frameCount = fCount;
	//refFile.open(""+"ref.txt",fstream::out);
	/* Print header */
	//char tmpWrite[256];
	//sprintf_s(tmpWrite,"Measures from reference interface\nframeCount r_e v th_e intr_e\n");
	//refFile.write((const char*)tmpWrite,strlen(tmpWrite));
	//return;
}

/* Destructor */
OptimalTrajectory::~OptimalTrajectory()
{
	for(int k = 0; k < optimal_trajectory_vector.size(); k++)
	{
		/* Deallocate all the memory */
		delete optimal_trajectory_vector.at(k);
	}
	//refFile.close();
}



/* Function that reads information from the file with the reference trajectory */
void OptimalTrajectory::read_optimal_trajectory(string fName)
{

	Optimal_trajectory_info optInfo;
	char line[256];

	/* Open the file to read from */
	fstream inputFile;
	fName = ""+fName;
	inputFile.open(fName.c_str(),fstream::in);

	if(!inputFile.is_open())
	{
		cout << "Couldn't open file " << fName << endl;
		return;
	}


	while(!inputFile.eof())
	{
		inputFile.getline(line,256);

		/* Extract information to a string */
		 //Isaks trajektoria
		/*int nrOfWritten = sscanf(line,"%f %f %f %f %f %f %f",&rInfo.x,&rInfo.y,&rInfo.v,&rInfo.th,
			&rInfo.thd,&rInfo.ug,&rInfo.us);
		if(nrOfWritten != 7)
			cout << "Wrong number of elements in reference\n";*/
		
		// 2012 ref traj

		int nrOfWritten = sscanf(line,"%f %f %f %f %f %f %f",&optInfo.x,&optInfo.y, &optInfo.v, &optInfo.a, &optInfo.heading, &optInfo.kappa, &optInfo.t);
		if(nrOfWritten != 7)
		{
			cout << "Wrong number of elements in reference\n";
		}
		else
		{
		optimal_trajectory_vector.push_back(new RefInfo(optInfo));
		State state=State(-100*optInfo.x, -100*optInfo.y, optInfo.v, optInfo.a, optInfo.heading-M_PI, optInfo.kappa, optInfo.t);
		optimal_trajectory_state_vector.push_back(state);
		}
	}
	
	/* WHY DO I HAVE TO DO THIS !! */
	optimal_trajectory_vector.pop_back();

	inputFile.close();
}


/* Function that searches for the index corresponding to the reference point 
	that is closest to the car */
int OptimalTrajectory::find_index(double x, double y)
{
	int N = optimal_trajectory_vector.size();


	/* Initiate minDist to something large */
	float min_distance = 100000;
	int min_distance_index=1;
	float dist_2;
	State state_info;

	/* Search through the vector to find the best possible point */
	for(int i = 1; i<=N; i++)
	{
		state_info = optimal_trajectory_state_vector.at(i);
	
		dist_2 = (x-state_info.x)*(x-state_info.x)+(y-state_info.y)*(y-state_info.y);
		if(dist_2 < min_distance)
		{
			min_distance = dist_2;
			min_distance_index = i;
		}
	}

	return min_distance_index;
}



float OptimalTrajectory::integration_from_to(int from_index,int to_index)
{
	float delta_x,delta_y,dist;
	float length=0;

	int N=optimal_trajectory_state_vector.size();

	if(from_index > to_index)
	{
		for(int i=from_index; i<N; i++)
		{
			delta_x = optimal_trajectory_state_vector[i+1].x-optimal_trajectory_state_vector[i].x;
			delta_y = optimal_trajectory_state_vector[i+1].y-optimal_trajectory_state_vector[i].y;
			dist=sqrt(delta_x*delta_x+delta_y*delta_y);
			length+=dist;
		}
		for(int i=1;i<to_index;i++)
		{
			delta_x = optimal_trajectory_state_vector[i+1].x-optimal_trajectory_state_vector[i].x;
			delta_y = optimal_trajectory_state_vector[i+1].y-optimal_trajectory_state_vector[i].y;
			dist=sqrt(delta_x*delta_x+delta_y*delta_y);
			length+=dist;
		}
		//Add missing between end och first point.
		delta_x = optimal_trajectory_state_vector[1].x-optimal_trajectory_state_vector[N].x;
		delta_y = optimal_trajectory_state_vector[1].y-optimal_trajectory_state_vector[N].y;
		dist=sqrt(delta_x*delta_x+delta_y*delta_y);
		length+=dist;
	}
	else
	{
		for(int i=from_index; i<to_index; i++)
		{
			delta_x = optimal_trajectory_state_vector[i+1].x-optimal_trajectory_state_vector[i].x;
			delta_y = optimal_trajectory_state_vector[i+1].y-optimal_trajectory_state_vector[i].y;
			dist=sqrt(delta_x*delta_x+delta_y*delta_y);
			length+=dist;
		}
	}
	return length;
}

int OptimalTrajectory::get_index_from_state_add_length(int index,float length)
{
	float temp_length=0;
	int temp_index=index;
	int N=optimal_trajectory_state_vector.size();

	while(temp_length<length)
	{
		if(temp_index==N)
		{
			temp_length+=integration_from_to(temp_index,1);
			temp_index=1;
		}
		else
		{
			temp_length+=integration_from_to(temp_index,temp_index+1);
			temp_index++;
		}
	}

	return temp_index;
}

// AW, test
void OptimalTrajectory::set_state(double x, double y,double heading, int index)
{
	State state = State(-100 * x, -100 * y, 1.12, 0, heading - M_PI, 0, 0);
	optimal_trajectory_state_vector.push_back(state);
}