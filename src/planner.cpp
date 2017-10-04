
#include "planner.h"

//HANDLE collisionTestSem;


Planner::Planner(int inumTimeSteps, volatile double is2final, double ikxi, double imaxOffsetD, double imaxOffsetS, int inumTrajS, int inumTrajD, double id1final, double id2final, double id3final, double is3final) {
	
	if (is2final < 0) {
		throw std::invalid_argument("s2final must be positive.");
	}
	else if (imaxOffsetD < 0) {
		throw std::invalid_argument("maxOffsteD must be positive.");
	}
	else if (imaxOffsetS < 0) {
		throw std::invalid_argument("maxOffsteS must be positive.");
	}
	else if (inumTrajS < 0 || inumTrajS % 2 == 0) { 
		throw std::invalid_argument("Invalid numTrajS, must be a positive odd number.");
	}
	else if (inumTrajD < 0 || inumTrajD % 2 == 0) {
		throw std::invalid_argument("Invalid numTrajD, must be a positive odd number.");
	}

	numTimeSteps = inumTimeSteps;
	s2final = is2final;
	kxi = ikxi;
	maxOffsetD = imaxOffsetD;
	maxOffsetS = imaxOffsetS;
	numTrajS = inumTrajS;
	numTrajD = inumTrajD;
	d1final = id1final;
	d2final = id2final;
	d3final = id3final;
	s3final = is3final;
	guessToFind = 0;
	optTrajIndex = 0;
	s1final = 1.5;//maxOffsetD + 0.1;
	numWerlingSteps = 0;
	werlingWindowEnd = 1.5;
	werlingWindowStart = 0.5;

	double sWerlingDist = 0.2;
	int numWerlingPointsD = 9;
	int numWerlingPoints = (int)round(5 * numWerlingPointsD*(ceil((werlingWindowEnd- werlingWindowStart) / sWerlingDist) + 1));

	optTraj = new Trajectory(6, numTimeSteps);
	int i;
	for (i = 0; i < numTrajD*numTrajS; ++i) {
		trajSetD.push_back(new Trajectory(4, numTimeSteps));
		trajSetD.at(i)->setValid(false);
	}
	for (i = 0; i < numTrajD*numTrajS; ++i) {
		trajSetS.push_back(new Trajectory(4, numTimeSteps));
		trajSetS.at(i)->setValid(false);
	}
	for (i = 0; i < numTrajD*numTrajS; ++i) {
		trajSet.push_back(new Trajectory(6, numTimeSteps));
		trajSet.at(i)->setValid(false);
	}
	
	for (i = 0; i < pow(numTrajD*numTrajS, numWerlingSteps); ++i) {
		trajSetDWerling.push_back(new Trajectory(4, numTimeSteps*numWerlingSteps));
		trajSetDWerling.at(i)->setValid(false);
	}
	for (i = 0; i < pow(numTrajD*numTrajS, numWerlingSteps); ++i) {
		trajSetSWerling.push_back(new Trajectory(4, numTimeSteps*numWerlingSteps));
		trajSetSWerling.at(i)->setValid(false);
	}
	for (i = 0; i < pow(numTrajD*numTrajS, numWerlingSteps); ++i) {
		trajSetWerling.push_back(new Trajectory(6, numTimeSteps*numWerlingSteps));
		trajSetWerling.at(i)->setValid(false);
	}

	J.resize(numTrajD*numTrajS);
	JWerling.resize(pow(numTrajD*numTrajS, numWerlingSteps));
	angles.resize(optTraj->cols() - 2);
}

Planner::~Planner() {
	trajSetS.clear();
	trajSetD.clear();
	trajSet.clear();
}

void Planner::generateQuinticTrajectory(double x10, double x20, double x30, double x1f, double x2f, double x3f, Trajectory* traj, double finalS1) {
	double C1 = x10;
	double C2 = x20;
	double C3 = x30/2;
	double trajdT;
	double tfTraj = (finalS1 - s1Init) / ((s2Init + s2final) / 2);
	if (tfTraj == 0) {
		trajdT = 0.01;
		tfTraj = 2;
	}

	else {
		trajdT = tfTraj / numTimeSteps;
		}
	double C4 = (4*(kxi*pow(tfTraj,5) - 180)*(C2 - x2f + 2*C3*tfTraj))/(pow(tfTraj,2)*(kxi*pow(tfTraj,5) + 720)) - (10*kxi*pow(tfTraj,2)*(C3*pow(tfTraj,2) + C2*tfTraj + C1 - x1f))/(kxi*pow(tfTraj,5) + 720) - ((2*C3 - x3f)*(kxi*pow(tfTraj,5) - 480))/(2*tfTraj*(kxi*pow(tfTraj,5) + 720));
	double C5 = (15*kxi*tfTraj*(C3*pow(tfTraj,2) + C2*tfTraj + C1 - x1f))/(kxi*pow(tfTraj,5) + 720) - ((7*kxi*pow(tfTraj,5) - 360)*(C2 - x2f + 2*C3*tfTraj))/(pow(tfTraj,3)*(kxi*pow(tfTraj,5) + 720)) + ((2*C3 - x3f)*(kxi*pow(tfTraj,5) - 180))/(pow(tfTraj,2)*(kxi*pow(tfTraj,5) + 720));
	double C6 = (3*kxi*tfTraj*(C2 - x2f + 2*C3*tfTraj))/(kxi*pow(tfTraj,5) + 720) - (6*kxi*(C3*pow(tfTraj,2) + C2*tfTraj + C1 - x1f))/(kxi*pow(tfTraj,5) + 720) - (kxi*pow(tfTraj,2)*(2*C3 - x3f))/(2*(kxi*pow(tfTraj,5) + 720));

	int i;


	double x1;
	double x2;
	double x3;

	for(i = 0; i < numTimeSteps; i++) {
		x1 = C6*pow(i*trajdT,5)  +    C5*pow(i*trajdT,4) +   C4*pow(i*trajdT,3) +   C3*pow(i*trajdT,2) + C2*i*trajdT + C1;
		x2 = 5*C6*pow(i*trajdT,4)  +  4*C5*pow(i*trajdT,3) + 3*C4*pow(i*trajdT,2) + 2*C3*i*trajdT    + C2;
		x3 =20*C6*pow(i*trajdT,3)  + 12*C5*pow(i*trajdT,2) + 6*C4*i*trajdT    + 2*C3;
		
		traj->set(0,i,x1);
		traj->set(1,i,x2);
		traj->set(2,i,x3);
		traj->set(3,i,i*trajdT);
		traj->setdT(trajdT);
		traj->setValid(true);
	}

}

void Planner::generateQuarticTrajectory(double x10, double x20, double x30, double x2f, double x3f, Trajectory* traj) {
	double C1 = x10;
	double C2 = x20;
	double C3 = x30/2;
	double C4 = (x30 - x3f)/(3*tf) - (x20 - x2f + tf*x30)/pow(tf,2);
	double C5 = (x20 - x2f + tf*x30)/(2*pow(tf,3)) - (x30 - x3f)/(4*pow(tf,2));

	int i;

	double x1;
	double x2;
	double x3;

	for(i = 0; i < tf/dT; i++) {
		x1 = C5*pow(i*dT,4) +   C4*pow(i*dT,3) +   C3*pow(i*dT,2) + C2*i*dT + C1;
		x2 = 4*C5*pow(i*dT,3) + 3*C4*pow(i*dT,2) + 2*C3*i*dT    + C2;
		x3 = 12*C5*pow(i*dT,2) + 6*C4*i*dT    + 2*C3;
		
		traj->set(0,i,x1);
		traj->set(1,i,x2);
		traj->set(2,i,x3);
		traj->set(3,i,i*dT);
	}
}


void Planner::generateQuinticSet(double x10, double x20, double x30, double x1f, double x2f, double x3f, int numTraj, std::vector<Trajectory*>& trajSet){

	int i = 0;

	Trajectory* traj;

	if (numTraj != 1) {

		double dD = 2.0*maxOffsetD/(numTraj - 1);
		double dx1f;

		for (i = 0; i < numTraj; i++) {
			traj = trajSet.at(i);
			dx1f = -maxOffsetD + dD*i;
			generateQuinticTrajectory(x10, x20, x30, x1f + dx1f, x2f, x3f, traj, s1final);
		}

	}
	else {
		traj = trajSet.at(0);
		generateQuinticTrajectory(x10, x20, x30, x1f, x2f, x3f,traj, s1final);
	}

}


void Planner::generateQuarticSet(double x10, double x20, double x30, double x2f, double x3f){
	
	int i = 0;

	Trajectory* traj;

	if (numTrajS != 1) {

		double dD = maxOffsetS/(numTrajS - 1.0);
		double dx2f;

		for (i = 0; i < numTrajS; i++) {
			traj = trajSetS.at(i);
			dx2f = maxOffsetS - dD*i;
			generateQuarticTrajectory(x10, x20, x30, x2f - dx2f, x3f, traj);
		}

	}
	else {
		traj = trajSetS.at(0);
		generateQuarticTrajectory(x10, x20, x30, x2f, x3f, traj);
	}
}

void Planner::generateQuinticSetWerling(WerlingPoints* werPoints) {

	int i = 0;
	bool crapShoot = false;

	std::vector<double*> werPointsInWindow = werPoints->getPointsInWindow(s1Init + werlingWindowStart, s1Init + werlingWindowEnd);
	std::vector<double*>::iterator it;
	double totS = werPoints->getTotalRefLength();

	if (werPointsInWindow.empty()) {
		std::cout << "No points in window" << std::endl;
	}
	else {
		for (it = werPointsInWindow.begin(); it != werPointsInWindow.end(); ++it) {
			if(*it[0] < s1Init)
			{
				generateQuinticTrajectory(s1Init, s2Init, s3Init, (*it)[0]+totS, s2final, s3final, trajSetS.at(i), (*it)[0] + totS);
				generateQuinticTrajectory(d1Init, d2Init, d3Init, (*it)[1], d2final, d3final, trajSetD.at(i), (*it)[0] + totS);
			}
			else 
			{
				generateQuinticTrajectory(s1Init, s2Init, s3Init, (*it)[0] , s2final, s3final, trajSetS.at(i), (*it)[0]);
				generateQuinticTrajectory(d1Init, d2Init, d3Init, (*it)[1], d2final, d3final, trajSetD.at(i), (*it)[0]);
			}

	
			i++;
		}
	}

	//std::cout << "Num points in Win: " << i << std::endl;
}

void Planner::generateSetConstantVelocity() {
	generateQuinticSet(d1Init, d2Init, d3Init, d1final, d2final, d3final, numTrajD, trajSetD);
	generateQuarticSet(s1Init, s2Init, s3Init, s2final, s3final);
}

void Planner::generateSetQuintic() {
	generateQuinticSet(d1Init, d2Init, d3Init, d1final, d2final, d3final, numTrajD, trajSetD);
	generateQuinticSet(s1Init, s2Init, s3Init, s1final, s2final, s3final, numTrajS, trajSetS);
}


void Planner::globalToFrenet(double x1, double x2, double theta, double v[], double a[], Trajectory* ref) {
	double tc[2]; 
	double nc[2];
	double e[2];
	double x[2] = {x1,x2};
	double etc;
	double enc;
	double vtc;
	double vnc;
	double atc;
	double anc;
	double tcnorm;
	int divInd = ref->cols()-1;
	
	int refIndex = findRefPos(x1, x2, ref);

	//tc[0] = ref->get(0, (int)fmod(refIndex + 1, divInd)) - ref->get(0, (int)fmod(refIndex - 1 + divInd, divInd));

	//tc[1] = ref->get(1, (int)fmod(refIndex + 1, divInd)) - ref->get(1, (int)fmod(refIndex - 1 + divInd, divInd));

	//c[1] = ref->get(1, (int)fmod(refIndex + 1, divInd)) - ref->get(1, (int)fmod(refIndex - 1 + divInd, divInd));

	// TODO: Probably most likely change to cos/sin
	tc[0] = cos(ref->get(thetarow, (int)fmod(refIndex, divInd)));
	tc[1] = sin(ref->get(thetarow, (int)fmod(refIndex, divInd)));

	tcnorm = norm(tc);

	tc[0] = tc[0] / tcnorm;
	tc[1] = tc[1] / tcnorm;

	nc[0] = tc[1];
	nc[1] = -1*tc[0];
	// TODO : swap - ????!?!?!?

	e[0] = x1 - ref->get(0, (int)fmod(refIndex, divInd));
	e[1] = x2 - ref->get(1, (int)fmod(refIndex, divInd));

	etc = (e[0]*tc[0] + e[1]*tc[1]);
	enc = (e[0]*nc[0] + e[1]*nc[1]);
	// TODO: stryk nämnare = 1 i etc/enc?

	vtc = (v[0]*tc[0] + v[1]*tc[1]);
	vnc = (v[0]*nc[0] + v[1]*nc[1]);
	atc = (a[0] * tc[0] + a[1] * tc[1]);
	anc = (a[0] * nc[0] + a[1] * nc[1]);

	d1Init = enc;
	d2Init = vnc;
	d3Init = anc;

	// TODO: ... +- etc i s1Init?
	s1Init = sumRefLength(ref, refIndex);
	s2Init = vtc;
	s3Init = atc;

	s1final = s1Init + 1.4;

	// TODO: Remove todo below :D

	/* TODO: Temp fix. This means that we assume that we are pointing the same way
			as the reference if the speed is very low.*/
	if (s2Init < 0.5) {
		s2Init = 0.5;
	}
}


double Planner::sumRefLength(Trajectory* ref, int refIndex) {
	double sum = 0;
	double lengthx, lengthy;
	
	for (int i = 0; i < refIndex; i++) {
		lengthx = ref->get(0,i+1) - ref->get(0,i);
		lengthy = ref->get(1,i+1) - ref->get(1,i);

		sum += sqrt(pow(lengthx,2) + pow(lengthy,2));
	}

	return sum;
}

int Planner::findRefPos(double x1, double x2, Trajectory* ref) {
	double etemp, e;
	int index = 0;
	int i;
	
	// High value so that first etemp < e will be true in first iteration.
	e = 9999999.0f;

	for (i = 0; i <  ref->cols(); i++) {
		etemp = pow(ref->get(0,i) - x1,2) + pow(ref->get(1,i) - x2, 2);
		if (etemp < e) {
			e = etemp;
			index = i;
		}
	}

	return index;
}

int Planner::findRefPosSmart(double x1, double x2, Trajectory* ref, int guess) {
	double etemp, e;
	double dx1, dx2;
	int index = 0;
	int refSize = ref->cols()-1;
	int horizon = 5; // Used to check 'horizon' steps backwards and forward of guess

					  // High value so that first etemp < e will be true in first iteration.
	e = 9999999.0f;

	int i, i_wrap;
	for (i = guess - horizon; i < guess + horizon + 1; i++) {
		i_wrap = (i % refSize + refSize) % refSize;
		dx1 = ref->get(0, i_wrap) - x1;
		dx2 = ref->get(1, i_wrap) - x2;
		etemp = pow(dx1, 2) + pow(dx2, 2);
		if (etemp < e) {
			e = etemp;
			index = i_wrap;
		}
	}
	if (e > 0.2) { // If no index around 'guess' is good enough - enforce global search
		index = findRefPos(x1, x2, ref);
	}

	return index;
}

int Planner::find(std::vector<double>* vec, double value) {
	int i;
	for (i = 1; i < (int)vec->size(); i++) {
		if (vec->at(i) > value) {
			return i-1;
		}
	}
	return -1;
}

int Planner::find(std::vector<double>* vec, double value, int guess) {
	int i;
	if (vec->at(guess) > value) {
		for (i = guess; i >= 0; --i) {
			if (vec->at(i) < value) {
				return i;
			}
		}
	}
	else {
		for (i = guess; i < (int)vec->size(); i++) {
			if (vec->at(i) > value) {
				return i - 1;
			}
		}
	}
	return find(vec, value);
}

double Planner::norm(double vec[]) {
	return sqrt(pow(vec[0],2) + pow(vec[1],2));
}

void Planner::transformToGlobal(Trajectory* d, Trajectory* s, std::vector<double>* refDistance, Trajectory* ref, Trajectory* x) {
	int refIndexTraj;
	double tc[2]; 
	double nc[2];
	double div = (*refDistance).back();
	double tcnorm;
	int divInd = ref->cols()-1;

	nc[0] = 0;
	nc[1] = 0;

	for (int i = 0; i < numTimeSteps; i++) {
		refIndexTraj = find(refDistance, fmod(s->get(0, i), div), guessToFind);
		guessToFind = refIndexTraj; // why global defined?

		//tc[0] = ref->get(0, (int)fmod(refIndexTraj + 1, divInd)) - ref->get(0, (int)fmod(refIndexTraj - 1 + divInd, divInd));
		//tc[1] = ref->get(1, (int)fmod(refIndexTraj + 1, divInd)) - ref->get(1, (int)fmod(refIndexTraj - 1 + divInd, divInd));
		tc[0] = cos(ref->get(thetarow, (int)fmod(refIndexTraj, divInd)));
		tc[1] = sin(ref->get(thetarow, (int)fmod(refIndexTraj, divInd)));
		
		// Normalize tc (and future nc that are built on tc).
		tcnorm = norm(tc);
		tc[0] = tc[0]/tcnorm;
		tc[1] = tc[1]/tcnorm;

		nc[0] = tc[1]; // We put nc in -y direction (when tc = x), because alot of the code is allready written assuming this.
		nc[1] = -1*tc[0]; // We put nc in -y direction (when tc = x), because alot of the code is allready written assuming this.

		x->set(xrow, i, ref->get(0, (int)fmod(refIndexTraj, divInd)) + d->get(0, i)*nc[0] + (fmod(s->get(0, i), div) - refDistance->at((int)fmod(refIndexTraj + 1, divInd)))*tc[0]);
		x->set(yrow, i, ref->get(1, (int)fmod(refIndexTraj, divInd)) + d->get(0, i)*nc[1] + (fmod(s->get(0, i), div) - refDistance->at((int)fmod(refIndexTraj + 1, divInd)))*tc[1]);
		x->set(vrow, i, 0); // Set to 0 for now, calculate later.
		x->set(arow, i, 0); // Set to 0 for now, calculate later.
		x->set(thetarow, i, 0); // Set to 0 for now, calculate later.
		x->set(kapparow, i, 0);		// Set to 0 for now, calculate later.
		x->setdT(s->getdT()); // We set the dT for time steps, calculated in generate quintic trajectory.
	} 
	x->setValid(true); // Not needed? TODO: Remove?
}

void Planner::transformAllToGlobal(Trajectory* ref) {
	std::vector<double> refDistance(ref->cols()-1,0);
	int i;

	refDistance.at(0) = 0;

	for (i = 1; i < ref->cols()-1; i++) {
		refDistance.at(i) = sqrt(pow(ref->get(0,i) - ref->get(0,i-1),2) + pow(ref->get(1,i) - ref->get(1,i-1),2)) + refDistance.at(i-1);
	}

	// Needed since .at(i-1) will not work...
	refDistance.erase(refDistance.begin());
	accuracyTune = 20; // Tuning fix for speed of Angle/Curvature. Higher gives faster program but less accurate.
	int sindex, dindex, trajindex = 0;

	for (sindex = 0; sindex < (int)trajSetS.size(); sindex++) {
		for (dindex = 0; dindex < (int)trajSetD.size(); dindex++) {
			// Transforms one specific trajectory, sets x,y to global coordinates, v,a,theta and kappa to 0 for later calculations.
			transformToGlobal(trajSetD.at(dindex), trajSetS.at(sindex), &refDistance, ref, trajSet.at(trajindex)); 
			//Calculates cost for the trajectory.
			J.at(trajindex) = calcCost(trajSetD.at(dindex), trajSetS.at(sindex)); 
			// Computes Angles for the given trajectory, adds it to thetarow.
			computeAnglesAccurate(trajSet.at(trajindex),trajSetD.at(dindex),trajSetS.at(sindex),ref);

			trajindex++; // To keep track of which trajectory we are on
			ReleaseSemaphore(collisionTestSem, 1, NULL); // Allows the collisionThread to check collision for this trajectory.
		}
	}

}

void Planner::transformAllToGlobalWerling(Trajectory* ref) {
	std::vector<double> refDistance(ref->cols() - 1, 0);
	int i;
	refDistance.at(0) = 0;

	for (i = 1; i < ref->cols() - 1; i++) {
		refDistance.at(i) = sqrt(pow(ref->get(0, i) - ref->get(0, i - 1), 2) + pow(ref->get(1, i) - ref->get(1, i - 1), 2)) + refDistance.at(i - 1);
	}

	// Needed since .at(i-1) will not work...
	refDistance.erase(refDistance.begin());

	int trajindex = 0;
	int j = 0;

	for (trajindex = 0; trajindex < (int)trajSetS.size(); trajindex++) {
		if (trajSetS.at(trajindex)->isValid() && trajSetD.at(trajindex)->isValid())
		{
			transformToGlobal(trajSetD.at(trajindex), trajSetS.at(trajindex), &refDistance, ref, trajSet.at(trajindex));
			/* TODO: Check if traj is invalid, maybe move call to calcCost*/
			J.at(trajindex) = calcCost(trajSetD.at(trajindex), trajSetS.at(trajindex));
			computeAnglesAccurate(trajSet.at(trajindex), trajSetD.at(trajindex), trajSetS.at(trajindex), ref);
			computeVelAccurate(trajSet.at(trajindex), trajSetD.at(trajindex), trajSetS.at(trajindex), ref);
			computeAccAccurate(trajSet.at(trajindex), trajSetD.at(trajindex), trajSetS.at(trajindex), ref);
			//computeAngles(trajSet.at(trajindex));
			checkValidity(ref);
			ReleaseSemaphore(collisionTestSem, 1, NULL);
			j++;
		}
		else
		{
			trajSet.at(trajindex)->setValid(false); 
			ReleaseSemaphore(collisionTestSem, 1, NULL);
		}
	}
	
		

}

double Planner::calcCost(Trajectory* d, Trajectory* s) {
	double Jtemp = 0;
	int i;
	double tau;
	double kTau = 1; //Design parameter for tau
	double kJerk = 100; //Design parameter for Jerk.
	double kxi = 1; //Design parameter for difference in d and sdot

	
	for (i = d->cols() - 1; i == 0; i--) {
		if (abs(d->get(0, i)) < 0.005) {
			tau = i*s->getdT(); // Calculates the earliest convergence
		}
	}	
	// TODO:: Insert u here instead of approximation.
	//Jtemp = tau / 2 * (pow((d->get(2, 1) - d->get(2, 0)) / dT, 2) + pow((d->get(2, d->cols() - 1) - d->get(2, d->cols() - 2)) / dT, 2));
	//Jtemp += tau / 2 * (pow((s->get(2, 1) - s->get(2, 0)) / dT, 2) + pow((s->get(2, s->cols() - 1) - s->get(2, s->cols() - 2)) / dT, 2));

	for (i = 1; i < d->cols(); i++)
	{
		Jtemp += pow((d->get(2, i) + d->get(2, i-1))/2 * s->getdT(), 2);
		Jtemp += pow((s->get(2, i) + s->get(2, i-1))/2 * s->getdT(), 2);
	}

	// Adds cost for jerk.
	Jtemp = kJerk*Jtemp;

	//std::cout << "Jerk Cost = " << Jtemp << std::endl;

	// Adds cost for difference in d and sdot.
	Jtemp += kxi / 2 * (pow(d->get(0, d->cols() - 1) - d1final, 2) + 10*pow(s->get(0, s->cols() - 1) - s1final, 2));

	// Adds cost for slow convergence.
	Jtemp += kTau*tau; 

	//std::cout << "Cost tot = " << Jtemp << std::endl;

	return Jtemp;
}

double Planner::compdist(double x, double y) {
	return sqrt(pow(x,2) + pow(y,2));
}

void Planner::computeVelAccurate(Trajectory* traj, Trajectory* trajD, Trajectory* trajS, Trajectory* ref)
{
	int divInd = ref->cols() - 1;
	int i;
	int refIndex = findRefPos(traj->get(0, 0), traj->get(1, 0), ref);

	double theta;
	double kappa_ref;
	double theta_ref;
	double delta_theta;
	double sdot;
	double d;
	double vel;

	for (i = 0; i < traj->cols() - 1; i++) {	
		// Calculate the index, otherwise it will get the wrong kappa_ref and theta_ref ( dont iterate )
		refIndex = findRefPosSmart(traj->get(0, i), traj->get(1, i), ref, refIndex);  
		kappa_ref = ref->get(kapparow, (int)fmod(refIndex, divInd));
		theta_ref = ref->get(thetarow, (int)fmod(refIndex, divInd));		

		// Pull theta from thetarow, calculated earlier in ComputeAnglesAccurate
		theta = traj->get(thetarow, i);
		// delta_theta calculations, differance in theta for trajectory and reference.
		delta_theta = theta - theta_ref;

		sdot = trajS->get(1, i);
		d = -trajD->get(0, i); // minus because n is placed in -y direction.

		// Dont have to check 1-K_r*d or delta_theta sizes, since we are allready on an optimal trajectory,
		// which has checked these in Angles/Curvature.
		vel = sdot*(1 - kappa_ref*d) / cos(delta_theta); 
		traj->set(vrow, i, vel); 
	}

}

void Planner::computeAccAccurate(Trajectory* traj, Trajectory* trajD, Trajectory* trajS, Trajectory* ref)
{
	int i;
	int refIndex = findRefPos(traj->get(0, 0), traj->get(1, 0),ref);
	int divInd = ref->cols() - 1;
		
	double kappa_ref;
	double kappa_ref2;

	double theta;
	double theta_ref;
	double delta_theta;

	double theta2;
	double theta_ref2;
	double delta_theta2;

	double ddelta_thetads;
	double dkappa_refds;

	double dprime;
	double sDiff;
	double d;
	double ddot;
	double sdot;
	double sddot;


	double acc = 0;

	for (i = 0; i < traj->cols() - 1; i++) {
		// Calculate the index, otherwise it will get the wrong kappa_ref and theta_ref ( dont iterate )
		refIndex = findRefPosSmart(traj->get(0, i), traj->get(1, i), ref, refIndex);

		theta_ref = ref->get(thetarow, (int)fmod(refIndex, divInd));
		theta_ref2 = ref->get(thetarow, (int)fmod(refIndex + 1, divInd));
		kappa_ref = ref->get(kapparow, (int)fmod(refIndex, divInd));
		kappa_ref2 = ref->get(kapparow, (int)fmod(refIndex + 1, divInd));

		theta = traj->get(thetarow, i);
		theta2 = traj->get(thetarow, i + 1);

		delta_theta = theta - theta_ref;
		delta_theta2 = theta2 - theta_ref2;

		sdot = trajS->get(1, i);
		ddot = -trajD->get(1, i); // minus because n = -y

		sDiff = trajS->get(0, i + 1) - trajS->get(0, i);

		ddelta_thetads = (delta_theta2 - delta_theta) / sDiff;
		dkappa_refds = (ref->get(kapparow, (int)fmod(refIndex + 1, divInd)) - ref->get(kapparow, (int)fmod(refIndex, divInd))) / sDiff;

		sddot = trajS->get(2, i);
		d = -trajD->get(0, i);  // minus because n = -y

		if (sDiff >= 0.0001 && sdot >= 0.0001) {
			dprime = ddot / sdot;
			acc = sddot*(1 - kappa_ref*d) / cos(delta_theta) + pow(sdot,2) * ((1 - kappa_ref*d)*tan(delta_theta)*ddelta_thetads - (dkappa_refds*d + kappa_ref*dprime));
		}
		traj->set(arow, i, acc);
	}

}

bool Planner::chooseOptTraj() {
	int i;
	int minPos = 0;
	int count = 0;
	bool temp = false;
	double min = 999999.0f;
	for (i = 0; i < (int)J.size(); ++i) {
		if (J.at(i) < min && trajSet.at(i)->isValid()) {
			minPos = i;
			min = J.at(i);
			temp = true;			
		}
		if(trajSet.at(i)->isValid())
			count++;
	}

	optTrajIndex = minPos;

	for (int j = 0; j < trajSet.at(minPos)->cols(); ++j) {
		optTraj->set(0, j, trajSet.at(minPos)->get(0, j));
		optTraj->set(1, j, trajSet.at(minPos)->get(1, j));
		optTraj->set(vrow, j, trajSet.at(minPos)->get(vrow, j));
		optTraj->set(arow, j, trajSet.at(minPos)->get(arow, j));
		optTraj->set(thetarow, j, trajSet.at(minPos)->get(thetarow, j));
		optTraj->set(kapparow, j, trajSet.at(minPos)->get(kapparow, j));
	}
	// Sets the optTrajs dT and t_final properly. (copies from trajSet where the trajectory is being copied from).
	double dTTemp = trajSet.at(minPos)->getdT();
	double tfTemp = trajSet.at(minPos)->getTf();
	
	optTraj->setdTtf(dTTemp, tfTemp);

	return temp;
}

bool Planner::chooseOptTrajWerling() {
	int i;
	int minPos = 0;
	int count = 0;
	bool temp = false;
	double min = 999999.0f;
	for (i = 0; i < (int)JWerling.size(); ++i) {
		if (JWerling.at(i) < min && trajSetWerling.at(i)->isValid()) {
			minPos = i;
			min = JWerling.at(i);
			temp = true;
		}
		if (trajSetWerling.at(i)->isValid())
			count++;
	}

	optTrajIndex = minPos;

	for (int j = 0; j < numTimeSteps; ++j) {
		optTraj->set(0, j, trajSetWerling.at(minPos)->get(0, j));
		optTraj->set(1, j, trajSetWerling.at(minPos)->get(1, j));
		optTraj->set(vrow, j, trajSetWerling.at(minPos)->get(vrow, j));
		optTraj->set(arow, j, trajSetWerling.at(minPos)->get(arow, j));
		optTraj->set(thetarow, j, trajSetWerling.at(minPos)->get(thetarow, j));
		optTraj->set(kapparow, j, trajSetWerling.at(minPos)->get(kapparow, j));
	}
	// Sets the optTrajs dT and t_final properly. (copies from trajSet where the trajectory is being copied from).
	double dTTemp = trajSetWerling.at(minPos)->getdT();
	double tfTemp = trajSetWerling.at(minPos)->getTf();

	optTraj->setdTtf(dTTemp, tfTemp);

	return temp;
}

void Planner::computeAnglesAccurate(Trajectory* traj, Trajectory* trajD, Trajectory* trajS, Trajectory* ref) {
	int i;
	int refIndex = findRefPos(traj->get(0, 0), traj->get(1, 0), ref);
	int divInd = ref->cols()-1;
	int counter = 0;	

	double theta = 0;
	double theta_ref;
	double kappa_ref;

	double ddot;
	double d;

	double delta_theta;
	double sdot;
	double dprime;
	for (i = 0; i < traj->cols() - 1; i++) {
		
		ddot = -trajD->get(1, i);
		sdot = trajS->get(1, i);
		if (abs(sdot) >= 0.001) {
			dprime = ddot / sdot;
			theta_ref = ref->get(thetarow, (int)fmod(refIndex, divInd));
			kappa_ref = ref->get(kapparow, (int)fmod(refIndex, divInd));
			
			refIndex = findRefPosSmart(traj->get(0, i), traj->get(1, i), ref, refIndex);

			d = -trajD->get(0, i); // minus because n = -y
			if (abs(1 - kappa_ref*d) < 0.000005) {
				traj->set(kapparow, 0, -10);
				break;
			}
			theta = atan2(dprime, 1 - kappa_ref*d) + theta_ref;
			delta_theta = theta - theta_ref;
			if (abs(delta_theta) >= pi / 2 - 0.000005) {
				traj->set(kapparow, 0, -10);
				break;
			}
		}	
		traj->set(thetarow, i, theta);	
	}
}

void Planner::checkValidity(Trajectory* ref) {
	int i;

	for (i = 0; i < (int)trajSet.size(); i++) {
		if (trajSet.at(i)->get(kapparow, 0) == -10) {
			trajSet.at(i)->setValid(false);
		}
	}
}

void Planner::checkCurvature(Trajectory* ref) {
	int i;
	for (i = 0; i < (int)trajSet.size(); i++) {
		if (trajSet.at(i)->isValid()) {
			computeCurvatureAccurate(trajSet.at(i), trajSetD.at(i), trajSetS.at(i), ref);
			//computeCurvature(trajSet.at(i));
			if (trajSet.at(i)->get(kapparow, 0) == -10) {
				trajSet.at(i)->setValid(false);
			}
		}
	}
}

void Planner::computeCurvatureAccurate(Trajectory* traj, Trajectory* trajD, Trajectory* trajS, Trajectory* ref) {
	int i;
	int divInd = ref->cols()-1;
	int refIndex = findRefPos(traj->get(0, 0), traj->get(1, 0), ref);
	int counter = 0;

	double theta;
	double theta_ref;
	double delta_theta;
	double crv = 0;
	double kappa_ref;
	double dkappa_refds;
	double dprime;
	double dbis;
	double d;
	double ddot;
	double dddot;
	double s;
	double sdot;
	double sddot;
	double sDiff;
	double kappaRefDiff;
	for (i = 0; i < traj->cols() - 1; i++) {
		/*counter++;
		if (counter == accuracyTune) {
			counter = 0;
			refIndex = findRefPos(traj->get(0, i), traj->get(1, i), ref);
		}
		else
			refIndex++;*/
		refIndex = findRefPosSmart(traj->get(0, i), traj->get(1, i), ref, refIndex);


		theta_ref = ref->get(thetarow, (int)fmod(refIndex, divInd));
		kappa_ref = ref->get(kapparow, (int)fmod(refIndex, divInd));

		theta = traj->get(thetarow, i);
		delta_theta = theta - theta_ref;
				
		sDiff = trajS->get(0, i + 1) - trajS->get(0, i);

		if (sDiff > 0.00001) {
			kappaRefDiff = ref->get(kapparow, (int)fmod(refIndex + 1, divInd)) - ref->get(kapparow, (int)fmod(refIndex, divInd));
			dkappa_refds = kappaRefDiff / sDiff;
		}
		else
			dkappa_refds = 0;

		// d , s
		s = trajS->get(0, i);
		d = -trajD->get(0, i);
		sdot = trajS->get(1, i);
		ddot = -trajD->get(1, i);
		sddot = trajS->get(2, i);
		dddot = -trajD->get(2, i);
		if (abs(sdot) >= 0.001) {
			// d'
			dprime = ddot / sdot;
			// d''
			dbis = (dddot - dprime*sddot) / (sdot*sdot);
			if ((1 - kappa_ref*d) >= 0.000005) {
				crv = cos(delta_theta) / (1 - kappa_ref*d)*(
					pow(cos(delta_theta), 2)/(1 - kappa_ref*d)*(dbis + (dkappa_refds*d + kappa_ref*dprime)*tan(delta_theta)) + kappa_ref);
			}
			else {
				traj->set(kapparow, 0, -10);
				break;
			}
		}
		if (crv < MAX_CURVATURE && crv > -MAX_CURVATURE) {
			traj->set(kapparow, i, crv);
		}
		else {
			traj->set(kapparow, 0, -10);
			break;
		}
			
	}
}


bool Planner::useRefInstead(double x1, double x2, double t, double v[], double a[], Trajectory* ref) {
	int i;
	int refIndex = findRefPos(x1, x2, ref);
	int divInd = ref->cols();
	double speedModifier = 1;
	double accModifier = 1;
	for (i = 0; i < trajSet.at(1)->cols(); i++) {
		trajSet.at(0)->set(0, i, ref->get(0, (int)fmod(refIndex + i, divInd)));
		trajSet.at(0)->set(1, i, ref->get(1, (int)fmod(refIndex + i, divInd)));
		trajSet.at(0)->set(2, i, ref->get(2, (int)fmod(refIndex + i, divInd)));
		trajSet.at(0)->set(3, i, ref->get(3, (int)fmod(refIndex + i, divInd)));
		trajSet.at(0)->set(4, i, ref->get(4, (int)fmod(refIndex + i, divInd)));
		trajSet.at(0)->set(5, i, ref->get(5, (int)fmod(refIndex + i, divInd)));
	}

	trajSet.at(0)->setValid(true);
	for (i = 1; i < (int)trajSet.size(); i++){
		trajSet.at(i)->setValid(false);
	}
	for (i = 0; i < (int)trajSet.size(); i++) {
		ReleaseSemaphore(collisionTestSem, 1, NULL); // just for sake of sake.
	}	
	optTrajIndex = 0; 
	for (i = 0; i < trajSet.at(0)->cols(); i++) {
		optTraj->set(0, i, trajSet.at(0)->get(0, i));
		optTraj->set(1, i, trajSet.at(0)->get(1, i));
		optTraj->set(2, i, trajSet.at(0)->get(2, i)*speedModifier);
		optTraj->set(3, i, trajSet.at(0)->get(3, i)*accModifier);
		optTraj->set(4, i, trajSet.at(0)->get(4, i));
		optTraj->set(5, i, trajSet.at(0)->get(5, i));
	}
	WaitForSingleObject(collisionTestDone, INFINITE); // Here we wait for collision to be done.
	// Sets the optTrajs dT and t_final properly. (copies from trajSet where the trajectory is being copied from).
	double dTTemp = ref->get(6, 1); // Fetches the first none-0 timestamp which defines dT.
	double tfTemp = ref->get(6, trajSet.at(0)->cols()-1); // Fetches the final time for the ref-traj.
	optTraj->setdTtf(dTTemp, tfTemp);

	if (trajSet.at(0)->isValid()) {
		return true;
	}
	else {
		return false;
	}

}

void Planner::generateWerlingSteps(double x1, double x2, double theta, double v[], double a[], Trajectory* ref, WerlingPoints * werPoints) {
	int trajSize = trajSet.at(0)->cols() - 1;
	double xInit;
	double yInit;
	double thetaInit;
	double vInit[] = { 0,0 };
	double aInit[] = { 0,0 };
	clock_t startTime, finishTime, startTime2, finishTime2;



	//
	for (int werlingStep = 1; werlingStep <= numWerlingSteps; werlingStep++) {
		//std::cout << "Werling step: " << werlingStep << std::endl;
		for (int i = 0; i < pow(trajSet.size(), werlingStep - 1); i++) {

			startTime = clock();

			// Send the estimated states in first werling step //
			if (werlingStep == 1) {
				globalToFrenet(x1, x2, theta, v, a, ref);
			}

			// Assign all valid trajectories starting coordinates for next trajectory iteration //
			else if (trajSetWerling.at(i*numTrajD*numTrajS)->isValid()) {     // Do we really need to check this again?

				xInit = trajSetWerling.at(i*numTrajD*numTrajS)->get(xrow, trajSize);

				yInit = trajSetWerling.at(i*numTrajD*numTrajS)->get(yrow, trajSize);

				thetaInit = trajSetWerling.at(i*numTrajD*numTrajS)->get(thetarow, trajSize);

				vInit[0] = trajSetWerling.at(i*numTrajD*numTrajS)->get(vrow, trajSize)*cos(thetaInit);
				vInit[1] = trajSetWerling.at(i*numTrajD*numTrajS)->get(vrow, trajSize)*sin(thetaInit);

				aInit[0] = trajSetWerling.at(i*numTrajD*numTrajS)->get(arow, trajSize)*cos(thetaInit);
				aInit[1] = trajSetWerling.at(i*numTrajD*numTrajS)->get(arow, trajSize)*sin(thetaInit);



				// Transform to Frenet frame
				globalToFrenet(xInit, yInit, thetaInit, vInit, aInit, ref);


			}

			// Generate new trajectories from each starting coordinates //
			generateQuinticSetWerling(werPoints);

			// Transform back to global coordinates, calculates cost for all new trajectories that are valid //
			transformAllToGlobalWerling(ref);

			startTime2 = clock();
			// Check for collision 

			WaitForSingleObject(collisionTestDone, INFINITE);
			finishTime2 = clock();
			//std::cout << "Collision test time: " << double((finishTime2 - startTime2)) / CLOCKS_PER_SEC << std::endl;

			checkCurvature(ref);

			for (i = 0; i < trajSetS.size(); i++) {
				trajSetS.at(i)->setValid(false);
				trajSetD.at(i)->setValid(false);
			}

			// For all generated trajectories
			for (int trajCount = 0; trajCount < (int)trajSet.size(); trajCount++) {
				// Do following for all valid trajectories
				if (trajSet.at(trajCount)->isValid()) {

					// if we are not at the final step
					if (werlingStep < numWerlingSteps) {

						// for each single trajectory, copy trajectory object from trajSet and place for 231*(numWerlingsteps-1) places
						for (int j = 0; j < trajSet.size(); j++) {

							// Add trajectory from trajSet vector to trajSetWerling vector //
							//trajSetWerling.at(i*numTrajD*numTrajS + trajCount*numTrajD*numTrajS + j)->addWerling(trajSet.at(trajCount), werlingStep, trajSize);

							// Assign costs to new trajectories, what do we assign it to? //
							JWerling.at(i*numTrajD*numTrajS + trajCount*numTrajD*numTrajS + j) += J.at(trajCount);

							// If we are on the first werling step set, all valid trajectories that are copied to trajsetwerling are set to true. Assign costs
							if (werlingStep == 1) {
								trajSetWerling.at(trajCount*numTrajD*numTrajS + j)->setValid(true);
								trajSetWerling.at(i*numTrajD*numTrajS + trajCount)->setdTtf(trajSet.at(trajCount)->getdT(), trajSet.at(trajCount)->getTf());
								JWerling.at(i*numTrajD*numTrajS + trajCount*numTrajD*numTrajS + j) = J.at(trajCount);
							}
						}
					}
					// If we are on the last step we do not generate a new dimension of subtrajectories, we simply add the new ones.
					else {
						//trajSetWerling.at(i*numTrajD*numTrajS + trajCount)->addWerling(trajSet.at(trajCount), werlingStep, trajSize);
						JWerling.at(i*numTrajD*numTrajS + trajCount) += J.at(trajCount);

						// If we numWerlingSteps = 1
						if (werlingStep == 1) {
							trajSetWerling.at(i*numTrajD*numTrajS + trajCount)->setValid(true);
							trajSetWerling.at(i*numTrajD*numTrajS + trajCount)->setdTtf(trajSet.at(trajCount)->getdT(), trajSet.at(trajCount)->getTf());
							JWerling.at(i*numTrajD*numTrajS + trajCount) = J.at(trajCount);
						}
					}
				}
				else {
					// If trajectory is not valid set false
					trajSetWerling.at(i*numTrajD*numTrajS + trajCount)->setValid(false);
				}
			}
			finishTime = clock();
			//	std::cout << "ONE step generation time: " << double((finishTime - startTime)) / CLOCKS_PER_SEC << std::endl;
		}
	}
}

bool Planner::calculateNewRefQuintic(double x1, double x2, double theta, double v[], double a[], Trajectory* ref, WerlingPoints * werPoints) {
	
	clock_t start, stop;
	start = clock();
	unsigned int i;

	globalToFrenet(x1, x2, theta, v, a, ref); // Calculate init/final values for the trajectories in frenet frame used in QuinticSet.

	generateQuinticSetWerling(werPoints); // Generates trajectories to the avaliable werlingpoints inside the werling_window

	transformAllToGlobalWerling(ref); // Transforms all trajSetS and trajSetD to global coordinates and puts it into trajSet.

	WaitForSingleObject(collisionTestDone, INFINITE); // Semaphore, waits for collision_test for trajSet to be done continuing.
	 
	checkCurvature(ref); // Excludes all trajectories with to high curvature, Theta, or otherwise unwanted trajectories.
	
	for (i = 0; i < trajSetS.size(); i++) {
		trajSetS.at(i)->setValid(false); // Set them all invalid for next time. 
		trajSetD.at(i)->setValid(false); // (so that we dont calculate with these next time aswell incase we have fewer trajs next iteration).
	}

	if (!chooseOptTraj()) { // Checks which traj in trajSet has the lowest cost and loads it into optTraj.
		return false;
	}
	/*
	if (PRINT_PLANNER_TRAJS) // Click 'd' or 'D' to toggle this to on, will print planners trajectories to file!
	{
		printTrajectories();
		PRINT_PLANNER_TRAJS = false;
	} AW */

	stop = clock(); // Comment line below in to get the time for a planning cycle.
	//std::cout << "Total time: " << stop - start << std::endl;
	return true;

}

bool Planner::calculateNewRefMultWerling(double x1, double x2, double theta, double v[], double a[], Trajectory * ref, WerlingPoints * werPoints)
{
	clock_t start, stop;
	start = clock();

	generateWerlingSteps(x1, x2, theta, v, a, ref, werPoints);

	if (!chooseOptTrajWerling()) {
		return false;
	}

	stop = clock();
	std::cout << "Total time: " << stop - start  << std::endl;
	
	return true;
}

void Planner::printTrajectories() {
	int i;
	FILE* pFile;
	std::string filename = "trajset.txt";
	pFile = fopen((datadir + filename).c_str(), "w");
	std::vector<Trajectory*>::iterator it;
	for (it = trajSet.begin(); it != trajSet.end(); ++it) {
		for (i = 0; i < (*it)->cols(); ++i) {
			fprintf(pFile, "%f ", (*it)->get(0, i));
		}
		fprintf(pFile, "\r\n");
		for (i = 0; i < (*it)->cols(); ++i) {
			fprintf(pFile, "%f ", (*it)->get(1, i));
		}
		for (i = 0; i < (*it)->cols(); ++i) {
			if((*it)->isValid())
				fprintf(pFile, "%i ", 1);
			else
				fprintf(pFile, "%i ", 0);
		}
		fprintf(pFile, "\r\n\r\n");
	}
	fclose(pFile);
	//throw;
}

