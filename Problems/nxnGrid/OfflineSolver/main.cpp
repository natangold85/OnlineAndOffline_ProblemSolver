#include <iostream>		// cout, cin
#include <string>		// std::string
#include <Windows.h>	// CreateFile ..
#include <fstream>      // std::ofstream
#include <ctime>      // time
#include <algorithm>      // for_each


// possible model solutions possibilities
#include "nxnGridOfflineGlobalActions.h"
#include "nxnGridOfflineLocalActions.h"

// properties of objects
#include "../model/Coordinate.h"
#include "../model/Move_Properties.h"
#include "../model/Attacks.h"
#include "../model/Observations.h"

// objects
#include "../model/Self_Obj.h"
#include "../model/Attack_Obj.h"
#include "../model/Movable_Obj.h"
#include "../model/ObjInGrid.h"

// choose model to run
enum MODELS_AVAILABLE { NXN_LOCAL_ACTIONS, NXN_GLOBAL_ACTIONS };
// observation option extracted from Observations.h
enum OBSERVATIONS_AVAILABLE { OBSERVATION_BY_DISTANCE };
// moves option extracted from Move_Properties.h
enum MOVES_AVAILABLE { SIMPLE_MOVE, GENERAL_DIRECTION, TARGET_DERIVED, NAIVE };
// attacks option extracted from Attacks.h
enum ATTACKS_AVAILABLE { DIRECT_ATTACK };

// problem to run:

// solver params
static int s_timeOutSec = 30000;
static double s_diffPrecision = 0.001;

// world
static int s_gridSize = 4;
static int s_targetLoc = s_gridSize * s_gridSize - 1;

// self
static MODELS_AVAILABLE s_UsingModel = NXN_GLOBAL_ACTIONS;
// the model is deciding the movement properties of self
static double s_self_pDirectMove = 0.9;
static double s_self_pStay = 1 - s_self_pDirectMove;

static OBSERVATIONS_AVAILABLE s_observationProp = OBSERVATION_BY_DISTANCE;
static double s_obsDistanceFactor = 0.4;

static ATTACKS_AVAILABLE s_selfAttack = DIRECT_ATTACK;
static double s_selfAttackRange = ((double)s_gridSize) / 4;
static double s_selfPHit = 0.5;

// enemy
std::vector<Coordinate> s_enemyLocations = { Coordinate(s_gridSize - 1, s_gridSize - 1) };
static MOVES_AVAILABLE s_enemyMoveProp = TARGET_DERIVED;
static double s_enemy_pDirectMove = 0.3;
static double s_enemy_pStay = 0.3;

static ATTACKS_AVAILABLE s_enemyAttack = DIRECT_ATTACK;
static double s_enemyAttackRange = s_selfAttackRange;
static double s_enemyPHit = 0.4;

// non-involved
std::vector<Coordinate> s_nonInvLocations = { /*Coordinate(s_gridSize / 2, s_gridSize / 2)*/ };
static MOVES_AVAILABLE s_nonInvMoveProp = NAIVE;
static double s_nonInv_pStay = 0.6;

// shelter
std::vector<Coordinate> s_shelter1Loc = { Coordinate(1, 3) };
//std::vector<Coordinate> s_shelter2Loc = { Coordinate(8, 2), Coordinate(8, 1), Coordinate(7, 2), Coordinate(7, 1) };

std::vector<std::vector<Coordinate>> s_shelterLocations = { s_shelter1Loc };


// buffer size for read
const static int s_BUFFER_SIZE = 1024;

using lutSarsop = std::map<int, std::vector<double>>;

// appl solver directory location
std::string s_SOLVER_LOCATION = "..\\..\\pomdp_solver\\src\\Release\\";
static std::ofstream s_BACKUP_FILE;


void ErrorAndExit(const char * errMsg)
{
	std::cout << "ERROR : " << errMsg << " press any key to exit...";
	char c;
	std::cin >> c;
	exit(1);
}

/// run solver save policy in policty_name
void RunSolver(std::string &fname, std::string &policy_name, std::string &solverOutF)
{
	// required precision in accord to rewards
	double requiredPrecision = nxnGridOffline::s_REWARD_WIN - nxnGridOffline::s_REWARD_LOSS;
	requiredPrecision *= s_diffPrecision;
	std::string cmd = s_SOLVER_LOCATION + "pomdpsol.exe --timeout " + std::to_string(s_timeOutSec) + " --precision " + std::to_string(requiredPrecision) + " ";
	cmd += fname + " -o " + policy_name + " > " + solverOutF;
		
	int stat = system(cmd.c_str());

	if (stat != 0)
	{
		std::cerr << "error in running SARSOP solver\npress any key to exit";
		char c;
		std::cin >> c;
		exit(1);
	}
}

void RunSimulator(std::string &pomdpFName, std::string &policy_name)
{
	// run simulator with 200 simulations
	std::string cmd = s_SOLVER_LOCATION + "pomdpsim.exe --simLen 50 --simNum 50 ";
	cmd += "--policy-file " + policy_name + " " + pomdpFName;
	int stat = system(cmd.c_str());

	if (stat != 0)
	{
		std::cerr << "error in running SARSOP simulator\npress any key to exit";
		char c;
		std::cin >> c;
		exit(1);
	}
}


void ReadReward(nxnGridOffline * pomdp, lutSarsop & sarsopMap, std::string & sarsopDataFName)
{
	std::ifstream data(sarsopDataFName, std::ios::in | std::ios::binary);
	if (data.fail())
	{
		std::cout << "open sarsop file failed with error code = " << GetLastError() << "\npress any key to exit\n";
		char c;
		std::cin >> c;
		exit(0);
	}

	int size;
	data.read((char *)&size, sizeof(int));
	int numActions;
	data.read((char *)&numActions, sizeof(int));
	for (int stateCount = 0; stateCount < size; ++stateCount)
	{
		int stateIdx = pomdp->StateCount2StateIdx(stateCount);
		sarsopMap[stateIdx].resize(numActions);

		for (int action = 0; action < numActions; ++action)
		{
			double reward;
			data.read((char *)&reward, sizeof(double));
			sarsopMap[stateIdx][action] = reward;
		}

	}
}

Attack_Obj CreateEnemy(Coordinate & location)
{
	std::shared_ptr<Attack> attack;
	if (s_enemyAttack == DIRECT_ATTACK)
	{
		std::shared_ptr<Attack> a(new DirectAttack(s_enemyAttackRange, s_enemyPHit));
		attack = a;
	}
	else
		ErrorAndExit("Non-Valid enemy attack");

	std::shared_ptr<Move_Properties> movement;
	if (s_enemyMoveProp == TARGET_DERIVED)
	{
		std::shared_ptr<Move_Properties> m(new TargetDerivedMoveProperties(s_enemy_pStay, s_enemy_pDirectMove));
		movement = m;
	}
	else
		ErrorAndExit("Non-Valid enemy movement");


	
	return Attack_Obj(location, movement, attack);
}

Self_Obj CreateSelf(Coordinate & location)
{
	// init observation type
	std::shared_ptr<Observation> observation;
	if (s_observationProp == OBSERVATION_BY_DISTANCE)
	{
		std::shared_ptr<Observation> obs(new ObservationByDistance(s_obsDistanceFactor));
		observation = obs;
	}
	else
		ErrorAndExit("Non-Valid observation");
	

	// init attack type
	std::shared_ptr<Attack> attack;
	if (s_selfAttack == DIRECT_ATTACK)
	{
		std::shared_ptr<Attack> tmp(new DirectAttack(s_selfAttackRange, s_selfPHit));
		attack = tmp;
	}
	else
		ErrorAndExit("Non-Valid self attack");

	std::shared_ptr<Move_Properties> movement;
	if (s_UsingModel == NXN_GLOBAL_ACTIONS)
	{
		std::shared_ptr<Move_Properties> m(new TargetDerivedMoveProperties(s_self_pStay, s_self_pDirectMove));
		movement = m;
	}
	else if (s_UsingModel == NXN_LOCAL_ACTIONS)
	{
		std::shared_ptr<Move_Properties> m(new LowLevelMoveProperties(s_self_pDirectMove));
		movement = m;
	}
	else
		ErrorAndExit("Non-Valid self Movement");

	return Self_Obj(location, movement, attack, observation);
}

Movable_Obj CreateNInv(Coordinate & location)
{
	std::shared_ptr<Move_Properties> movement;
	if (s_nonInvMoveProp == NAIVE)
	{
		std::shared_ptr<Move_Properties> m(new NaiveMoveProperties(s_nonInv_pStay));
		movement = m;
	}
	else
		ErrorAndExit("Non-Valid enemy movement");

	// location doesn't mind because each run we change location
	return Movable_Obj(location, movement);
}

ObjInGrid CreateShelter(Coordinate & location)
{
	return ObjInGrid(location);
}

void CreateLUT(nxnGridOffline * model, std::string & prefix, lutSarsop & sarsopMap)
{
	std::string pomdpFName = prefix + ".pomdp";
	// remove pomdp file before writing it
	remove(pomdpFName.c_str());
	FILE *fptr;
	auto err = fopen_s(&fptr, pomdpFName.c_str(), "w");
	if (0 != err)
	{
		std::cout << "ERROR OPEN\n";
		exit(1);
	}
	// write pomdp file
	model->SaveInPomdpFormat(fptr);
	fclose(fptr);

	// print initial state
	std::cout << "finished writing pomdp file\n";

	// create policy and calculate duration of solver
	std::string policyName = prefix + ".policy";
	std::string solverOutFname = prefix + ".txt";
	time_t solverStart = time(nullptr);
	RunSolver(pomdpFName, policyName, solverOutFname);
	time_t solverDuration = time(nullptr) - solverStart;

	
	std::cout << "finished creating policy file\n";
	// run simulator
	RunSimulator(pomdpFName, policyName);
	std::cout << "finished running simulator\n";
	std::string sarsopDataFName = prefix + "_sarsopData.bin";
	ReadReward(model, sarsopMap, sarsopDataFName);
}

void TraverseAllShelterLoc(nxnGridOffline * pomdp, std::string & prefix, lutSarsop & sarsopMap, int shelterIdx)
{
	int gridSize = pomdp->GetGridSize();

	prefix += "S";
	for (auto s : s_shelterLocations[shelterIdx])
	{
		pomdp->SetLocationShelter(s, shelterIdx);

		int locIdx = s.GetIdx(s_gridSize);
		prefix += std::to_string(locIdx);
		
		if (shelterIdx + 1 < s_shelterLocations.size())
			TraverseAllShelterLoc(pomdp, prefix, sarsopMap, shelterIdx + 1);
		else
			CreateLUT(pomdp, prefix, sarsopMap);

		prefix.pop_back();
		if (locIdx > 9)
			prefix.pop_back();
		if (locIdx > 99)
			prefix.pop_back();
	}
}

int main()
{	
	Coordinate m(0, 0);
	nxnGridOffline * model = nullptr;

	if (s_UsingModel == NXN_LOCAL_ACTIONS)
		model = new nxnGridOfflineLocalActions(s_gridSize, s_targetLoc, CreateSelf(m), false);
	else if (s_UsingModel == NXN_GLOBAL_ACTIONS)
		model = new nxnGridOfflineGlobalActions(s_gridSize, s_targetLoc, CreateSelf(m), false);
	else
		ErrorAndExit("Non-valid model");
	
	// ADD ENEMIES
	for (auto e : s_enemyLocations)
		model->AddObj(CreateEnemy(e));

	//ADD N_INV
	for (auto n : s_nonInvLocations)
		model->AddObj(CreateNInv(n));

	//ADD SHELTERS (insert location seperatley in TraverseAllShelterLoc)
	Coordinate GenericLoc(0, 0);
	for (int i = 0; i < s_shelterLocations.size(); ++i)
		model->AddObj(CreateShelter(GenericLoc));

	// create prefix for file name
	std::string prefix = std::to_string(s_gridSize) + "x" + std::to_string(s_gridSize) + "Grid";
	prefix += std::to_string(model->CountEnemies()) + "x" + std::to_string(model->CountNInv()) + "x" + std::to_string(model->CountShelters());

	std::string lutFName(prefix);
	lutFName += "_LUT.bin";
	lutSarsop sarsopMap;
	
	// create all format and solver options for the model
	if (0 != model->CountShelters())
		TraverseAllShelterLoc(model, prefix, sarsopMap, 0);
	else
		CreateLUT(model, prefix, sarsopMap);


	// write map to lut
	int numActions = model->GetNumActions();
	std::ofstream lut(lutFName, std::ios::out | std::ios::binary);
	int mapSize = sarsopMap.size();
	lut.write((const char *)&mapSize, sizeof(int));
	lut.write((const char *)&numActions, sizeof(int));
	std::for_each(sarsopMap.begin(), sarsopMap.end(), [&lut](lutSarsop::const_reference itr)
	{
		lut.write((const char *)&itr.first, sizeof(int));
		for (auto reward : itr.second)
		{
			lut.write((const char *)&reward, sizeof(double));
		}
	});

	//std::string dest = "C:\\Users\\moshe\\Documents\\GitHub\\Despot\\" + lutFName;
	//std::string src = "C:\\Users\\moshe\\Documents\\GitHub\\Despot\\" + lutFName;

	//lut.close();
	//if (!MoveFile(src.c_str(), dest.c_str()))
	//{
	//	std::cout << "failed moving lut to main directory\n";
	//	std::cout << "press any key to exit";
	//	char c;
	//	std::cin >> c;
	//}
}
