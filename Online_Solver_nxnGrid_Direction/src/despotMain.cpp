#include <fstream>      // std::ofstream

/// tui class
#include "../include/despot/simple_tui.h"

/// models available
#include "nxnGridGlobalActions.h"
#include "nxnGridLocalActions.h"

/// for nxnGrid

// properties of objects
#include "Coordinate.h"
#include "Move_properties.h"
#include "Attacks.h"
#include "Observations.h"

// objects
#include "Attack_Obj.h"
#include "Movable_Obj.h"
#include "Self_Obj.h"
#include "ObjInGrid.h"

using namespace despot;

// choose model to run
enum MODELS_AVAILABLE { NXN_LOCAL_ACTIONS, NXN_GLOBAL_ACTIONS };
// observation option extracted from Observations.h
enum OBSERVATIONS_AVAILABLE { OBSERVATION_BY_DISTANCE };
// moves option extracted from Move_Properties.h
enum MOVES_AVAILABLE { SIMPLE_MOVE, GENERAL_DIRECTION_MOVE, TARGET_DERIVED_MOVE, NAIVE_MOVE };
// attacks option extracted from Attacks.h
enum ATTACKS_AVAILABLE { DIRECT_ATTACK };


// lut properties
static std::vector<std::string> s_LUTFILENAMES{  "10x10Grid1x0x1_LUT_POMDP.bin"}; // "5x5Grid2x0x1_LUT_POMDP.bin", 
static std::vector<int> s_LUT_GRIDSIZE{ 10};
static std::vector<nxnGrid::CALCULATION_TYPE> s_CALCTYPE{ nxnGrid::CALCULATION_TYPE::WO_NINV }; // , nxnGrid::CALCULATION_TYPE::ONE_ENEMY

// parameters of model:
static const bool s_ONLINE_ALGO = true;
static const bool s_RESAMPLE_FROM_LAST_OBS = false;
static const bool s_VBS_EVALUATOR = false;
static const bool s_TO_SEND_TREE = false;
static const int s_PERIOD_OF_DECISION = 1; // sending action not in every decision
static const int s_SEARCH_PRIOD = 1;
static const int s_PORT_SEND_TREE = 5678;
static const int s_PORT_VBS = 5432;

// world simulating
static const int s_ONLINE_GRID_SIZE = 10;

// self
static MODELS_AVAILABLE s_ACTIONS_TYPE = NXN_GLOBAL_ACTIONS;
static const bool s_IS_ACTION_MOVE_FROM_ENEMY_EXIST = false;
// the model is deciding the movement properties of self
static double s_self_pMove = 0.9;

static OBSERVATIONS_AVAILABLE s_observationProp = OBSERVATION_BY_DISTANCE;
static double s_obsDistanceFactor = 0.4;


static ATTACKS_AVAILABLE s_selfAttack = DIRECT_ATTACK;
static double s_selfAttackRange = ((double)s_ONLINE_GRID_SIZE) / 4;
static double s_selfPHit = 0.5;

// enemy
std::vector<Coordinate> s_enemyLocations = { Coordinate(s_ONLINE_GRID_SIZE - 1, s_ONLINE_GRID_SIZE - 1) };
static MOVES_AVAILABLE s_enemyMoveProp = TARGET_DERIVED_MOVE;
double s_enemySpeed = 0.8;
double s_enemyTowardTarget = 0.4;
double s_enemyPStop = 0.3;
double s_enemyPRandomChangeOfDirection = 0.2;

static ATTACKS_AVAILABLE s_enemyAttack = DIRECT_ATTACK;
static double s_enemyAttackRange = s_selfAttackRange;
static double s_enemyPHit = 0.4;

// non-involved
std::vector<Coordinate> s_nonInvLocations = { /*Coordinate(s_gridSize / 2, s_gridSize / 2)*/ };
static MOVES_AVAILABLE s_nonInvMoveProp = NAIVE_MOVE;
double s_nonInvSpeed = 0.5;
double s_nonInvPRandomChangeOfDirection = 0.3;
double s_nonInvPStop = 0.4;

// shelter
static std::vector<Coordinate> s_shelter1Loc = { Coordinate(1, 3) };
static std::vector<std::vector<Coordinate>> s_shelterLocations = { s_shelter1Loc };


void ReadOfflineLUT(std::string & lutFName, std::map<STATE_TYPE, std::vector<double>> &offlineLut);
void Run(int argc, char* argv[], std::string & outputFName, int numRuns);
void InitObjectsLocations(std::vector<std::vector<int>> & objVec, int gridSize);

Attack_Obj CreateEnemy();
Movable_Obj CreateNInv();
Self_Obj CreateSelf();
ObjInGrid CreateShelter();

void ErrorAndExit(const char * errMsg)
{
	std::cout << "ERROR : " << errMsg << " press any key to exit...";
	char c;
	std::cin >> c;
	exit(1);
}

/// solve nxn Grid problem
class NXNGrid : public SimpleTUI {
public:
	NXNGrid() {}

	DSPOMDP* InitializeModel(option::Option* options) override
	{
		// create nxnGrid problem

		// init static members
		DetailedState::InitStatic();
		Self_Obj self = CreateSelf();
		int targetLoc = s_ONLINE_GRID_SIZE * s_ONLINE_GRID_SIZE - 1;

		std::vector<std::vector<int>> objVec(5);
		InitObjectsLocations(objVec, s_ONLINE_GRID_SIZE);

		nxnGrid *model;
/*		if (s_ACTIONS_TYPE == NXN_LOCAL_ACTIONS)
			model = new nxnGridLocalActions(s_ONLINE_GRID_SIZE, targetLoc, self, objVec);
		else */if (s_ACTIONS_TYPE == NXN_GLOBAL_ACTIONS)
			model = new nxnGridGlobalActions(s_ONLINE_GRID_SIZE, targetLoc, self, objVec, s_IS_ACTION_MOVE_FROM_ENEMY_EXIST);
		else
		{
			std::cout << "model not recognized... exiting!!\n";
			exit(0);
		}
		// add objects to model

		model->AddObj(CreateEnemy());
		//model->AddObj(CreateEnemy(0, 0, s_onlineGridSize));

		model->AddObj(CreateNInv());
		//model->AddObj(CreateNInv(0, 0));

		model->AddObj(CreateShelter());
		return model;
	}

	void InitializeDefaultParameters() override {}
};

int main(int argc, char* argv[]) 
{
	int numRuns = 20;
	srand(time(NULL));

	int vbsPort = s_VBS_EVALUATOR ? s_PORT_VBS : -1;
	int treePort = s_TO_SEND_TREE ? s_PORT_SEND_TREE : -1;

	nxnGrid::InitUDP(vbsPort, treePort);
	nxnGrid::InitStaticMembers(s_ONLINE_ALGO, s_PERIOD_OF_DECISION, s_RESAMPLE_FROM_LAST_OBS);
	Globals::config.time_per_move = s_SEARCH_PRIOD;

	// using lut code:
	//// init lut
	//std::map<STATE_TYPE, std::vector<double>> offlineLut;
	//ReadOfflineLUT(s_LUTFILENAMES[0], offlineLut);
	//nxnGrid::InitLUT(offlineLut, s_LUT_GRIDSIZE[0], s_CALCTYPE[0]);
	//// create output file

	//std::string outputFName(s_LUTFILENAMES[0]);
	//// pop ".bin"
	//outputFName.pop_back();
	//outputFName.pop_back();
	//outputFName.pop_back();
	//outputFName.pop_back();

	////outputFName.append("_resultOffline.txt");
	//outputFName.append("_result.txt");
	//Run(argc, argv, outputFName, numRuns);

	// init lut
	std::map<STATE_TYPE, std::vector<double>> offlineLutNaive;
	nxnGrid::InitLUT(offlineLutNaive, s_ONLINE_GRID_SIZE, nxnGrid::WITHOUT);
	// create output file

	std::string outputFNameNaive("Naive");
	outputFNameNaive.append("_result.txt");
	Run(argc, argv, outputFNameNaive, numRuns);


	char c;
	std::cout << "result written succesfully. press any key to exit\n";
	std::cin >> c;
	return 0;
}

void ReadOfflineLUT(std::string & lutFName, std::map<STATE_TYPE, std::vector<double>> &offlineLut)
{
	std::ifstream readLut(lutFName, std::ios::in | std::ios::binary);
	if (readLut.fail())
	{
		std::cout << "failed open lut file for write\n\n\n";
		exit(1);
	}
	else
	{
		int size;
		readLut.read(reinterpret_cast<char *>(&size), sizeof(int));
		int numActions;
		readLut.read(reinterpret_cast<char *>(&numActions), sizeof(int));
		for (int i = 0; i < size; ++i)
		{
			int state;
			readLut.read(reinterpret_cast<char *>(&state), sizeof(int));
			
			std::vector<double> rewards(numActions);
			for (int a = 0; a < numActions; ++a)
			{
				readLut.read(reinterpret_cast<char *>(&rewards[a]), sizeof(double));
			}

			if ((!s_IS_ACTION_MOVE_FROM_ENEMY_EXIST) & s_ACTIONS_TYPE == NXN_GLOBAL_ACTIONS)
				rewards.erase(rewards.begin() + 2);

			offlineLut[state] = rewards;
		}
		if (readLut.bad())
		{
			std::cout << "failed write lut\n\n\n";
			exit(1);
		}
		else
			std::cout << "lut written succesfuly\n\n\n";

		readLut.close();
	}
}

void Run(int argc, char* argv[], std::string & outputFName, int numRuns)
{
	remove(outputFName.c_str());
	std::ofstream output(outputFName.c_str(), std::ios::out);
	if (output.fail())
	{
		std::cerr << "failed open output file";
		exit(1);
	}

	// run model numRuns times
	output << "results for naive online. num runs = " << numRuns << "\n";
	for (size_t i = 0; i < numRuns; i++)
	{
		std::cout << "\n\n\trun #" << i << ":\n";
		output << "\n\n\trun #" << i << ":\n";
		NXNGrid().run(argc, argv, output);
		output.flush();
	}

	if (output.fail())
	{
		std::cerr << "error in write output file";
		exit(1);
	}
}

void InitObjectsLocations(std::vector<std::vector<int>> & objVec, int gridSize)
{
	int obj = 0;

	// insert self locations
	objVec[obj].emplace_back(0);
	++obj;

	// insert enemies locations
	std::vector<int> enemy1Loc{ 99, 98, 89, 88};
	objVec[obj] = enemy1Loc;
	++obj;

	//std::vector<int> enemy2loc{ 26, 27, 36, 37 };
	//objVec[obj] = enemy2loc;
	//++obj;

	std::vector<int> nonInv1Loc{ 55, 56, 65, 66 };
	objVec[obj] = nonInv1Loc;
	++obj;

	std::vector<int> shelter1Loc{ 62,63,72,73 };
	objVec[obj] = shelter1Loc;

}
Attack_Obj CreateEnemy()
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
	if (s_enemyMoveProp == TARGET_DERIVED_MOVE)
	{
		std::shared_ptr<Move_Properties> m(new TargetDerivedMoveProperties(s_enemySpeed, s_enemyTowardTarget, s_enemyPStop, s_enemyPRandomChangeOfDirection));
		movement = m;
	}
	else
		ErrorAndExit("Non-Valid enemy movement");



	return Attack_Obj(Coordinate(), movement, attack);
}

Self_Obj CreateSelf()
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
	if (s_ACTIONS_TYPE == NXN_GLOBAL_ACTIONS)
	{
		std::shared_ptr<Move_Properties> m(new SimpleMoveProperties(s_self_pMove));
		movement = m;
	}
	else if (s_ACTIONS_TYPE == NXN_LOCAL_ACTIONS)
	{
		std::shared_ptr<Move_Properties> m(new SimpleMoveProperties(s_self_pMove));
		movement = m;
	}
	else
		ErrorAndExit("Non-Valid self Movement");

	return Self_Obj(Coordinate() , movement, attack, observation);
}

Movable_Obj CreateNInv()
{
	std::shared_ptr<Move_Properties> movement;
	if (s_nonInvMoveProp == NAIVE_MOVE)
	{
		std::shared_ptr<Move_Properties> m(new NaiveMoveProperties(s_nonInvSpeed, s_nonInvPRandomChangeOfDirection, s_nonInvPStop));
		movement = m;
	}
	else
		ErrorAndExit("Non-Valid enemy movement");

	// location doesn't mind because each run we change location
	return Movable_Obj(Coordinate(), movement);
}

ObjInGrid CreateShelter()
{
	return ObjInGrid(Coordinate());
}
