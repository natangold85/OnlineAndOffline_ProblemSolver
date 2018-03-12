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

// solver type :
static const std::string solverType = "Parallel_POMCP"; //"POMCP";  //"Parallel_POMCP"; // "user"
static const bool s_PARALLEL_RUN = solverType == "Parallel_POMCP";
enum MODELS_AVAILABLE { NXN_LOCAL_ACTIONS, NXN_GLOBAL_ACTIONS };

// lut properties
static std::vector<std::string> s_LUTFILENAMES{  "10x10Grid1x0x1_LUT_POMDP.bin"}; // "5x5Grid2x0x1_LUT_POMDP.bin", 
static std::vector<int> s_LUT_GRIDSIZE{ 10};
static std::vector<nxnGrid::CALCULATION_TYPE> s_CALCTYPE{ nxnGrid::CALCULATION_TYPE::WO_NINV }; // , nxnGrid::CALCULATION_TYPE::ONE_ENEMY

// parameters of model:
static MODELS_AVAILABLE s_ACTIONS_TYPE = NXN_GLOBAL_ACTIONS;
static const bool s_ONLINE_ALGO = true;
static const bool s_TREE_REUSE = true;
static const bool s_RESAMPLE_FROM_LAST_OBS = true;
static const bool s_VBS_EVALUATOR = false;
static const bool s_TO_SEND_TREE = false;
static const bool s_IS_ACTION_MOVE_FROM_ENEMY_EXIST = false;

static const int s_ONLINE_GRID_SIZE = 10;
static const int s_PERIOD_OF_DECISION = 1; // sending action not in every decision
static const int s_SEARCH_PRIOD = 1;
static const int s_PORT_SEND_TREE = 5678;
static const int s_PORT_VBS = 5432;

void ReadOfflineLUT(std::string & lutFName, std::map<STATE_TYPE, std::vector<double>> &offlineLut);
void Run(int argc, char* argv[], std::string & outputFName, int numRuns);
void InitObjectsLocations(std::vector<std::vector<int>> & objVec, int gridSize);

Attack_Obj CreateEnemy(int x, int y, int gridSize);
Movable_Obj CreateNInv(int x, int y);
Self_Obj CreateSelf(int x, int y, int gridSize);
ObjInGrid CreateShelter(int x, int y);

/// solve nxn Grid problem
class NXNGrid : public SimpleTUI {
public:
	NXNGrid() {}

	DSPOMDP* InitializeModel(option::Option* options) override
	{
		// create nxnGrid problem

		// init static members
		DetailedState::InitStatic();

		Self_Obj self = CreateSelf(0, 0, s_ONLINE_GRID_SIZE);
		int targetLoc = s_ONLINE_GRID_SIZE * s_ONLINE_GRID_SIZE - 1;

		std::vector<std::vector<int>> objVec(5);
		InitObjectsLocations(objVec, s_ONLINE_GRID_SIZE);

		nxnGrid *model;
		if (s_ACTIONS_TYPE == NXN_LOCAL_ACTIONS)
			model = new nxnGridLocalActions(s_ONLINE_GRID_SIZE, targetLoc, self, objVec);
		else if (s_ACTIONS_TYPE == NXN_GLOBAL_ACTIONS)
			model = new nxnGridGlobalActions(s_ONLINE_GRID_SIZE, targetLoc, self, objVec, s_IS_ACTION_MOVE_FROM_ENEMY_EXIST);
		else
		{
			std::cout << "model not recognized... exiting!!\n";
			exit(0);
		}
		// add objects to model

		model->AddObj(CreateEnemy(0, 0, s_ONLINE_GRID_SIZE));
		//model->AddObj(CreateEnemy(0, 0, s_onlineGridSize));

		model->AddObj(CreateNInv(0, 0));
		//model->AddObj(CreateNInv(0, 0));

		model->AddObj(CreateShelter(0, 0));
		return model;
	}

	void InitializeDefaultParameters() override {}
};

int main(int argc, char* argv[]) 
{
	int numRuns = 20;
	
	/// seed main thread random num
	Random ran((unsigned)time(NULL));
	Random::s_threadSafeRand[GetCurrentThreadId()] = ran;

	int vbsPort = s_VBS_EVALUATOR ? s_PORT_VBS : -1;
	int treePort = s_TO_SEND_TREE ? s_PORT_SEND_TREE : -1;

	nxnGrid::InitUDP(vbsPort, treePort);
	nxnGrid::InitStaticMembers(s_ONLINE_ALGO, s_PARALLEL_RUN, s_PERIOD_OF_DECISION, s_RESAMPLE_FROM_LAST_OBS);
	Globals::config.time_per_move = s_SEARCH_PRIOD;

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

			if ((!s_IS_ACTION_MOVE_FROM_ENEMY_EXIST) && s_ACTIONS_TYPE == NXN_GLOBAL_ACTIONS)
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
		NXNGrid().run(argc, argv, output, solverType);
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
	objVec[obj].emplace_back(62);
	++obj;

	// insert enemies locations
	std::vector<int> enemy1Loc{ 69 /*99, 98, 89, 88*/};
	objVec[obj] = enemy1Loc;
	++obj;

	//std::vector<int> enemy2loc{ 26, 27, 36, 37 };
	//objVec[obj] = enemy2loc;
	//++obj;

	std::vector<int> nonInv1Loc{ 99/* 55, 56, 65, 66*/ };
	objVec[obj] = nonInv1Loc;
	++obj;

	std::vector<int> shelter1Loc{ 62/*,63,72,73*/ };
	objVec[obj] = shelter1Loc;

}
Attack_Obj CreateEnemy(int x, int y, int gridSize)
{
	int attackRange = gridSize / 4;

	double pHit = 1;
	std::shared_ptr<Attack> attack(new DirectAttack(attackRange, pHit));

	double pStay = 0.4;
	double pTowardSelf = 0.6;
	double pSpawnIfdead = 0;

	Coordinate location(x, y);
	std::shared_ptr<Move_Properties> movement(new TargetDerivedMoveProperties(pStay, pTowardSelf, pSpawnIfdead));

	return Attack_Obj(location, movement, attack);
}

Self_Obj CreateSelf(int x, int y, int gridSize)
{
	int attackRange = gridSize / 4;

	double pHit = 0.4;
	std::shared_ptr<Attack> attack(new DirectAttack(attackRange, pHit));
	double pDistanceFactor = 0.4;
	std::shared_ptr<Observation> obs(new ObservationByDistance(pDistanceFactor));

	Coordinate location(x, y);
	std::shared_ptr<Move_Properties> movement;

	if (s_ACTIONS_TYPE == NXN_GLOBAL_ACTIONS)
	{
		double pMove = 0.75;
		std::shared_ptr<Move_Properties> m(new SimpleMoveProperties(pMove));
		movement = m;
	}
	else if (s_ACTIONS_TYPE == NXN_LOCAL_ACTIONS)
	{
		double pSuccess = 0.9;
		std::shared_ptr<Move_Properties> m(new LowLevelMoveProperties(pSuccess));
		movement = m;
	}

	return Self_Obj(location, movement, attack, obs);
}

Movable_Obj CreateNInv(int x, int y)
{
	double pStay = 0.4;

	Coordinate location(x, y);
	std::shared_ptr<Move_Properties> movement(new NaiveMoveProperties(pStay));

	return Movable_Obj(location, movement);
}

ObjInGrid CreateShelter(int x, int y)
{
	Coordinate location(x, y);

	return ObjInGrid(location);
}
