#include <fstream>      // std::ofstream

/// tui class
#include <simple_tui.h>


/// models available
#include "Sc2QTableHallucinate.h"

// properties of objects
#include "Coordinate.h"

using namespace despot;

// tables names
static std::string s_Q_TABLE_NAME = "melee_attack_qtable_naiveExploration.txt";
static std::string s_T_TABLE_NAME = "transitionTableExample.txt";
static std::string s_R_TABLE_NAME = "melee_attack_rtable_naiveExploration.txt";

// parameters of model:
static const bool s_ONLINE_ALGO = true;
static const bool s_TREE_REUSE = true;
static const bool s_EXTERNAL_SIMULATOR = false;
static const bool s_TO_SEND_TREE = false;

// solver type :
static const std::string solverType = "user"; //"POMCP";  //"Parallel_POMCP"; // "user"
static const std::string beliefType = "FullyObserved"; //"DEFAULT";  //"FullyObserved";
static const bool s_PARALLEL_RUN = solverType == "Parallel_POMCP";

// solver params
static const int MINIMUM_STATE_VISITS_TO_SIMULATE = 0;
static const int s_PERIOD_OF_DECISION = 1; // sending action not in every decision
static const int s_SEARCH_PRIOD = 1;
static const int s_PORT_SEND_TREE = 5678;
static const int s_PORT_VBS = 5432;

// model params
static const int s_ONLINE_GRID_SIZE = 4;
static const int s_BASE_LOCATION = 0;






void ErrorAndExit(char *errorMsg);
void ReadQTable(OnlineSolverModel::OnlineSolverLUT & lutForModel);
void Run(int argc, char* argv[], std::string & outputFName, int numRuns);

/// solve nxn Grid problem
class SC2SOLVER : public SimpleTUI {
public:
	SC2SOLVER() {}

	DSPOMDP* InitializeModel(option::Option* options) override
	{
		// create problem

		SC2Agent *model = new SC2Agent();
		return model;
	}

	void InitializeDefaultParameters() override {}
};

int main(int argc, char* argv[]) 
{
	int numRuns = 20;
	
	/// seed main thread random num
	Random randStream((unsigned)time(NULL));
	Random::s_threadSafeRand[GetCurrentThreadId()] = randStream;

	int vbsPort = s_EXTERNAL_SIMULATOR ? s_PORT_VBS : -1;
	int treePort = s_TO_SEND_TREE ? s_PORT_SEND_TREE : -1;

	SC2Agent::InitUDP(vbsPort, treePort);
	SC2Agent::InitSolverParams(s_ONLINE_ALGO, s_PARALLEL_RUN, s_PERIOD_OF_DECISION);
	Globals::config.time_per_move = s_SEARCH_PRIOD;

	bool stat = SC2Agent::InitTables(s_Q_TABLE_NAME, s_T_TABLE_NAME, s_R_TABLE_NAME, MINIMUM_STATE_VISITS_TO_SIMULATE);
	if (!stat)
		ErrorAndExit("read tables");


	// create output file
	std::string outputFNameNaive("test");

	outputFNameNaive.append("_result.txt");
	Run(argc, argv, outputFNameNaive, numRuns);

	char c;
	std::cout << "result written succesfully. press any key to exit\n";
	std::cin >> c;
	return 0;
}

void ErrorAndExit(char *errorMsg)
{
	std::cout << "Error!!! error in " << errorMsg << "\npress any key to exit...\n";
	char c;
	std::cin >> c;
	exit(1);
}
void Run(int argc, char* argv[], std::string & outputFName, int numRuns)
{
	remove(outputFName.c_str());
	std::ofstream output(outputFName.c_str(), std::ios::out);
	if (output.fail())
		ErrorAndExit("open output file");

	// run model numRuns times
	output << "results for naive online. num runs = " << numRuns << "\n";
	for (size_t i = 0; i < numRuns; i++)
	{
		std::cout << "\n\n\trun #" << i << ":\n";
		output << "\n\n\trun #" << i << ":\n";
		SC2SOLVER().run(argc, argv, output, solverType, beliefType);
		output.flush();
	}

	if (output.fail())
	{
		ErrorAndExit("write output file");
	}
}

void ReadQTable(OnlineSolverModel::OnlineSolverLUT & lutForModel)
{
	//bool isDoubles = false;
	//while (!qTable.eof())
	//{
	//	// read state
	//	SC2DetailedState currState;
	//	
	//	// read counts
	//	for (int c = 0; c < SC2DetailedState::NUM_COUNTS; ++c)
	//		qTable >> currState[c];

	//	// read hotspots

	//	int idx = 0;
	//	qTable >> txt;
	//	//while (txt != "...")
	//	//{
	//	//	int hotspotVal = atoi(txt.c_str());
	//	//	hotspotVal = min(1, hotspotVal);
	//	//	miniMap[idx] = (PIXEL)hotspotVal;
	//	//	++idx;
	//	//	qTable >> txt;
	//	//}

	//	//// rest of idx that not been sent will be set to 0 in the mean time
	//	//for (; idx < miniMap.size(); ++idx)
	//	//	miniMap[idx] = FREE;
	//	//// insert base location
	//	//miniMap[SC2DetailedState::s_baseLocation] = BLUE;

	//	doubleVec rewardVec(SC2Agent::NUM_ACTIONS);
	//	idx = 0;
	//	// read values
	//	qTable >> txt;
	//	while (txt != "[" && !qTable.eof())
	//	{
	//		if (idx < rewardVec.size())
	//			rewardVec[idx] = atof(txt.c_str());

	//		++idx;
	//		qTable >> txt;
	//	}
	//	
	//	auto itr = lutForModel.find(currState.GetStateId());	
	//	if (itr == lutForModel.end())
	//		lutForModel[currState.GetStateId()] = rewardVec;
	//	else
	//		isDoubles = true;
	//}

	//if (isDoubles)
	//	std::cout << "\n\n\nThere is duplicate states in Q Table !!!!\n\n\n";
}