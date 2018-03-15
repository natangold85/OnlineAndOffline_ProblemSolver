#include <fstream>      // std::ofstream

/// tui class
#include <simple_tui.h>


/// models available
#include "SC2BasicAgent.h"

// properties of objects
#include "Coordinate.h"

using namespace despot;

// parameters of model:
static const bool s_ONLINE_ALGO = true;
static const bool s_TREE_REUSE = true;
static const bool s_EXTERNAL_SIMULATOR = true;
static const bool s_TO_SEND_TREE = false;

// solver type :
static const std::string solverType = "POMCP"; //"POMCP";  //"Parallel_POMCP"; // "user"
static const std::string beliefType = "SC2AgentBelief"; //"DEFAULT";  //"SC2AgentBelief";
static const bool s_PARALLEL_RUN = solverType == "Parallel_POMCP";


static const int s_ONLINE_GRID_SIZE = 4;
static const int s_PERIOD_OF_DECISION = 1; // sending action not in every decision
static const int s_SEARCH_PRIOD = 1;
static const int s_PORT_SEND_TREE = 5678;
static const int s_PORT_VBS = 5432;

void Run(int argc, char* argv[], std::string & outputFName, int numRuns);

/// solve nxn Grid problem
class SC2SOLVER : public SimpleTUI {
public:
	SC2SOLVER() {}

	DSPOMDP* InitializeModel(option::Option* options) override
	{
		// create nxnGrid problem

		// init static members
		SC2DetailedState::InitStatic();

		int baseLoc = 0;
		int enemyBaseLoc = s_ONLINE_GRID_SIZE * s_ONLINE_GRID_SIZE - 1;

		SC2BasicAgent *model = new SC2BasicAgent(s_ONLINE_GRID_SIZE, baseLoc, enemyBaseLoc);
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

	SC2BasicAgent::InitUDP(vbsPort, treePort);
	SC2BasicAgent::InitSolverParams(s_ONLINE_ALGO, s_PARALLEL_RUN, s_PERIOD_OF_DECISION);
	Globals::config.time_per_move = s_SEARCH_PRIOD;

	// create output file
	std::string outputFNameNaive("Naive");
	outputFNameNaive.append("_result.txt");
	Run(argc, argv, outputFNameNaive, numRuns);

	char c;
	std::cout << "result written succesfully. press any key to exit\n";
	std::cin >> c;
	return 0;
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
		SC2SOLVER().run(argc, argv, output, solverType, beliefType);
		output.flush();
	}

	if (output.fail())
	{
		std::cerr << "error in write output file";
		exit(1);
	}
}