#include "../../include/despot/solver/ParallelSolver.h" 

#include "../../include/despot/solver/pomcp.h" // for tree printing of parallel solver

#include <windows.h> // get currthread

using namespace std;
using namespace despot;


/* =============================================================================
 * ParallelSolver class
 * =============================================================================*/

ParallelSolver::ParallelSolver(std::vector<Solver *> solvers, int numActions)
: solvers_(solvers)
, threadsData_(numActions)
, threads_()
, barrierCounter_(0)
, barrierMutex_()
, mngrData_()
{
	for (int a = 0; a < numActions; ++a)
	{
		std::thread thread([this, a] { this->TreeThreadsMainFunction(a); });
		threads_.emplace_back(std::move(thread));
	}
}



void ParallelSolver::ThreadsMngrFunction()
{
	// start round
	StartRoundMngr();
	bool firstStep = true;
	bool terminal = false;
	while (!terminal)
	{
		// search for action
		int action = Search().action;
		{	//  update action and flags to after action found
			std::lock_guard<std::mutex> lock(mngrData_.m_flagsMutex);
			mngrData_.m_action = action;
			mngrData_.m_observationRecieved = false;
			mngrData_.m_actionNeeded = false;
			mngrData_.m_actionRecieved = true;
			terminal = mngrData_.m_terminal;
		}

		if (terminal)
			break;

		// wait until recieved observation from evaluator
		bool observationRecieved = false;
		while (!observationRecieved)
		{
			std::lock_guard<std::mutex> lock(mngrData_.m_flagsMutex);
			observationRecieved = mngrData_.m_observationRecieved;
			terminal = mngrData_.m_terminal;
		}

		// update observation and action in solvers
		{
			std::lock_guard<std::mutex> lock(mngrData_.m_mainMutex);
			// update if not terminal
			if (!terminal)
				Update(mngrData_.m_action, mngrData_.m_lastObservation);
		}

		{ // inform to builder that round is over
			std::lock_guard<std::mutex> lock(mngrData_.m_flagsMutex);
			terminal = mngrData_.m_terminal;
		}

		firstStep = false;
	}
	EndRoundMngr();
}

void ParallelSolver::TreeThreadsMainFunction(int actionToDevelop)
{
	/// seed tree thread random num
	Random ran((unsigned)time(NULL));
	Random::s_threadSafeRand[GetCurrentThreadId()] = ran;

	TreeDevelopThread * currThreadData = &threadsData_[actionToDevelop];
	
	while (true)
	{
		bool endRound = false;

		{ // init flags
			std::lock_guard<std::mutex> lock(currThreadData->m_treeFlagsMutex);
			currThreadData->m_toDevelop = false;
			currThreadData->m_toUpdate = false;
		}

		// wait until search flag is on and search when its on
		bool waitToPing = true;
		while (waitToPing)
		{
			std::lock_guard<std::mutex> lock(currThreadData->m_treeFlagsMutex);
			waitToPing = !currThreadData->m_toDevelop;
			if (currThreadData->m_terminal)
			{
				endRound = true;
				break;
			}
		}

		if (endRound)
			continue;
		
		solvers_[actionToDevelop]->Search(currThreadData, actionToDevelop);
		QNode * currAction = ((POMCP *)solvers_[actionToDevelop])->root()->Child(actionToDevelop);
		// signal to barrier that search is finished
		{
			std::lock_guard<std::mutex> lock(barrierMutex_);
			++barrierCounter_;
		}

		OBS_TYPE observation;
		int actionToUpdate;

		// wait for mngr to signal to update
		waitToPing = true;
		while (waitToPing)
		{
			std::lock_guard<std::mutex> lock(currThreadData->m_treeFlagsMutex);
			if (currThreadData->m_toUpdate)
			{
				waitToPing = false;
				observation = currThreadData->m_obsToUpdate;
				actionToUpdate = currThreadData->m_actionToUpdate;
			}

			if (currThreadData->m_terminal)
			{
				endRound = true;
				break;
			}
		}

		if (endRound)
			continue;

		solvers_[actionToDevelop]->Update(actionToUpdate, observation);
		{
			std::lock_guard<std::mutex> lock(barrierMutex_);
			++barrierCounter_;
		}
	}
}

ValuedAction ParallelSolver::Search()
{
	// build tree (exit when action is needed)
	for (int a = 0; a < threadsData_.size(); ++a)
	{
		std::lock_guard<std::mutex> lock(threadsData_[a].m_treeFlagsMutex);
		threadsData_[a].m_toDevelop = true;
	}

	// wait for signal to end of search 
	bool actionNeeded = false, terminal = false;
	while (!actionNeeded && !terminal)
	{
		std::lock_guard<std::mutex> lock(mngrData_.m_flagsMutex);
		actionNeeded = mngrData_.m_actionNeeded;
		terminal = mngrData_.m_terminal;
	}

	return FindPrefferedAction();
}

ValuedAction ParallelSolver::FindPrefferedAction()
{
	//pass signal to threads to end search 
	for (int a = 0; a < threadsData_.size(); ++a)
	{
		std::lock_guard<std::mutex> lock(threadsData_[a].m_treeFlagsMutex);
		threadsData_[a].m_toDevelop = false;
	}

	// wait for all thread to finish build tree
	int threadsCounter = 0;
	while (threadsCounter != threads_.size())
	{
		std::lock_guard<std::mutex> lock(barrierMutex_);
		threadsCounter = barrierCounter_;
	}

	// find max reward action
	float maxValue = Globals::NEG_INFTY;
	int prefferredAction = -1;
	for (int a = 0; a < threadsData_.size(); ++a)
	{
		if (threadsData_[a].m_value > maxValue)
		{
			maxValue = threadsData_[a].m_value;
			prefferredAction = a;
		}
	}

	{
		std::lock_guard<std::mutex> lock(barrierMutex_);
		barrierCounter_ = 0;
	}

	return ValuedAction(prefferredAction, maxValue);
}

void ParallelSolver::Update(int action, OBS_TYPE obs)
{
	for (int a = 0; a < threadsData_.size(); ++a)
	{
		std::lock_guard<std::mutex> lock(threadsData_[a].m_treeFlagsMutex);
		threadsData_[a].m_actionToUpdate = action;
		threadsData_[a].m_obsToUpdate = obs;
		threadsData_[a].m_toUpdate = true;
	}

	// wait for all thread to finish update
	int threadsCounter = 0;
	while (threadsCounter != threads_.size())
	{
		std::lock_guard<std::mutex> lock(barrierMutex_);
		threadsCounter = barrierCounter_;
	}

	std::lock_guard<std::mutex> lock(barrierMutex_);
	barrierCounter_ = 0;
}

void ParallelSolver::StartRoundMngr()
{
	/// seed mngr thread random num
	Random ran((unsigned)time(NULL));
	Random::s_threadSafeRand[GetCurrentThreadId()] = ran;

	// make sure that terminal flag is off
	for (int a = 0; a < threadsData_.size(); ++a)
	{
		std::lock_guard<std::mutex> lock(threadsData_[a].m_treeFlagsMutex);
		threadsData_[a].m_terminal = false;
	}

	std::lock_guard<std::mutex> lock(mngrData_.m_flagsMutex);
	// reset params before exiting
	mngrData_.m_action = -1;
	mngrData_.m_lastObservation = -1;
	mngrData_.m_actionNeeded = false;
	mngrData_.m_actionRecieved = false;
	mngrData_.m_terminal = false;
	mngrData_.m_observationRecieved = false;
}
void ParallelSolver::EndRoundMngr()
{
	// inform to tree threads that round is over
	for (int a = 0; a < threadsData_.size(); ++a)
	{
		std::lock_guard<std::mutex> lock(threadsData_[a].m_treeFlagsMutex);
		threadsData_[a].m_terminal = true;
	}
}

void ParallelSolver::UpdateHistory(int action, OBS_TYPE obs)
{
	//TODO think about funcion implementation
}

void ParallelSolver::belief(Belief* b)
{
	// should not get to this function
	assert(false);
}

void ParallelSolver::belief(Belief* b, int idxSolver)
{
	solvers_[idxSolver]->belief(b);
}

Belief* ParallelSolver::belief()
{
	return solvers_[0]->belief();
}

void ParallelSolver::DeleteBelief()
{
	for (int sol = 0; sol < solvers_.size(); ++sol)
	{
		Belief* b = solvers_[sol]->belief();
		delete b;
	}
}


void ParallelSolver::GetTreeProperties(Tree_Properties & treeProp) const
{
	int numChildren = solvers_.size();
	treeProp.m_actionsChildren.resize(numChildren);

	for (int sol = 0; sol < numChildren; ++sol)
	{
		solvers_[sol]->GetSingleActionTreeProp(treeProp.m_actionsChildren[sol], sol);
	}

	int treeSize = 0;
	int treeCount = 0;
	float value = Globals::NEG_INFTY;
	int maxHeight = 0;
	for (int sol = 0; sol < numChildren; ++sol)
	{
		treeSize += treeProp.m_actionsChildren[sol].m_size;
		treeCount += treeProp.m_actionsChildren[sol].m_nodeCount;
		value = max(value, treeProp.m_actionsChildren[sol].m_nodeValue);
		maxHeight = max(maxHeight, treeProp.m_actionsChildren[sol].m_height);
	}

	treeProp.m_rootTreeProp.m_nodeCount = treeCount;
	treeProp.m_rootTreeProp.m_size = treeSize;
	treeProp.m_rootTreeProp.m_nodeValue = value;
	treeProp.m_rootTreeProp.m_height = maxHeight + 1;
}
