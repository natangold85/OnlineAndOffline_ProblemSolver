#include <string>
#include <math.h>

#include "../include/despot/OnlineSolverModel.h"

namespace despot 
{

int SumVector(const intVec & vec)
{
	int sum = 0;
	for (auto v : vec)
		sum += v;

	return sum;
}

// model params
std::vector<std::string> OnlineSolverModel::s_ACTIONS_STR;

int OnlineSolverModel::s_periodOfDecision = 1;

bool OnlineSolverModel::s_isOnline = true;
bool OnlineSolverModel::s_parallelRun = true;
bool OnlineSolverModel::s_isExternalSimulator = false;
bool OnlineSolverModel::s_toSendTree = false;
bool OnlineSolverModel::s_treeReuse = false;

UDP_Server OnlineSolverModel::s_udpSimulator;
UDP_Server OnlineSolverModel::s_udpTree;

// for memory allocating
std::mutex OnlineSolverModel::s_memoryMutex;
MemoryPool<OnlineSolverState> OnlineSolverModel::memory_pool_;

/* =============================================================================
* OnlineSolverModel Functions
* =============================================================================*/

bool OnlineSolverModel::InitUDP(int vbsPort, int sendTreePort)
{
	bool stat = true;
	if (vbsPort > 0)
	{
		stat &= s_udpSimulator.Init(vbsPort);
		s_isExternalSimulator = true;
	}
	else
		s_isExternalSimulator = false;

	if (sendTreePort > 0)
	{
		stat &= s_udpTree.Init(sendTreePort);
		char c;
		s_udpTree.Read(&c);
		s_toSendTree = c;
	}
	else
		s_toSendTree = false;

	if (stat)
		std::cout << "required udp connections initialized\n";
	else
		std::cout << "udp connections not initialized correctly\n";

	return stat;
}

void OnlineSolverModel::InitSolverParams(bool isOnline, bool parallelRun, bool treeReuse, int periodOfDecision)
{
	s_isOnline = isOnline;
	s_parallelRun = parallelRun;
	s_periodOfDecision = periodOfDecision;
	s_treeReuse = treeReuse;
}

void OnlineSolverModel::SendEndRun()
{
	unsigned int buf[1];

	buf[0] = END_RUN;
	s_udpTree.Write(reinterpret_cast<char *>(buf), 1 * sizeof (int));
}


void OnlineSolverModel::SendModelDetails2TreeVisualizator(int numActions)
{
	// insert to unsigned int for sending tree compatibility
	unsigned int buf[4];

	buf[0] = START_NEW_TRIAL;
	buf[1] = numActions;
	buf[2] = END_TREE;

	s_udpTree.Write(reinterpret_cast<char *>(buf), 4 * sizeof (int));
}

void OnlineSolverModel::SendTree(State * state, VNode *root)
{
	// create tree
	uintVec buffer;

	buffer.emplace_back(START_NEW_STEP);

	InsertState2Buffer(buffer, state);

	SendTreeRec(root, 0, 1, buffer);

	buffer.emplace_back(END_TREE);

	// send to client
	int sent = 0;
	// to make sure bufsize is divided by sizeof int
	int maxWrite = UDP_Server::s_BUF_LEN - UDP_Server::s_BUF_LEN % sizeof(int);
	while ( sent < (buffer.size()) )
	{
		char *ptr = reinterpret_cast<char *>(&buffer[sent]);
		int leftToWrite = buffer.size() - sent;
		int size = min(leftToWrite * sizeof(int), maxWrite);
		s_udpTree.Write(ptr, size);
		sent += size / sizeof(int);
	}
}

unsigned int OnlineSolverModel::SendTreeRec(VNode *node, unsigned int parentId, unsigned int id, uintVec & buffer)
{
	if (node == nullptr || node->count() == 0)
		return id - 1;

	float val = node->value();
	unsigned int count = node->count();

	buffer.emplace_back(parentId);
	buffer.emplace_back(id);
	buffer.emplace_back(*reinterpret_cast<int *>(&val));
	buffer.emplace_back(count);

	int numch = node->children().size() + 1;
	int nextId = id + node->children().size();
	int actionId = id + 1;
	for (auto action : node->children())
	{
		nextId = SendTreeRec(action, id, actionId, nextId, buffer);
		++actionId;
	}

	return nextId;
}

unsigned int OnlineSolverModel::SendTreeRec(QNode *node, unsigned int parentId, unsigned int id, unsigned int nextAvailableId, uintVec & buffer)
{
	if (node == nullptr) 
		return id; // for action node need to remember id for completion of non initialized nodes

	float val = node->value();
	unsigned int count = node->count();

	buffer.emplace_back(parentId);
	buffer.emplace_back(id);
	buffer.emplace_back(*reinterpret_cast<int *>(&val));
	buffer.emplace_back(count);

	int nextId = nextAvailableId;
	for (auto obs : node->children())
	{
		++nextId;
		nextId = SendTreeRec(obs.second, id, nextId, buffer);
	}

	return nextId;
}


double OnlineSolverModel::ObsProb(OBS_TYPE obs, const State & s, int action) const
{
	return DetailedState::ObsProb(s.state_id, obs);
}



int OnlineSolverModel::FindMaxReward(const doubleVec & rewards, double & expectedReward)
{
	int maxAction = -1;
	double maxReward = REWARD_LOSS - 1;

	for (int a = 0; a < rewards.size(); ++a)
	{
		if (rewards[a] > maxReward)
		{
			maxAction = a;
			maxReward = rewards[a];
		}
	}

	expectedReward = maxReward;
	return maxAction;
}


State * OnlineSolverModel::Allocate(STATE_TYPE state_id, double weight) const
{
	OnlineSolverState* particle = nullptr;
	{
		std::lock_guard<std::mutex> lock(s_memoryMutex);
		particle = memory_pool_.Allocate();
	}

	particle->state_id = state_id;
	particle->weight = weight;
	return particle;
}

State * OnlineSolverModel::Copy(const State * particle) const
{
	OnlineSolverState* new_particle = nullptr;
	{
		std::lock_guard<std::mutex> lock(s_memoryMutex);
		new_particle = memory_pool_.Allocate();
	}

	*new_particle = *static_cast<const OnlineSolverState*>(particle);
	new_particle->SetAllocated();
	return new_particle;
}

void OnlineSolverModel::Free(State * particle) const
{
	std::lock_guard<std::mutex> lock(s_memoryMutex);
	memory_pool_.Free(static_cast<OnlineSolverState*>(particle));
}

int OnlineSolverModel::NumActiveParticles() const
{
	return memory_pool_.num_allocated();
}

void OnlineSolverModel::UpdateRealAction(int & action)
{
	static int counter = 0;
	static int prevAction = 0;

	if (counter % s_periodOfDecision == 0)
		prevAction = action;
	else
		action = prevAction;

	++counter;
}

void OnlineSolverModel::CreateRandomVec(doubleVec & randomVec, int size)
{
	// insert to a vector numbers between 0-1
	for (int i = 0; i < size; ++i)
		randomVec.emplace_back(RandomNum());
}

void OnlineSolverModel::SendAction(int action)
{
	char a = action;
	s_udpSimulator.Write(reinterpret_cast<char *>(&a), sizeof(char));
}

bool OnlineSolverModel::RcvState(State * s, double & reward, OBS_TYPE & obs)
{
	intVec buffer(100);
	int length = s_udpSimulator.Read(reinterpret_cast<char *>(&buffer[0]));

	return RcvStateIMP(buffer, s, reward, obs);
}

void OnlineSolverModel::InitState()
{
	if (ToSendTree())
		SendModelDetails2TreeVisualizator(NumActions());

	// TODO see if necessary in curr implementation

	//if (s_isExternalSimulator)
	//	InitStateExternalSimulator();
	//else
	//	InitStateInternalSimulator();
}

/// init state according to s_objectsInitLocations
void OnlineSolverModel::InitStateInternalSimulator()
{
	// right now only base location is affecting init state
}

/// init state from simulator
void OnlineSolverModel::InitStateExternalSimulator()
{
	// TODO : read init state from external simulator
	//InitStateIMP(state, observation);
}

} //end ns despot