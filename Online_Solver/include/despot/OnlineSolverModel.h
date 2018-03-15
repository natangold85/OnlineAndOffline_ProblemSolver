#ifndef ONLINE_SOLVER_MODEL_H
#define ONLINE_SOLVER_MODEL_H

#include <string>

#include <pomcp.h>

#include <UDP_Prot.h>

namespace despot 
{

using intVec = std::vector<int>;
using uintVec = std::vector<unsigned int>;
using doubleVec = std::vector<double>;

/* =============================================================================
* State class
* =============================================================================*/
class OnlineSolverState : public State 
{

public:
	OnlineSolverState() = default;
	OnlineSolverState(STATE_TYPE state_id, double weight = 0.0) : State(state_id, weight){}
};

// translation between stateId to logical repressentation of state
class DetailedState
{
public:
	static double ObsProb(STATE_TYPE state_id, OBS_TYPE obs);
	
	/*STATIC MEMBERS*/
	static const STATE_TYPE s_ONE = 1;
};


/* =============================================================================
* abstract class for onnline solver
* =============================================================================*/
class OnlineSolverModel : public DSPOMDP
{	
public:
	using OnlineSolverLUT = std::map < STATE_TYPE, doubleVec >;
	// prams for tree sending
	enum TREE_SEND_PARAMS { END_TREE, START_NEW_TRIAL, START_NEW_STEP, END_RUN };

	OnlineSolverModel() = default;
	~OnlineSolverModel() = default;

	/*MODEL SOLVER DETAILS*/
	static bool IsOnline() { return s_isOnline; };
	static bool TreeReuse() { return s_treeReuse; };
	static bool ParallelRun() { return s_parallelRun; };
	static bool IsExternalSimulator() { return s_isExternalSimulator; };
	static bool ToSendTree() { return s_toSendTree; };


	/*INIT FUNCTIONS*/ 

	/// init udp server
	static bool InitUDP(int portNum = -1, int sendTreePort = -1);
	/// init model type, periodOfdecision
	static void InitSolverParams(bool isOnline, bool parallelRun, bool treeReuse, int periodOfDecision = 1);

	/*EXTERNAL SIMULATOR FUNCTIONS*/

	/// send action to simulator
	void SendAction(int action);


	/*TREE VISUALISATOR FUNCTIONS*/

	/// send end of trial signal
	static void SendEndRun();
	/// send num actions to tree visualsiator
	static void SendModelDetails2TreeVisualizator(int numActions);
	/// send tree in udp for tree visualisation
	static void SendTree(State * state, VNode *root);


	/*SOLVER FUNCTIONS*/

	/// return the probability for an observation given a state and an action
	virtual double ObsProb(OBS_TYPE obs, const State& state, int action) const override;
	
	/// initializ and allocate memory for a state
	virtual State* Allocate(STATE_TYPE state_id, double weight) const override;
	/// alocate memory and copy a state
	virtual State* Copy(const State* particle) const override;
	virtual void Free(State* particle) const override;
	virtual int NumActiveParticles() const override;

	/// return the max reward available
	virtual double GetMaxReward() const override = 0;

	/// update real action given action (if we want to make decision once in every several algorithm decisions)
	static void UpdateRealAction(int & action);

	/*STEP AND ACTION CALC*/
protected:

	inline static double RandomNum() { return Random::s_threadSafeRand[GetCurrentThreadId()].NextDouble(); }
	/// create a vector of random numbers between 0 - 1
	static void CreateRandomVec(doubleVec & randomVec, int size);

private:

	/*TREE VISU FUNCS*/
	///// fill values of subtrees
	static unsigned int SendTreeRec(VNode *node, unsigned int parentId, unsigned int id, uintVec & buffer);
	static unsigned int SendTreeRec(QNode *node, unsigned int parentId, unsigned int id, unsigned int nextAvailableId, uintVec & buffer);

	/*SOLVER FUNCS*/
	
	static int FindMaxReward(const doubleVec & rewards, double & expectedReward);

	/// init state from vbs simulator
	void InitStateExternalSimulator();
	/// init state according to s_objectsInitLocations
	void InitStateInternalSimulator();

	/*FUNCTIONS FOR BASE CLASS TO IMPLEMENT*/
public:

	/// return vector of rewards given model and prior
	static void ChoosePreferredAction(POMCPPrior * prior, const DSPOMDP* m, doubleVec & expectedRewards);
	/// return a preferred action given model and prior
	static int ChoosePreferredAction(POMCPPrior * prior, const DSPOMDP* m, double & expectedReward);

	static void InsertState2Buffer(uintVec & buffer, State * state);

	/// return initial state
	virtual State *CreateStartState(std::string type) const override = 0;
	///  return initial belief
	virtual Belief* InitialBelief(const State* start, std::string type) const override = 0;

	/// recieve init state from simulator
	virtual void InitState();
	/// recieve current state from simulator and update reward and observation
	bool RcvState(State * s, double & reward, OBS_TYPE & obs);

	/// display model params to out
	virtual void DisplayParameters(std::ofstream & out) const override = 0;

	/*PRINT FUNCTIONS*/

	virtual void PrintState(const State& state, std::ostream& out = std::cout) const override = 0;
	virtual void PrintBelief(const Belief& belief, std::ostream& out = std::cout) const override = 0;
	virtual void PrintObs(const State& state, OBS_TYPE obs, std::ostream& out = std::cout) const override = 0;
	virtual void PrintAction(int action, std::ostream & out) const override = 0;

protected:

	virtual bool RcvStateIMP(intVec & buffer, State * s, double & reward, OBS_TYPE & obs) const = 0;

	/// return random legal action given last observation
	virtual bool LegalAction(OBS_TYPE observation, int action) const = 0;

protected:
	// lut for actions names (initialized by base class)
	static std::vector<std::string> s_ACTIONS_STR;

	// for memory allocating
	static std::mutex s_memoryMutex;
	static MemoryPool<OnlineSolverState> memory_pool_;

	/// for comunication with VBS
	static UDP_Server s_udpSimulator;
	/// for sending tree
	static UDP_Server s_udpTree;

public:

	/// type of model
	static bool s_isOnline;
	static bool s_parallelRun;
	static bool s_isExternalSimulator;
	static bool s_treeReuse;
	static bool s_toSendTree;

	static int s_periodOfDecision;

	static const int REWARD_WIN;
	static const int REWARD_LOSS;
};


} // end ns despot

#endif	// ONLINE_SOLVER_MODEL_H