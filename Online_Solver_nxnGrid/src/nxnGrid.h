#ifndef NXNGRID_H
#define NXNGRID_H

#include <string>

#include "..\include\despot\solver\pomcp.h"

#include <UDP_Prot.h>

#include "Self_Obj.h"
#include "Attack_Obj.h"
#include "Movable_Obj.h"
#include "ObjInGrid.h"

namespace despot 
{

using intVec = std::vector<int>;

/* =============================================================================
* NxNState class
* =============================================================================*/
/// the class is non-thread safe. 
/// the static members are specialized to one type of nxnGrid, so only one problem simulation can simultanuasely
class nxnGridState : public State 
{

public:
	nxnGridState() = default;
	nxnGridState(STATE_TYPE state_id, double weight = 0.0);
	
	/// print the state
	std::string text() const override;
};

// translation between stateId to logical repressentation of state
class DetailedState
{
public:

	DetailedState();
	explicit DetailedState(STATE_TYPE state_id);
	DetailedState(const State & state);
	explicit DetailedState(unsigned int sizeState);

	/// init static members
	static void InitStatic();

	// Id to state interface functions
	static void InitLocationsFromId(STATE_TYPE state_id, intVec & locations);
	static int GetObjLocation(STATE_TYPE state_id, int objIdx);
	static int GetObservedObjLocation(OBS_TYPE state_id, int objIdx);

	STATE_TYPE GetStateId() const;
	static STATE_TYPE MaxState();

	// location vec functions
	unsigned int size() const { return m_locations.size(); };

	intVec & LocationVec() { return m_locations; };
	const intVec & LocationVec() const { return m_locations; };

	int &operator[](int idx) { return m_locations[idx]; };
	int operator[](int idx) const { return m_locations[idx]; };
	
	static double ObsProb(STATE_TYPE state_id, OBS_TYPE obs, int gridSize, const Observation & obsType);
	static double ObsProbOneObj(STATE_TYPE state_id, OBS_TYPE obs, int objIdx, int gridSize, const Observation & obsType);
	
	// print state functions

	/// print the state
	std::string text() const;
	static void PrintGrid(const intVec & locations, std::string & buffer);
	/// return char identified location
	static char ObjIdentity(const intVec & locations, int location);

	
	// object related functions

	void EraseNonInv();
	void EraseObject(int objectIdx);

	bool NoEnemies(int gridSize) const;
	bool IsNonInvDead(int gridSize) const;

	/*MEMBERS*/
	intVec m_locations;

	/*STATIC MEMBERS*/

	/// bits for location
	static const int s_NUM_BITS_LOCATION = 8;
	
	/// shelter vector
	static intVec s_shelters;

	/// number of objects in grid (size of state vector)
	static int s_numNonInvolved;
	/// the border between the enemies location and the non-involved location in the state vector
	static int s_numEnemies;
	/// gridSize
	static int s_gridSize;
	/// target location
	static int s_targetLoc;
};



/* =============================================================================
* nxnGrid class
* =============================================================================*/
/// base class for nxnGrid model. derived classes are including step and actions implementations
/// the class is non thread safe. 
/// the static members are specialized to one type of nxnGrid so only one problem can simultaneously run
class nxnGrid : public DSPOMDP
{	
public:
	using intVec = std::vector<int>;
	using doubleVec = std::vector<double>;
	using lut_t = std::map < STATE_TYPE, doubleVec >;
	///	enum of objects
	enum OBJECT { SELF, ENEMY, NON_INV, SHELTER, TARGET, NUM_OBJECTS };
	enum VBS_OBJECTS { SELF_VBS, ENEMY_VBS, NON_INVOLVED_VBS, SHELTER_VBS, TARGET_VBS, OBSERVED_ENEMY_VBS, OBSERVED_NON_INVOLVED_VBS };
	/// enum of type of calculation using sarsop data map
	enum CALCULATION_TYPE { WITHOUT, ALL, WO_NINV, JUST_ENEMY, ONE_ENEMY, WO_NINV_STUPID };
	 
	explicit nxnGrid(int gridSize, int traget, Self_Obj & self, std::vector<intVec> & objectsInitLoc);
	~nxnGrid() = default;
	
	// Functions for self use

	// model static details
	static bool IsOnline() { return s_isOnline; };
	static bool ParallelRun() { return s_parallelRun; };
	static bool IsResampleLastObs() { return s_resampleFromLastObs; };
	static bool IsExternalSimulator() { return s_isExternalSimulator; };
	static bool ToSendTree() { return s_toSendTree; };

	/// Init functions 

	/// init udp server
	static bool InitUDP(int portNum = -1, int sendTreePort = -1);
	/// init offline decision lut
	static void InitLUT(lut_t & offlineLut, int offlineGridSize, CALCULATION_TYPE cType = WITHOUT);
	/// init model type, periodOfdecision
	static void InitStaticMembers(bool isOnline, bool parallelRun, int periodOfDecision = 1, bool resampleFromLastObs = true);

	/// Functions for creating of model

	/// add enemy object
	void AddObj(Attack_Obj&& obj); 
	/// add non involved object
	void AddObj(Movable_Obj&& obj);
	/// add shelter
	void AddObj(ObjInGrid&& obj);

	
	/// update real action given action (if we want to make decision once in every several algorithm decisions)
	static void UpdateRealAction(int & action);

	/// send end of trial signal
	static void SendEndRun();
	/// send num actions to visualizator
	static void SendModelDetails(int numActions, int numObj);
	/// send tree in udp for visualisation
	static void SendTree(State * state, VNode *root);


	/// count number of moving objects(self, enemies, non involved, shelters)
	int CountAllObjects() const;
	/// count number of moving objects(self, enemies, non involved)
	unsigned int CountMovingObjects() const;

	/// return grid size
	int GetGridSize() const { return m_gridSize; };

	/// Functions for online solver improvement

	/// return vector of rewards given model and prior
	static void ChoosePreferredAction(POMCPPrior * prior, const DSPOMDP* m, doubleVec & expectedRewards);
	/// return a preferred action given model and prior
	static int ChoosePreferredAction(POMCPPrior * prior, const DSPOMDP* m, double & expectedReward);

	/// specific resample ccording to model
	static std::vector<State*> ResampleLastObs(int num, const std::vector<State*>& belief, const DSPOMDP* model, History history);

	/// initialize beliefState according to history
	void InitBeliefState(DetailedState & beliefState, const History & h) const;
	
	/// recieve init state from simulator
	void InitState();
	/// send action to simulator
	void SendAction(int action);
	/// recieve current state from simulator and update rewrd and observation
	bool RcvState(State * s, OBS_TYPE lastObs, double & reward, OBS_TYPE & obs);

	// Functions for despot algorithm (for more information read the document "Tutorial on Using DESPOT with cpp model")

	/// return the probability for an observation given a state and an action
	virtual double ObsProb(OBS_TYPE obs, const State& state, int action) const override;
	/// return the probability for an observation given a state and an action
	double ObsProbOneObj(OBS_TYPE obs, const State& state, int action, int objIdx) const;

	// create particle vector based on possible objects location and its weight
	void CreateParticleVec(std::vector<std::vector<std::pair<int, double> > > & objLocations, std::vector<State*> & particles) const;

	/// return initial state
	virtual State *CreateStartState(std::string type) const override;
	///  return initial belief
	virtual Belief* InitialBelief(const State* start, std::string type) const override;
	
	/// initializ and allocate memory for a state
	virtual State* Allocate(STATE_TYPE state_id, double weight) const override;
	/// alocate memory and copy a state
	virtual State* Copy(const State* particle) const override;
	virtual void Free(State* particle) const override;
	virtual int NumActiveParticles() const override;

	/// return the max reward available
	virtual double GetMaxReward() const override{ return REWARD_WIN; };

	virtual void DisplayParameters(std::ofstream & out) const override;

	virtual void PrintState(const State& state, std::ostream& out = std::cout) const override;
	virtual void PrintBelief(const Belief& belief, std::ostream& out = std::cout) const override;
	virtual void PrintObs(const State& state, OBS_TYPE obs, std::ostream& out = std::cout) const override;
	virtual void PrintAction(int action, std::ostream & out) const override;

/// functions that are necessary for step and action calculation
protected:
	/// check if 2 idx are in a given range on the grid
	static bool InRange(int idx1, int idx2, double range, int gridSize);

	/// return true if the robot is dead by enemy attack given random num(0-1). state is not reference by reason
	bool CalcIfKilledByEnemy(const DetailedState & state, int enemyIdx, double & randomNum) const;

	/// return identity of the objIdx
	enum OBJECT WhoAmI(int objIdx) const;

	/// create a vector of random numbers between 0 - 1
	static void CreateRandomVec(doubleVec & randomVec, int size);

	/// retrieve the observed state given current state and random number
	OBS_TYPE FindObservation(const DetailedState & state, double p) const;

	/// advance the state to the next step position (regarding to other objects movement)
	void SetNextPosition(DetailedState & state, doubleVec & randomNum) const;

	/// change object location according to its movement properties and random number
	void CalcMovement(DetailedState & state, const Movable_Obj *object, double rand, int objIdx) const;

	intVec GetSheltersVec() const;

private:
	///// fill values of subtrees
	static unsigned int SendTreeRec(VNode *node, unsigned int parentId, unsigned int id, std::vector<unsigned int> & buffer);
	static unsigned int SendTreeRec(QNode *node, unsigned int parentId, unsigned int id, unsigned int nextAvailableId, std::vector<unsigned int> & buffer);

	/// implementation of choose prefferred action
	void ChoosePreferredActionIMP(const DetailedState & beliefState, doubleVec & expectedReward) const;

	static int FindMaxReward(const doubleVec & rewards, double & expectedReward);
	/// create particle from available locations
	void CreateParticleVecRec(DetailedState & state, std::vector<std::vector<std::pair<int, double> > > & objLocations, std::vector<State*> & particles, double weight, int currIdx) const;

	/// create vector of all possible particles for init state
	void InitialBeliefStateRec(DetailedState & state, int currObj, double stateProb, std::vector<State*> & particles) const;

	/// find the observed state according to random number and original state
	void DecreasePObsRec(const DetailedState & originalState, DetailedState & currState, int currIdx, double pToDecrease, double &pLeft) const;

	// Rescaling state functions:
	void ScaleState(const DetailedState & beliefState, DetailedState & scaledState) const;
	void ScaleState(const DetailedState & beliefState, DetailedState & scaledState, int newGridSize, int prevGridSize) const;

	/// initialize rewards vector of 2 enemies from 2 vectors of rewards vec of 1 enemy
	void Combine2EnemiesRewards(const intVec & beliefState, const doubleVec & rewards1E, const doubleVec & rewards2E, doubleVec & rewards) const;


	/// move non protected shelters to close non-object location
	void MoveNonProtectedShelters(const intVec & baliefState, intVec & scaledState, int oldGridSize, int newGridSize) const;
	/// move object location to the closest scaled spot but current scaled location
	void MoveObjectLocation(const intVec & beliefState, intVec & scaledState, int objIdx, int oldGridSize, int newGridSize) const;
	/// in case self is on target after scaling we need to shift it from target
	void ShiftSelfFromTarget(const intVec & beliefState, intVec & scaledState, int oldGridSize, int newGridSize) const;
	/// drop shelter from state (make shelter in accessible)
	void DropNonProtectedShelter(intVec & woShelter, int oldGridSize, int newGridSize) const;

	/// return num enemies in calculation
	int NumEnemiesInCalc() const;
	/// return true if the lut is without non-involved
	int NumNonInvInCalc() const;


	/// init state according to s_objectsInitLocations
	void InitStateRandom();
	/// init state from vbs simulator
	void InitStateSimulator();
	/// recieve from vbs smulator current state
	void InitStateIMP(DetailedState & state, DetailedState & observation);

	/// return the object location considering object type and object num(idx)
	int FindObject(intVec & state, intVec & identity, int object, int idx);
	int FindObject(intVec & state, intVec & identity, int object, int observedObj, bool & isObserved, int idx);

	/// add actions to specific enemy
	virtual void AddActionsToEnemy() = 0;
	virtual void AddActionsToShelter() = 0;

	/// return idx of enemy action is related to
	virtual int EnemyRelatedActionIdx(int action) const = 0;
	/// return true if action is related to enemy
	virtual bool EnemyRelatedAction(int action) const = 0;

	/// return random legal action given last observation
	virtual bool LegalAction(const DetailedState & observedState, int action) const = 0;

protected:
	int m_gridSize;
	int m_targetIdx;

	Self_Obj m_self;
	std::vector<Attack_Obj> m_enemyVec;
	std::vector<Movable_Obj> m_nonInvolvedVec;
	
	std::vector<ObjInGrid> m_shelters;

	// init locations of objects (needs to be in the size of number of objects - including self)
	static std::vector<intVec> s_objectsInitLocations;

	// lut for actions names
	static std::vector<std::string> s_actionsStr;

	/// offline data LUT
	static lut_t s_LUT;
	static int s_lutGridSize;

	// for model
	mutable MemoryPool<nxnGridState> memory_pool_;

	/// for comunication with VBS
	static UDP_Server s_udpVBS;
	/// for sending tree
	static UDP_Server s_udpTree;
public:
	/// type of model
	static bool s_isOnline;
	static bool s_parallelRun;
	static bool s_resampleFromLastObs;
	static bool s_isExternalSimulator;
	static bool s_toSendTree;

	static enum CALCULATION_TYPE s_calculationType;
	static int s_periodOfDecision;

	/// rewards for different events
	static const double REWARD_MIN;
	static const double REWARD_WIN;
	static const double REWARD_LOSS;
	static const double REWARD_KILL_ENEMY;
	static const double REWARD_KILL_NINV;
	static const double REWARD_ILLEGAL_MOVE;
	static const double REWARD_STEP;
	static const double REWARD_FIRE;

};

} // end ns despot

#endif	// NXNGRID_H