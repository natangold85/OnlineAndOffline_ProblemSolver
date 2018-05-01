#ifndef NXNGRID_H
#define NXNGRID_H

#include <string>

#include <OnlineSolverModel.h>

#include "Self_Obj.h"
#include "Attack_Obj.h"
#include "Movable_Obj.h"
#include "ObjInGrid.h"

namespace despot 
{

// TODO : need 2 find solution for last observation using this model (after inserting solverto lib)
// translation between stateId to logical repressentation of state
class nxnGridDetailedState : public DetailedState
{
public:

	nxnGridDetailedState();
	explicit nxnGridDetailedState(STATE_TYPE state_id);
	nxnGridDetailedState(const State & state);
	explicit nxnGridDetailedState(unsigned int sizeState);

	/// init static members
	static void InitStatic();

	// Id to state interface functions
	static void InitLocationsFromId(STATE_TYPE & state_id, intVec & locations);
	static void InitIsObservedBoolFromId(STATE_TYPE & state_id, boolVec & locations);

	static int GetObjLocation(STATE_TYPE state_id, int objIdx);
	static bool IsEnemyObserved(OBS_TYPE state_id, int enemyIdx);
	static void GetEnemyObservedVec(OBS_TYPE state_id, boolVec & enemyObsVec);

	STATE_TYPE GetStateId() const;
	OBS_TYPE GetObsId() const;
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
	void PrintGrid(std::string & buffer) const;
	/// return char identified location
	char ObjIdentity(int location) const;

	// object related functions
	bool IsEnemyObserved(int enemyIdx) const { return m_isEnemyObserved[enemyIdx]; };
	void IsEnemyObserved(int enemyIdx, bool isObs) { m_isEnemyObserved[enemyIdx] = isObs; };

	void EraseNonInv();
	void EraseObject(int objectIdx);
	
	bool IsProtected(int objIdx) const;

	bool NoEnemies(int gridSize) const;
	bool IsNonInvDead(int gridSize) const;

	//bool NonValidState() const;
	/*MEMBERS*/
	intVec m_locations;
	boolVec m_isEnemyObserved;

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
class nxnGrid : public OnlineSolverModel
{	
public:
	///	enum of objects
	enum OBJECT { SELF, ENEMY, NON_INV, SHELTER, TARGET, NUM_OBJECTS };

	/// enum of type of calculation using sarsop data map
	enum CALCULATION_TYPE { WITHOUT, ALL, WO_NINV, JUST_ENEMY, ONE_ENEMY, WO_NINV_STUPID };
	 
	explicit nxnGrid(int gridSize, int traget, Self_Obj & self, std::vector<intVec> & objectsInitLoc);
	~nxnGrid() = default;
	
	/// init offline decision lut
	static void InitLUT(OnlineSolverLUT & offlineLut, int offlineGridSize, CALCULATION_TYPE cType = WITHOUT);

	int ChoosePreferredAction(POMCPPrior * prior, double & expectedReward) const;
	void ChoosePreferredAction(POMCPPrior * prior, doubleVec & expectedRewards) const;

	/// Functions for creating of model

	/// add enemy object
	void AddObj(Attack_Obj&& obj); 
	/// add non involved object
	void AddObj(Movable_Obj&& obj);
	/// add shelter
	void AddObj(ObjInGrid&& obj);

	/// count number of moving objects(self, enemies, non involved, shelters)
	int CountAllObjects() const;

	/// count number of moving objects(self, enemies, non involved)
	unsigned int CountMovingObjects() const;

	/// return grid size
	int GetGridSize() const { return m_gridSize; };

	/// initialize beliefState according to history
	void InitBeliefState(nxnGridDetailedState & beliefState, const History & h) const;
	
	/// recieve init state from simulator
	virtual void InitState() override;

	virtual double ObsProb(OBS_TYPE obs, const State& state, int action) const override;

	/// return the probability for an observation given a state and an action
	double ObsProbOneObj(OBS_TYPE obs, const State& state, int action, int objIdx) const;

	/// return initial state
	virtual State *CreateStartState(std::string type) const override;
	///  return initial belief
	virtual Belief* InitialBelief(const State* start, std::string type) const override;

	// create particle vector based on possible objects location and its weight (for resample)
	void CreateParticleVec(std::vector<std::vector<std::pair<int, double> > > & objLocations, std::vector<State*> & particles) const;

	/// implementation of choose prefferred action
	void ChoosePreferredActionIMP(const nxnGridDetailedState & beliefState, doubleVec & expectedReward) const;

	/// return the max reward available
	virtual double GetMaxReward() const override{ return REWARD_WIN; };

	virtual void DisplayParameters(std::ofstream & out) const override;

	virtual void PrintState(const State& state, std::ostream& out = std::cout) const override;
	virtual void PrintBelief(const Belief& belief, std::ostream& out = std::cout) const override;
	virtual void PrintObs(const State& state, OBS_TYPE obs, std::ostream& out = std::cout) const override;
	virtual void PrintAction(int action, std::ostream & out) const override;

/// functions that are necessary for step and action calculation
protected:

	virtual int randLegalAction(OBS_TYPE observation) const = 0;

	static CALCULATION_TYPE getLUTCalcType() { return s_calculationType; };

	/// recieve current state from simulator and update rewrd and observation
	virtual bool RcvStateIMP(intVec & buffer, State * s, double & reward, OBS_TYPE & obs) const override;

	/// check if 2 idx are in a given range on the grid
	static bool InRange(int idx1, int idx2, double range, int gridSize);

	/// return true if the robot is dead by enemy attack given random num(0-1). state is not reference by reason
	bool CalcIfKilledByEnemy(const nxnGridDetailedState & state, int enemyIdx, double & randomNum) const;

	/// return identity of the objIdx
	enum OBJECT WhoAmI(int objIdx) const;

	inline static double RandomNum()
	{	
		return Random::s_threadSafeRand[GetCurrentThreadId()].NextDouble();
	}
	/// create a vector of random numbers between 0 - 1
	static void CreateRandomVec(doubleVec & randomVec, int size);

	/// retrieve the observed state given current state and random number
	OBS_TYPE UpdateObservation(nxnGridDetailedState & state, double p) const;

	/// advance the state to the next step position (regarding to other objects movement)
	void SetNextPosition(nxnGridDetailedState & state, doubleVec & randomNum) const;

	/// change object location according to its movement properties and random number
	void CalcMovement(nxnGridDetailedState & state, const Movable_Obj *object, double rand, int objIdx) const;

	intVec GetSheltersVec() const;

	void GetNonValidLocations(const nxnGridDetailedState & state, int objIdx, intVec& nonValLoc) const;

private:
	///// fill values of subtrees
	static unsigned int SendTreeRec(VNode *node, unsigned int parentId, unsigned int id, std::vector<unsigned int> & buffer);
	static unsigned int SendTreeRec(QNode *node, unsigned int parentId, unsigned int id, unsigned int nextAvailableId, std::vector<unsigned int> & buffer);

	static int FindMaxReward(const doubleVec & rewards, double & expectedReward);
	/// create particle from available locations
	void CreateParticleVecRec(nxnGridDetailedState & state, std::vector<std::vector<std::pair<int, double> > > & objLocations, std::vector<State*> & particles, double weight, int currIdx) const;

	/// create vector of all possible particles for init state
	void InitialBeliefStateRec(nxnGridDetailedState & state, int currObj, double stateProb, std::vector<State*> & particles) const;

	/// find the observed state according to random number and original state
	void DecreasePObsRec(const nxnGridDetailedState & originalState, nxnGridDetailedState & currState, int currIdx, double pToDecrease, double &pLeft) const;

	// Rescaling state functions:
	void ScaleState(const nxnGridDetailedState & beliefState, nxnGridDetailedState & scaledState) const;
	void ScaleState(const nxnGridDetailedState & beliefState, nxnGridDetailedState & scaledState, int newGridSize, int prevGridSize) const;

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
	void InitStateIMP(const intVec & data, nxnGridDetailedState & state, nxnGridDetailedState & observation) const;

	/// add actions to specific enemy
	virtual void AddActionsToEnemy() = 0;
	virtual void AddActionsToShelter() = 0;

	/// return idx of enemy action is related to
	virtual int EnemyRelatedActionIdx(int action) const = 0;
	/// return true if action is related to enemy
	virtual bool EnemyRelatedAction(int action) const = 0;

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
	static OnlineSolverLUT s_LUT;
	static int s_lutGridSize;

	static enum CALCULATION_TYPE s_calculationType;

public:
	/// rewards for different events
	static const int REWARD_MIN;
	static const int REWARD_MAX;
	static const int REWARD_KILL_ENEMY;
	static const int REWARD_KILL_NINV;
	static const int REWARD_ILLEGAL_MOVE;
	static const int REWARD_STEP;
	static const int REWARD_FIRE;

};

/* =============================================================================
* nxnGridBelief class
* =============================================================================*/
class nxnGridBelief : public ParticleBelief
{
public:
	nxnGridBelief(std::vector<State*> particles, const DSPOMDP* model, Belief* prior = NULL, bool split = true)
		: ParticleBelief(particles, model, prior, split) {}

protected:
	virtual void Update(int action, OBS_TYPE obs) override;
	static std::vector<State*> Resample(int num, const std::vector<State*>& belief, const DSPOMDP* model, History history);
};


} // end ns despot

#endif	// NXNGRID_H