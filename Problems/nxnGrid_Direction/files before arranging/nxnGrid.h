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

using locationVec = std::vector<int>;
using directionVec = std::vector<DIRECTIONS>;
using locAndDirPair = std::pair<int, DIRECTIONS>;
using doubleVec = std::vector<double>;

class DetailedState;
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
	nxnGridState(const DetailedState & state);

	static STATE_TYPE MaxState();
};

class DetailedState
{
public:
	static const int s_NUM_BITS_LOCATION = 8;
	static const int s_NUM_BITS_DIRECTION = 4;
	
	DetailedState(int sizeState, bool isEmpty);
	DetailedState(STATE_TYPE state_id);
	DetailedState(const State & state);

	static locationVec InitLocationsFromId(STATE_TYPE state_id);
	static directionVec InitDirectionsFromId(STATE_TYPE state_id);
	static void GetSelfStatus(STATE_TYPE state_id, int & selfLocation, DIRECTIONS & selfDirection);

	STATE_TYPE GetStateId() const;
	static STATE_TYPE GetLocationsId(const locationVec & locations);
	static STATE_TYPE GetDirectionsId(const directionVec & directions);

	unsigned int size() const { return m_locations.size(); };
	/// init static members so previous model will not affect current
	static void InitStatic();
	
	static STATE_TYPE MaxState();

	/// print the state
	std::string text() const;
	static void PrintGrid(const locationVec & locations, std::string & buffer);
	/// return char identified location
	static char ObjIdentity(const locationVec & locations, int location);

	void EraseNonInv();
	void EraseObject(int objectIdx);
	
	bool NoEnemies(int gridSize) const;
	bool IsDead(int objIdx, int gridSize) const;
	locationVec m_locations;
	directionVec m_directions;

public:
	/// shelter vector
	static std::vector<int> s_shelters;

	/// number of objects in grid (size of state vector)
	static int s_numNonInvolved;
	/// the border between the enemies location and the non-involved location in the state vector
	static int s_numEnemies;
	/// gridSize
	static int s_gridSize;
	/// target location
	static int s_targetLoc;
};

class DetailedObservation
{
public:
	DetailedObservation() = default;
	DetailedObservation(OBS_TYPE);
	DetailedObservation(const DetailedState & state, const Self_Obj & self, int gridSize, doubleVec & prob);

	static locationVec InitObservation(const locationVec & realLocation, const Self_Obj & self, int gridSize, doubleVec & prob);
	static locationVec InitObservation(OBS_TYPE obs);

	OBS_TYPE GetObsType();

	/// print the observation
	std::string text() const;

	/*MEMBERS*/
	locationVec m_locations;
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
	using singleObjStatus = std::pair<int, int>;
	using lut_t = std::map < STATE_TYPE, doubleVec >;
	///	enum of objects
	enum OBJECT { SELF, ENEMY, NON_INV, SHELTER, TARGET, NUM_OBJECTS };
	enum VBS_OBJECTS { SELF_VBS, ENEMY_VBS, NON_INVOLVED_VBS, SHELTER_VBS, TARGET_VBS, OBSERVED_ENEMY_VBS, OBSERVED_NON_INVOLVED_VBS };
	/// enum of type of calculation using sarsop data map
	enum CALCULATION_TYPE { WITHOUT, ALL, WO_NINV, JUST_ENEMY, ONE_ENEMY, WO_NINV_STUPID };
	 
	explicit nxnGrid(int gridSize, int traget, Self_Obj & self, std::vector<locationVec> & objectsInitLoc);
	~nxnGrid() = default;
	
	// Functions for self use

	static bool IsOnline() { return s_isOnline; };
	static bool IsResampleLastObs() { return s_resampleFromLastObs; };
	static bool IsVBSEvaluator() { return s_isVBSEvaluator; };
	static bool ToSendTree() { return s_toSendTree; };

	/// init udp server
	static bool InitUDP(int portNum = -1, int sendTreePort = -1);
	/// init offline decision lut
	static void InitLUT(lut_t & offlineLut, int offlineGridSize, CALCULATION_TYPE cType = WITHOUT);

	/// special resample for VBS synchronization
	static std::vector<State*> ResampleLastObs(int num, const std::vector<State*>& belief, const DSPOMDP* model, History history);
	/// init model type, periodOfdecision
	static void InitStaticMembers(bool isOnline, int periodOfDecision = 1, bool resampleFromLastObs = true);
	/// update real action given action (if we want to make decision once in every several algorithm decisions)
	static void UpdateRealAction(int & action);

	/// send end of trial signal
	static void SendEndRun();
	/// send num actions to visualizator
	static void SendModelDetails(int numActions, int numObj);
	/// send tree in udp for visualisation
	static void SendTree(State * state, VNode *root);

	bool NoEnemies(State * state);
	/// add enemy object
	void AddObj(Attack_Obj&& obj); 
	/// add non involved object
	void AddObj(Movable_Obj&& obj);
	/// add shelter
	void AddObj(ObjInGrid&& obj);

	/// count number of moving objects(self, enemies, non involved, shelters)
	int CountAllObjects() const;
	/// count number of moving objects(self, enemies, non involved)
	int CountMovingObjects() const;

	/// return grid size
	int GetGridSize() const { return m_gridSize; };

	/// get observed location of object (identified by idx) given observation
	int GetObsLoc(OBS_TYPE obs, int objIdx) const;
	int GetObsLoc(OBS_TYPE obs, int objIdx) const;

	/// return vector of rewards given model and prior
	static void ChoosePreferredAction(const State * state, POMCPPrior * prior, const DSPOMDP* m, doubleVec & expectedRewards);
	/// return a preferred action given model and prior
	static int ChoosePreferredAction(const State * state, POMCPPrior * prior, const DSPOMDP* m, double & expectedReward);

	/// initialize beliefState according to history
	void InitBeliefState(DetailedState & beliefState, const History & h) const;

	/// add to state shelter locations
	void AddSheltersLocations(locationVec & shelters) const;
	locationVec GetSheltersVec() const;
	
	/// recieve init state from simulator
	void InitState();
	/// send action to simulator
	void SendAction(int action);
	/// recieve current state from simulator and update rewrd and observation
	bool RcvState(State * s, int action, OBS_TYPE lastObs, double & reward, OBS_TYPE & obs);

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

/// functions that are necessary for step and action calculation
protected:
	/// check if 2 idx are in a given range on the grid
	static bool InRange(int idx1, int idx2, double range, int gridSize);

	/// return true if the robot is dead by enemy attack given random num(0-1). state is not reference by reason
	bool CalcIfDead(int enemyIdx, DetailedState & state, double & randomNum) const;

	/// return identity of the objIdx
	enum OBJECT WhoAmI(int objIdx) const;

	/// create a vector of random numbers between 0 - 1
	static void CreateRandomVec(doubleVec & randomVec, int size);

	/// retrieve the observed state given current state and random number
	OBS_TYPE FindObservation(DetailedState & state, doubleVec obsRandNums) const;

	/// advance the state to the next step position (regarding to other objects movement)
	void SetNextPosition(DetailedState & state, doubleVec & randomNum) const;

	/// change object location according to its movement properties and random number
	void CalcMovement(DetailedState & state, const Movable_Obj *object, int objIdx, double rand, int targetLocation = -1) const;

	// CHECK LOCATIONS :
	/// check if the next location (location + (x,y)) is in grid boundary
	bool InBoundary(int location, int xChange, int yChange) const;
	/// return true if the object idx location does not repeat in state
	static bool NoRepetitions(locationVec & state, int currIdx, int gridSize);
	/// return true if location is valid for a given state (no repeats)
	static bool ValidLocation(locationVec & state, int location);
	/// return true if location is valid for a given state (no repeats & in grid)
	static bool ValidLegalLocation(locationVec & state, Coordinate location, int end, int gridSize);

private:
	///// fill values of subtrees
	static unsigned int SendTreeRec(VNode *node, unsigned int parentId, unsigned int id, std::vector<unsigned int> & buffer);
	static unsigned int SendTreeRec(QNode *node, unsigned int parentId, unsigned int id, unsigned int nextAvailableId, std::vector<unsigned int> & buffer);

	/// implementation of choose prefferred action
	void ChoosePreferredActionIMP(const DetailedState & state, doubleVec & expectedReward) const;

	static int FindMaxReward(const doubleVec & rewards, double & expectedReward);
	/// create particle from available locations
	void CreateParticleVecRec(DetailedState & state, std::vector<std::vector<std::pair<int, double> > > & objLocations, std::vector<State*> & particles, double weight, int currIdx) const;

	/// create vector of all possible particles for init state
	void InitialBeliefStateRec(DetailedState & state, int currObj, double stateProb, std::vector<State*> & particles) const;

	/// find the observed state according to random number and original state
	void DecreasePObsRec(locationVec & currState, const locationVec & originalState, int currIdx, double pToDecrease, double &pLeft) const;
	
	/// move a specific object closer to robot
	void GetCloser(locationVec & state, int objIdx, int gridSize) const;

	/// return move given random number (0-1)
	int FindObjMove(int currLocation, double random, int gridSize) const;

	/// return true if observedLocation is surrouning a location (in 1 of the 8 directions to location)
	static bool InSquare(int location, int location2, int squareSize, int gridSize);

	// Rescaling state functions:
	void ScaleState(const DetailedState & beliefState, DetailedState & scaledState, locationVec & sheltersLocation) const;
	void ScaleState(const DetailedState & beliefState, DetailedState & scaledState, locationVec & sheltersLocation, int newGridSize, int prevGridSize) const;

	/// initialize rewards vector of 2 enemies from 2 vectors of rewards vec of 1 enemy
	void Combine2EnemiesRewards(const locationVec & beliefStateLocations, const doubleVec & rewards1E, const doubleVec & rewards2E, doubleVec & rewards) const;


	/// move non protected shelters to close non-object location
	void MoveNonProtectedShelters(const locationVec & baliefState, locationVec & scaledState, int oldGridSize, int newGridSize) const;
	/// move object location to the closest scaled spot but current scaled location
	void MoveObjectLocation(const locationVec & beliefState, locationVec & scaledState, int objIdx, int oldGridSize, int newGridSize) const;
	/// in case self is on target after scaling we need to shift it from target
	void ShiftSelfFromTarget(const locationVec & beliefState, locationVec & scaledState, int oldGridSize, int newGridSize) const;
	/// drop shelter from state (make shelter in accessible)
	void DropNonProtectedShelter(locationVec & woShelter, int oldGridSize, int newGridSize) const;

	/// return num enemies in calculation
	int NumEnemiesInCalc() const;
	/// return true if the lut is without non-involved
	int NumNonInvInCalc() const;


	/// init state according to s_objectsInitLocations
	void InitStateRandom();
	/// init state from vbs simulator
	void InitStateVBS();
	/// recieve from vbs smulator current state
	void InitStateIMP(locationVec & state, locationVec & observation);

	/// return the object location considering object type and object num(idx)
	int FindObject(locationVec & state, locationVec & identity, int object, int idx);
	int FindObject(locationVec & state, locationVec & identity, int object, int observedObj, bool & isObserved, int idx);

	/// add actions to specific enemy
	virtual void AddActionsToEnemy() = 0;
	virtual void AddActionsToShelter() = 0;

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
	static std::vector<locationVec> s_objectsInitLocations;

	static int s_numBasicActions;
	static int s_numEnemyRelatedActions;

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
	static bool s_resampleFromLastObs;
	static bool s_isVBSEvaluator;
	static bool s_toSendTree;

	static enum CALCULATION_TYPE s_calculationType;
	static int s_periodOfDecision;

	/// rewards for different events
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