//	Purpose: create POMDP format given specific state (including 1 robot, enemies, non-involved movable objects and shelters on NxN Grid)
//			win is getting to an idx in the map(idxTarget), loss is killing non-involved and the death of the robot by the enemies.

//	Author: Natan Gold

//	COMMENTS REGARDING THE RULES OF THE GAME:
//	1-	moving objects can not be in the same idx in the grid
//	2-	a shot of the robot is absorbed by the first object that it met whether it is an enemy, non-involved or shelter
//	3-	when in enemy range the robot has specific probability to be killed by the enemy
//	4-	there should not be enemy or self with range 0
//  5-	the model is yet not support varied initial locations of objects
//	6-	the model is yet not support more than 1 enemy and 1 shelter


#ifndef NXNGRIDOFFLINE_H
#define NXNGRIDOFFLINE_H

#include <vector>
#include <map>

#include "Self_Obj.h"
#include "Attack_Obj.h"
#include "Movable_Obj.h"
#include "ObjInGrid.h"

#include "Move_Properties.h"

struct State
{
	State(int numObjects)
	: m_locations(numObjects)
	, m_directions(numObjects)
	{}

	unsigned int size() const { return m_locations.size(); };
	static long State2Idx(State & state, int gridSize);
	static State Idx2State(long state, int gridSize);

	bool operator==(const State & state) const;

	bool ValidDirections2Locations(int gridSize) const;

	std::vector<int> m_locations;
	std::vector<DIRECTIONS> m_directions;
};

//struct Observation_Instance
//{
//	Observation_Instance(int numObjects)
//	: m_locations(numObjects)
//	{}
//
//	unsigned int size() const { return m_locations.size(); };
//	static long Observation2Idx(Observation_Instance & observation, int gridSize);
//	static Observation_Instance Idx2Observation(long observation, int gridSize);
//
//	std::vector<int> m_locations;
//};

class nxnGridOffline
{
	/// operator << to print the pomdp
	friend std::ostream& operator<<(std::ostream& o, const nxnGridOffline& map);

public:
	using outFile = std::ofstream;

	explicit nxnGridOffline() = default;
	explicit nxnGridOffline(int gridSize, int targetIdx, Self_Obj& self, bool isFullyObs, double discount = 0.95);
	~nxnGridOffline() = default;
	nxnGridOffline(const nxnGridOffline &) = default;
	nxnGridOffline& operator=(const nxnGridOffline&) = default;

	using intVec = std::vector<int>; 
	using observation_t = intVec;
	using mapProb = std::map<observation_t, double>;
	using pairMap = std::pair<intVec, double>;

	enum IDENTITY { SELF = 0, ENEMY = 1, NON_INVOLVED = 2, SHELTER = 3 };

	/// save model in pomdp format to file
	virtual void SaveInPomdpFormat(outFile & fptr) = 0;

	// init model functions

	void UpdateSelf(Self_Obj && obj);
	void AddObj(Attack_Obj&& obj);
	void AddObj(Movable_Obj&& obj);
	void AddObj(ObjInGrid&& obj);

	// count objects functions

	/// return number of moving objects in grid
	int CountMovableObj() const;
	
	int CountEnemies() const;
	int CountNInv() const;
	int CountShelters() const;
	int GetGridSize() const;
	bool IsFullyObs() const { return m_isFullyObs; };
	// change model functions

	void SetLocationSelf(Coordinate & newLocation);
	void SetLocationEnemy(Coordinate & newLocation, int idxEnemy);
	void SetLocationNonInv(Coordinate & newLocation, int idxNonInv);
	void SetLocationShelter(Coordinate & newLocation, int idxShelter);
	void SetTarget(int idx);
	void SetGridSize(int gridSize);

	long StateCount2StateIdx(long stateCount) const;

	virtual int GetNumActions() const = 0;

	static const int s_REWARD_STEP = -1.0;
	static const int s_REWARD_ATTACK = -1.0;
	static const int s_REWARD_WIN = 50;
	static const int s_REWARD_LOSS = -100.0;

/// protected to access for actions implementations
protected:
	// map & model properties
	int m_gridSize;
	int m_targetIdx;
	double m_discount;
	bool m_isFullyObs;
	// objects in model
	Self_Obj m_self;
	std::vector<Attack_Obj> m_enemyVec;
	std::vector<Movable_Obj> m_nonInvolvedVec;
	std::vector<ObjInGrid> m_shelterVec;

/// functions necessary for actions implementations

	// main functions for saving format to file: 

	/// insert observation and reward to file
	void ObservationsAndRewards(outFile & fptr);

	/// insert of possible state or observations(depending on type) to buffer
	void WriteAllStates(std::string& buffer);
	void WriteAllObservations(std::string& buffer);

	void CalcStatesAndObs(const char * type, std::string& buffer);

	/// insert probability to init of all states
	void CalcStartState(std::string& buffer);

	/// insert the end-states positions (states and probabilities) from a single state(state) to buffer
	double PositionSingleState(State & state, State & currentState, intVec & shelters, double prob, std::string& action, std::string& buffer) const;
	
	/// insert states where robot is in target position transition to win state to buffer
	void AddTargetPositionRec(State & state, int currIdx, std::string& buffer);
	/// calculation of transition of all actions
	void AddActionsAllStates(std::string& buffer);

	/// return true if there is dead non-involved
	bool IsNonInvDead(State & state) const;
	bool IsNonInvDead(intVec & locations) const;

	/// returns true if the location is sheltered
	bool SearchForShelter(int location) const;
	/// return location of the nearest shelter
	int FindNearestShelter(int location) const;
	/// create location vector of all shelters
	void CreateShleterVec(intVec & shelters) const;

	/// translate a state to the pomdp format string
	std::string GetStringState(State & state) const;
	/// translate an observation to the pomdp format string
	static std::string GetStringObservation(observation_t & observation, int gridSize);

	// translate to string with higher precision
	static std::string Dbl2Str(double d);

	/// return true if location is not presence on state
	//bool NoRepeatsLocation(State& state, int location) const;
	/// return true if the state idx is with no repetitions to previous objects locations
	//bool NoRepeatsIdxLocation(State & locationsVec, int idx) const;
	/// return true if state + advance factor is inside the grid
	static bool InBoundary(int state, int advanceFactor, int gridSize);
	/// return true if state + xChange + yChange is inside the grid
	static bool InBoundary(int state, int xChange, int yChange, int gridSize);

	/// return true if objIdx is an idx of an enemy
	bool IsEnemy(int objIdx) const;

	std::shared_ptr<Move_Properties> GetObjMovement(int objIdx) const;
private:

	// main functions for saving format to file: 

	/// insert init lines, init states, state list and comments to file
	virtual void CommentsAndInitLines(outFile & fptr) = 0;
	/// insert transitions of all actions to file
	virtual void AddAllActions(outFile & fptr) = 0;

	/// find corresponding state to stateCount (basic count of state)
	void FindStateCount(State & state, long & count, int currIdx) const;

	/// run on all possible states insert them to buffer
	void WriteAllStatesRec(State& state, int currIdx, std::string& buffer);
	/// run on all possible observations insert them to buffer (observing only locations)
	void WriteAllObservationsRec(observation_t & observation, int currIdx, std::string& buffer);

	/// run on all states and insert the probability of each state to init in
	void CalcStartStateRec(State& state, State& initState, int currIdx, std::string& buffer);

	/// insert possible moveStates (states and probability) to pMap
	void AddMoveStatesRec(State & state, double prob, int currIdx, std::string & prefix, std::string & buffer) const;

	/// add state and probability to state (itr) to buffer
	static void AddStateToBuffer(std::string& buffer, pairMap & itr, int gridSize);

	/// returns the real end-state from a given moveState
	State MoveToIdx(State state, std::vector<intVec> & moveStates, intVec arrOfIdx) const;

	///insert all observations to buffer
	void CalcObs(std::string& buffer);
	/// run on all states and insert observation probability to buffer
	void CalcObsRec(State & state, int currIdx, std::string& buffer);
	/// insert observation probability of a single state
	void CalcObsSingleState(State & state, std::string& buffer);
	/// calculate probability of observation from originalState
	void CalcObsMapRec(observation_t & observation, State & originalState, mapProb& pMap, double pCurr, int currObj);

	/// return true if object is in observation range
	static bool InObsRange(int self, int object, int gridSize, int range);
	
	/// return true if there is any deads in state
	bool AnyDead(State & state) const;
};

# endif //NXNGRIDOFFLINE_H