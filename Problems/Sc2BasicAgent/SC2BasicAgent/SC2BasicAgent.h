#ifndef SC2_BASIC_AGENT_H
#define SC2_BASIC_AGENT_H

#include <string>

#include <OnlineSolverModel.h>

namespace despot 
{

using pairInt = std::pair<int, int>;

// translation between stateId to logical repressentation of state
class SC2DetailedState : public DetailedState
{
public:
	enum SELF_STATUS_COUNT { COMMAND_CENTER, SUPPLY_DEPOTS, BARRACKS, ARMY, MINERALS, NUM_COUNTS };
	enum ENEMY_STATUS { BASE_POWER, ARMY_POWER, ARMY_LOCATION, ENEMY_OBSERVED_LOCATION, NUM_ENEMY_STATUS };

	SC2DetailedState(int *arr);
	SC2DetailedState();
	explicit SC2DetailedState(STATE_TYPE state_id);
	SC2DetailedState(const State & state);

	/// init static members
	static void InitStatic(int baseLocation = -1, int enemyBaseLoc = -1, int gridSize = -1);

	// Id to state interface functions
	static void InitCountsFromId(STATE_TYPE & state_id, intVec & countVec);
	static void InitEnemyStatusFromId(STATE_TYPE & state_id, intVec & enemyStatus);
	static void DeleteEnemyFromId(STATE_TYPE & state_id);

	STATE_TYPE GetStateId() const;
	OBS_TYPE GetObsId() const;
	OBS_TYPE GetObsId(double rand) const;

	static void CopyFullyObservedFromObservation(const OBS_TYPE obs, State * state);
	
	static STATE_TYPE GetIDCounts(const intVec & countVec);
	static STATE_TYPE GetIDEnemyStatus(const intVec & enemyStatus);
	STATE_TYPE GetIDEnemyDetails(double rand) const;

	static double ObsProbPartialObsVars(STATE_TYPE state_id, OBS_TYPE obs);
	
	void StateVector(uintVec & vec) const;

	int &operator[](int idx) { return m_countVec[idx]; };
	int operator[](int idx) const { return m_countVec[idx]; };
	
	bool MaxCount(int idxCount) const;

	bool NoEnemy() const;
	bool IsObservedEnemy() const;
	bool EnemyLocation(int & enemyPower) const;
	bool UpdatePower(int attackLocation, int power);
	// print state functions

	/// print the state
	std::string text() const;

	/*MEMBERS*/

	/*FULLY OBSERVABLE*/
	intVec m_countVec;

	/*PARTIALLY OBSERVABLE*/
	intVec m_enemyStatus;


	/*STATIC MEMBERS*/
		
	/// num bits
	static intVec InitNumBitsPerCount();
	static intVec InitNumBitsPerEnemyStatus();

	static intVec s_NUM_BITS_PER_COUNT;
	static int s_NUM_BITS_COUNT_PART;

	static intVec s_NUM_BITS_PER_ENEMY_STATUS;
	static int s_NUM_BITS_ENEMY_STATUS_PART;

	/// static details
	static int s_gridSize;
	static int s_baseLocation;
	static int s_enemyBaseLocation;

	static const int s_NON_VALID_LOCATION = -1;
	// static lut
	static std::vector<std::string> s_countNames;
	static std::vector<std::string> s_enemyStatusNames;
};


/* =============================================================================
* SC2Agent class
* =============================================================================*/
class SC2BasicAgent : public OnlineSolverModel
{	
	using EnemyTransitionLUT = std::map<int, int>;
	using AttacksLUT = std::map<pairInt, pairInt>;
public:
	enum ACTION { NOTHING, BUILD_SUPPLY_DEPOT, BUILD_BARRACKS, TRAIN_ARMY, ATTACK, NUM_ACTIONS };
	
	SC2BasicAgent(int gridSize, int baseLocation);
	~SC2BasicAgent() = default;

	static void InitQTable(OnlineSolverLUT & qTable, int gridSize, int baseLocation);

	/// take one step for a given state and action return true if the simulation terminated, update reward and observation
	virtual bool Step(State& s, double random, int action, double & reward, OBS_TYPE& obs) const override;

	virtual int NumActions() const override { return NUM_ACTIONS; };

	virtual double GetMaxReward() const override { return REWARD_MAX; };

	virtual ValuedAction GetMinRewardAction() const override { return ValuedAction(NOTHING, -10.0); }
	/*EXTERNAL SIMULATOR FUNCTIONS*/

	/// recieve init state from simulator
	virtual void InitState() override;

	/// return initial state
	virtual State *CreateStartState(std::string type) const override;
	///  return initial belief
	virtual Belief* InitialBelief(const State* start, std::string type) const override;

	/// display model params to out
	virtual void DisplayParameters(std::ofstream & out) const override;

	/*PRINT FUNCTIONS*/
	virtual void PrintState(const State& state, std::ostream& out = std::cout) const override;
	virtual void PrintBelief(const Belief& belief, std::ostream& out = std::cout) const override;
	virtual void PrintObs(const State& state, OBS_TYPE obs, std::ostream& out = std::cout) const override;
	virtual void PrintAction(int action, std::ostream & out) const override;

protected:
	/// return random legal action given last observation
	virtual bool LegalAction(OBS_TYPE observation, int action) const override;

	/// recieve current state from simulator and update reward and observation
	virtual bool RcvStateIMP(intVec & data, State * s, double & reward, OBS_TYPE & obs) const override;

private:
	/// return grid size
	int getGridSize() const { return m_gridSize; };


	// ACTIONS FUNCTION:
	void buildSupplyDepot(SC2DetailedState & state, double & reward) const;
	void buildBarracks(SC2DetailedState & state, double & reward) const;
	void trainArmy(SC2DetailedState & state, double & reward) const;
	void scout(SC2DetailedState & state, double & reward) const;
	void defendBase(SC2DetailedState & state, double & reward) const;
	void attack(SC2DetailedState & state, double random, double & reward) const;

	/// advance the state to the next step position (regarding to other objects movement)
	void setNextState(SC2DetailedState & state, double spreadingEnemy, double enemyAttack) const;

protected:
	int m_gridSize;
	int m_baseLocation;
	int m_enemyBaseLocation;

	// lut for actions names
	static std::vector<std::string> s_ACTION_STR;

	static OnlineSolverLUT s_Q_TABLE;
	static EnemyTransitionLUT s_ENEMY_TRANSITION_LUT;
	static AttacksLUT s_ATTACKS_LUT;
public:

	// power transition
	static const int NUM_START_SCV = 10;
	static const int COMMAND_CENTER_2_POWER = 9;
	static const int SUPPLY_DEPOT_2_POWER = 4;
	static const int BARRACKS_2_POWER = 4;

	/// prices of minerals for different action
	static const int TRAINING_PRICE = 50;
	static const int BARRACKS_PRICE = 100;
	static const int SUPPLY_DEPOT_PRICE = 200;

	static const int MINNING_RATE = 50;

	/// rewards for different events

	static const int REWARD_ILLEGAL_MOVE;
	static const int REWARD_STEP;

	static const int REWARD_MAX;
	static const int REWARD_MIN;
};


/* =============================================================================
* SC2AgentBelief class
* =============================================================================*/
class SC2AgentBelief : public ParticleBelief
{
public:
	SC2AgentBelief(std::vector<State*> particles, const DSPOMDP* model, Belief* prior = NULL, bool split = true)
	: ParticleBelief(particles, model, prior, split) {}

protected:
	virtual void Update(int action, OBS_TYPE obs) override;
	static std::vector<State*> Resample(int num, const std::vector<State*>& belief, const DSPOMDP* model, History history);
};


} // end ns despot

#endif	// SC2_BASIC_AGENT_H