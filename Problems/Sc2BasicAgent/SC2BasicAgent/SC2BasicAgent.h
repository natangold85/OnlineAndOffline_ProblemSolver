#ifndef SC2_BASIC_AGENT_H
#define SC2_BASIC_AGENT_H

#include <string>

#include <OnlineSolverModel.h>

namespace despot 
{

enum PIXEL {FREE = 0, RED, BLUE, NUM_PIXEL_OPTIONS};
using miniMap_t = std::vector<PIXEL>;


// translation between stateId to logical repressentation of state
class SC2DetailedState : public DetailedState
{
public:
	enum STATUS_COUNT { COMMAND_CENTER, SUPPLY_DEPOTS, BARRACKS, ARMY, MINERALS, NUM_COUNTS };

	SC2DetailedState(int *arr);
	SC2DetailedState();
	explicit SC2DetailedState(STATE_TYPE state_id);
	SC2DetailedState(const State & state);

	/// init static members
	static void InitStatic();

	// Id to state interface functions
	static void InitCountsFromId(STATE_TYPE & state_id, intVec & countVec);
	static void InitMapFromId(STATE_TYPE & state_id, miniMap_t & map);

	STATE_TYPE GetStateId() const;
	OBS_TYPE GetObsId() const;
	OBS_TYPE GetObsId(double rand) const;

	static void CopyFullyObservedFromObservation(const OBS_TYPE obs, State * state);
	
	static STATE_TYPE GetIDCounts(const intVec & countVec);
	static STATE_TYPE GetIDMiniMap(const miniMap_t & map);
	static STATE_TYPE GetIDMiniMap(const miniMap_t & map, double random);

	static void InitProbMat(const miniMap_t & map, std::vector<std::pair<int, double>> &probMat);
	static void GetIDMiniMapRec(miniMap_t & map, const std::vector<std::pair<int, double>> &probMat, double prob, double & random, int currIdx);

	static double ObsProbPartialObsVars(STATE_TYPE state_id, OBS_TYPE obs);
	
	void StateVector(uintVec & vec) const;

	miniMap_t & GetMiniMap() { return m_miniMap; };
	const miniMap_t & GetMiniMap() const { return m_miniMap; };

	int &operator[](int idx) { return m_countVec[idx]; };
	int operator[](int idx) const { return m_countVec[idx]; };
	
	bool MaxCount(int idxCount) const;

	int EnemyPower() const;
	bool NoEnemy() const;
	bool RedNearBase() const;
	std::string MiniMapStr() const;

	
	// print state functions

	/// print the state
	std::string text() const;
	std::string PrintGrid() const;

	/*MEMBERS*/

	/*FULLY OBSERVABLE*/
	intVec m_countVec;

	/*PARTIALLY OBSERVABLE*/
	miniMap_t m_miniMap;


	static intVec InitNumBitsPerCount();
	
	/*STATIC MEMBERS*/
	
	static const STATE_TYPE s_ONE = 1;
	/// num bits
	static intVec s_NUM_BITS_PER_COUNT;
	static const int s_NUM_BITS_PIXEL = 2;
	static int s_NUM_BITS_COUNT_PART;

	/// gridSize
	static int s_gridSize;
	static int s_baseLocation;
	static int s_enemyBaseLocation;
	static std::vector<std::string> s_countNames;
	static std::vector<char> s_pixelStatusNames;
};


/* =============================================================================
* SC2Agent class
* =============================================================================*/
class SC2BasicAgent : public OnlineSolverModel
{	
	enum ACTION { NOTHING, BUILD_SUPPLY_DEPOT, BUILD_BARRACKS, TRAIN_ARMY, ATTACK, NUM_ACTIONS };

public:
	SC2BasicAgent(int gridSize, int baseLocation, int enemyBaseLoc);
	~SC2BasicAgent() = default;

	/// take one step for a given state and action return true if the simulation terminated, update reward and observation
	virtual bool Step(State& s, double random, int action, double & reward, OBS_TYPE& obs) const override;

	virtual int NumActions() const override { return NUM_ACTIONS; };

	virtual double GetMaxReward() const override { return REWARD_MAX; };

	virtual ValuedAction GetMinRewardAction() const override { return ValuedAction(NOTHING, -10.0); }
	/*EXTERNAL SIMULATOR FUNCTIONS*/

	/// recieve init state from simulator
	virtual void InitState() override;

	/// return grid size
	int GetGridSize() const { return m_gridSize; };

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

private:

	/// return random legal action given last observation
	virtual bool LegalAction(OBS_TYPE observation, int action) const override;

	/// recieve current state from simulator and update reward and observation
	virtual bool RcvStateIMP(intVec & data, State * s, double & reward, OBS_TYPE & obs) const override;

	// ACTIONS FUNCTION:
	void BuildSupplyDepot(SC2DetailedState & state, double & reward) const;
	void BuildBarracks(SC2DetailedState & state, double & reward) const;
	void TrainArmy(SC2DetailedState & state, double & reward) const;
	void Scout(SC2DetailedState & state, double & reward) const;
	void DefendBase(SC2DetailedState & state, double & reward) const;
	void Attack(SC2DetailedState & state, double random, double & reward) const;

	/// advance the state to the next step position (regarding to other objects movement)
	void SetNextState(SC2DetailedState & state, double spreadingEnemy, double enemyAttack) const;

protected:
	int m_gridSize;
	int m_baseLocation;
	int m_enemyBaseLocation;

	// lut for actions names
	static std::vector<std::string> s_ACTION_STR;

public:

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