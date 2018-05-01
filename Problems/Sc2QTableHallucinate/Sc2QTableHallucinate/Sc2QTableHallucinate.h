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
	struct IntRange
	{
		IntRange() : m_min(INT_MAX), m_max(INT_MIN) {}
		int m_min;
		int m_max; 
	};

	static void InitStateValFromLUT(std::vector<IntRange> rangeStateVal);

	SC2DetailedState(int *arr);
	SC2DetailedState();
	explicit SC2DetailedState(STATE_TYPE state_id);
	explicit SC2DetailedState(const intVec & vars);
	SC2DetailedState(const State & state);

	// Id to state interface functions
	static void InitVarsFromId(STATE_TYPE & state_id, intVec & vars);

	STATE_TYPE GetStateId() const;
	OBS_TYPE GetObsId() const;
	OBS_TYPE GetObsId(double rand) const;

	void State2Buffer(uintVec & buffer);
	
	static STATE_TYPE GetIDFullyObsVars(const intVec & fullyObsVars);	
	static bool IsTerminal(STATE_TYPE stateId);

	/// print the state
	std::string text() const;

	/*MEMBERS*/

	// in this implementation state is fully observable
	/*FULLY OBSERVABLE*/
	intVec m_fullyObsVars;

	/*STATIC MEMBERS*/	
	/// num bits
	static int s_NUM_VARS_IN_STATE;
	static int s_NUM_BITS_FULLY_OBSERVED_PART;
	static intVec s_NUM_BITS_PER_FULLY_OBSERVED_VARS;
	static intVec s_OFFSET_FULLY_OBSERVED_VARS;

	static STATE_TYPE WIN_STATE;
	static STATE_TYPE LOSS_STATE;
	static STATE_TYPE TIE_STATE;
};


/* =============================================================================
* SC2Agent class
* =============================================================================*/
class SC2Agent : public OnlineSolverModel
{	

	using StateAndProb = std::pair<STATE_TYPE, float>;
	using actionResultVec = std::vector<std::vector<StateAndProb> >;
	using TransitionLUT = std::map<STATE_TYPE, actionResultVec >;
	using RewardLUT = std::map<STATE_TYPE, double>;

public:
	SC2Agent();
	~SC2Agent() = default;

	static bool InitTables(const std::string & qtableName, const std::string & ttableName, const std::string & rtableName, int numStateVisitsToContinueSimulation);

	/// take one step for a given state and action return true if the simulation terminated, update reward and observation
	virtual bool Step(State& s, double random, int action, double & reward, OBS_TYPE& obs) const override;

	virtual int NumActions() const override { return s_NUM_ACTIONS; };

	virtual double GetMaxReward() const override { return REWARD_WIN; };

	virtual ValuedAction GetMinRewardAction() const override { return ValuedAction(0, REWARD_LOSS); }

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
	/// tables functions 
	static bool Init_QTable(const std::string & qtableName);
	static bool Init_TTable(const std::string & ttableName, int numStateVisitsToContinueSimulation);
	static bool Init_RTable(const std::string & rtableName);

	static void RetrieveTableState(std::string str, intVec & stateVars);
	static void RetrieveTableValues(std::string str, doubleVec & vals);

	static void RetrieveNumFromStr(std::string str, intVec & vals);
	static float FindNextNum(const std::string& str);

protected:
	// lut for actions names
	static int s_NUM_ACTIONS;

	static OnlineSolverLUT s_Q_TABLE;
	static TransitionLUT s_TRANSITION_LUT;
	static RewardLUT s_REWARD_LUT;
};


} // end ns despot

#endif	// SC2_BASIC_AGENT_H