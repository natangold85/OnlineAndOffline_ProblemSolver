#include "Sc2QTableHallucinate.h"

#include "Move_Properties.h"
#include "Coordinate.h"

#include <string>
#include <math.h>

namespace despot 
{

static OBS_TYPE s_ObsWin = 0;
static OBS_TYPE s_ObsLoss = 0;

static int SumVector(const intVec & vec)
{
	int sum = 0;
	for (auto v : vec)
		sum += v;

	return sum;
}

// init static members
int SC2DetailedState::s_NUM_VARS_IN_STATE(0);
int SC2DetailedState::s_NUM_BITS_FULLY_OBSERVED_PART(0);

intVec SC2DetailedState::s_NUM_BITS_PER_FULLY_OBSERVED_VARS;
intVec SC2DetailedState::s_OFFSET_FULLY_OBSERVED_VARS;

STATE_TYPE SC2DetailedState::WIN_STATE = 0;
STATE_TYPE SC2DetailedState::LOSS_STATE = 0;
STATE_TYPE SC2DetailedState::TIE_STATE = 0;
// tables
SC2Agent::OnlineSolverLUT SC2Agent::s_Q_TABLE;
SC2Agent::TransitionLUT SC2Agent::s_TRANSITION_LUT;
SC2Agent::RewardLUT SC2Agent::s_REWARD_LUT;

// model params
int SC2Agent::s_NUM_ACTIONS = -1;

const int OnlineSolverModel::REWARD_WIN = 1;;
const int OnlineSolverModel::REWARD_LOSS = -1;

/* =============================================================================
* State Functions
* =============================================================================*/

void SC2DetailedState::InitStateValFromLUT(std::vector<IntRange> rangeStateVal)
{
	s_NUM_VARS_IN_STATE = rangeStateVal.size();
	s_NUM_BITS_PER_FULLY_OBSERVED_VARS.resize(s_NUM_VARS_IN_STATE);
	s_OFFSET_FULLY_OBSERVED_VARS.resize(s_NUM_VARS_IN_STATE);

	for (int i = 0; i < s_NUM_VARS_IN_STATE; ++i)
	{
		s_OFFSET_FULLY_OBSERVED_VARS[i] = rangeStateVal[i].m_min;
		
		int range = rangeStateVal[i].m_max - rangeStateVal[i].m_min + 1;
		int numBits = 0;
		while (range != 0)
		{
			++numBits;
			range >>= 1;
		}

		s_NUM_BITS_PER_FULLY_OBSERVED_VARS[i] = numBits;
		s_NUM_BITS_FULLY_OBSERVED_PART += numBits;
	}

	WIN_STATE = -1;
	LOSS_STATE = -2;
	TIE_STATE = -3;
}

double DetailedState::ObsProb(STATE_TYPE state_id, OBS_TYPE obs)
{
	// so far state is fully observable
	return state_id == obs;
}

SC2DetailedState::SC2DetailedState()
: m_fullyObsVars(s_NUM_VARS_IN_STATE)
{
}

SC2DetailedState::SC2DetailedState(const State & state)
: SC2DetailedState(state.state_id)
{
}

SC2DetailedState::SC2DetailedState(STATE_TYPE stateId)
: m_fullyObsVars(s_NUM_VARS_IN_STATE)
{
	InitVarsFromId(stateId, m_fullyObsVars);
}

SC2DetailedState::SC2DetailedState(const intVec & vars)
: m_fullyObsVars(vars)
{
}

SC2DetailedState::SC2DetailedState(int * arr)
: m_fullyObsVars(s_NUM_VARS_IN_STATE)
{
	int idx = 0;
	for (int i = 0; i < s_NUM_VARS_IN_STATE; ++i, ++idx)
		m_fullyObsVars[i] = arr[idx];

}


void SC2DetailedState::InitVarsFromId(STATE_TYPE & stateId, intVec & vars)
{
	// retrieve each location according to num bits vec
	for (int var = 0; var < s_NUM_VARS_IN_STATE; ++var)
	{
		int bitsForSingleVar = (s_ONE << s_NUM_BITS_PER_FULLY_OBSERVED_VARS[var]) - 1;
		vars[var] = stateId & bitsForSingleVar;
		stateId >>= s_NUM_BITS_PER_FULLY_OBSERVED_VARS[var];
	}
}


STATE_TYPE SC2DetailedState::GetStateId() const
{
	STATE_TYPE stateId = GetIDFullyObsVars(m_fullyObsVars);	
	return stateId;
}

OBS_TYPE SC2DetailedState::GetObsId() const
{
	return GetStateId();
}

OBS_TYPE SC2DetailedState::GetObsId(double rand) const
{
	// all vars are fully observed
	return GetObsId();
}

STATE_TYPE SC2DetailedState::GetIDFullyObsVars(const intVec & fullyObsVars)
{
	STATE_TYPE id = 0;

	for (int var = s_NUM_VARS_IN_STATE - 1; var >= 0; --var)
	{
		id <<= s_NUM_BITS_PER_FULLY_OBSERVED_VARS[var];
		id |= fullyObsVars[var];
	}

	return id;
}

bool SC2DetailedState::IsTerminal(STATE_TYPE stateId)
{
	return stateId == WIN_STATE || stateId == LOSS_STATE || stateId == TIE_STATE;
}

std::string SC2DetailedState::text() const
{
	std::string ret = "(";
	for (int i = 0; i < m_fullyObsVars.size(); ++i)
	{
		ret += std::to_string(m_fullyObsVars[i]) + ", ";
	}
	ret += ")\n";

	return ret;
}


void SC2DetailedState::State2Buffer(uintVec & buffer)
{
	for (auto v : m_fullyObsVars)
		buffer.emplace_back(v);
}


/* =============================================================================
* SC2Agent Functions
* =============================================================================*/

void OnlineSolverModel::ChoosePreferredAction(POMCPPrior * prior, const DSPOMDP* m, doubleVec & expectedRewards)
{
	// TODO : update reward vec for all actions
	expectedRewards = doubleVec(m->NumActions(), SC2Agent::REWARD_LOSS);
}

int OnlineSolverModel::ChoosePreferredAction(POMCPPrior * prior, const DSPOMDP* m, double & expectedReward)
{
	// TODO : return preffered action and update reward 
	return rand() % m->NumActions();
}

void OnlineSolverModel::InsertState2Buffer(uintVec & buffer, State * state)
{
	SC2DetailedState s(state->state_id);
	s.State2Buffer(buffer);
}

SC2Agent::SC2Agent()
{
}

bool SC2Agent::InitTables(const std::string & qtableName, const std::string & ttableName, const std::string & rtableName, int numStateVisitsToContinueSimulation)
{
	if (qtableName != "" && !Init_QTable(qtableName))
		return false;

	if (ttableName != "" && !Init_TTable(ttableName, numStateVisitsToContinueSimulation))
		return false;

	if (rtableName != "" && !Init_RTable(rtableName))
		return false;
}

bool SC2Agent::Init_QTable(const std::string & qtableName)
{
	std::ifstream qTable(qtableName, std::ios::in);

	if (qTable.fail())
		return false;

	int numVarsInState = -1;
	int numActions = -1;

	// build q table and init detailed state params
	std::vector<SC2DetailedState::IntRange> rangeStateVal;
	std::vector<std::pair<intVec, doubleVec>> stateAndVals;
	while (!qTable.eof())
	{
		std::string txt;
		std::getline(qTable, txt);
		if (txt.find('[') != std::string::npos)
		{
			intVec stateVars;
			RetrieveTableState(txt, stateVars);
			doubleVec vals;
			RetrieveTableValues(txt, vals);
			if (numVarsInState <= 0)
			{
				numVarsInState = stateVars.size();
				numActions = vals.size();
				rangeStateVal.resize(numVarsInState);
			}
			else if (numVarsInState != stateVars.size() || numActions != vals.size())
				return false;

			std::pair<intVec, doubleVec> pair(stateVars, vals);
			stateAndVals.emplace_back(pair);

			for (int i = 0; i < numVarsInState; ++i)
			{
				rangeStateVal[i].m_max = max(rangeStateVal[i].m_max, stateVars[i]);
				rangeStateVal[i].m_min = min(rangeStateVal[i].m_min, stateVars[i]);
			}

		}
	}

	s_NUM_ACTIONS = numActions;
	SC2DetailedState::InitStateValFromLUT(rangeStateVal);

	for (auto v : stateAndVals)
	{
		STATE_TYPE id = SC2DetailedState::GetIDFullyObsVars(v.first);
		if (s_Q_TABLE.find(id) != s_Q_TABLE.end())
			return false;
		s_Q_TABLE[id] = v.second;
	}

	return true;
}

bool SC2Agent::Init_TTable(const std::string & ttableName, int numStateVisitsToContinueSimulation)
{
	std::ifstream ttable(ttableName, std::ios::in);

	if (ttable.fail())
		return false;

	int numVarsInState = SC2DetailedState::s_NUM_VARS_IN_STATE;

	// build q table and init detailed state params
	std::vector<SC2DetailedState::IntRange> rangeStateVal;
	std::vector<std::pair<intVec, doubleVec>> stateAndVals;

	while (!ttable.eof())
	{
		std::string txt;
		std::getline(ttable, txt);
		if (txt.find('[') != std::string::npos)
		{
			STATE_TYPE s1 = 0, s2 = 0;
			intVec s1Vars, s2Vars;
			RetrieveTableState(txt, s1Vars);
			std::string s2Str = txt.substr(txt.find(']') + 1);
			RetrieveTableState(s2Str, s2Vars);

			intVec actionCount;
			s1 = SC2DetailedState::GetIDFullyObsVars(s1Vars);
			
			if (s1Vars.size() != numVarsInState)
				return false;

			if (s2Vars.size() != numVarsInState)
			{
				if (txt.find("win") != std::string::npos)
					s2 = SC2DetailedState::WIN_STATE;
				else if (txt.find("loss") != std::string::npos)
					s2 = SC2DetailedState::LOSS_STATE;
				else if (txt.find("tie") != std::string::npos)
					s2 = SC2DetailedState::TIE_STATE;
				else
					return false;

				RetrieveNumFromStr(s2Str.substr(s2Str.find(' ') + 1), actionCount);
			}
			else
			{
				s2 = SC2DetailedState::GetIDFullyObsVars(s2Vars);
				RetrieveNumFromStr(s2Str.substr(s2Str.find(']') + 1), actionCount);
			}

			if (actionCount.size() != s_NUM_ACTIONS)
				return false;


			if (s_TRANSITION_LUT.find(s1) == s_TRANSITION_LUT.end())
				s_TRANSITION_LUT[s1] = actionResultVec(s_NUM_ACTIONS);

			for (int a = 0; a < s_NUM_ACTIONS; ++a)
			{
				if (actionCount[a] > 0)
					s_TRANSITION_LUT[s1][a].emplace_back(std::make_pair(s2, actionCount[a]));
			}
		}
	}

	// create a vector to reset transition in case of not enough samples
	StateAndProb tieReset(SC2DetailedState::TIE_STATE, 1.0);
	std::vector<StateAndProb> resetVector;
	resetVector.emplace_back(tieReset);

	float sumSizeTable = 0;
	int numNonZero = 0;
	int numBellowThreshold = 0;
	int numZero = 0;
	for (auto & itr : s_TRANSITION_LUT)
	{
		for (int a = 0; a < s_NUM_ACTIONS; ++a)
		{
			int sumVisits = 0;
			for (auto const & v : itr.second[a])
				sumVisits += v.second;
			
			for (auto & v : itr.second[a])
				v.second /= sumVisits;

			if (sumVisits > numStateVisitsToContinueSimulation)
			{
				sumSizeTable += sumVisits;
				numNonZero += 1;
			}
			else
			{
				if (sumVisits == 0)
					++numZero;
				else
					++numBellowThreshold;
				itr.second[a] = resetVector;
			}
			
		}
	}

	std::cout << "num pairs states,a = " << s_TRANSITION_LUT.size() * s_NUM_ACTIONS << " numNonZero Visits = " << numNonZero << ", numBellow threshold Visits = " << numBellowThreshold << ", numZero Visits = " << numZero << "\n";
	std::cout << "avg visits of non zero =" << sumSizeTable / numNonZero << "\n";
	return true;
}

bool SC2Agent::Init_RTable(const std::string & rtableName)
{
	std::ifstream rtable(rtableName, std::ios::in);
	while (!rtable.eof())
	{
		std::string txt;
		std::getline(rtable, txt);

		if (txt.find('[') != std::string::npos)
		{
			intVec state;
			RetrieveTableState(txt, state);
			float num = FindNextNum(txt.substr(txt.find(']')));
			s_REWARD_LUT[SC2DetailedState::GetIDFullyObsVars(state)] = num;
		}
		else if (txt.find("win") != std::string::npos)
		{
			float num = FindNextNum(txt);
			s_REWARD_LUT[SC2DetailedState::WIN_STATE] = num;
		}

		else if (txt.find("loss") != std::string::npos)
		{
			float num = FindNextNum(txt);
			s_REWARD_LUT[SC2DetailedState::LOSS_STATE] = num;
		}
		else if (txt.find("tie") != std::string::npos)
		{
			float num = FindNextNum(txt);
			s_REWARD_LUT[SC2DetailedState::TIE_STATE] = num;
		}
	}
	return true;
}

void SC2Agent::RetrieveTableState(std::string str, intVec & stateVars)
{
	int start = str.find('[') + 1;
	int end = str.find(']');

	std::string onlyNum = "";
	for (int i = start; i < end; i++)
		onlyNum += str[i];

	std::istringstream ss(onlyNum);
	int num = 0;
	while (ss >> num)
		stateVars.emplace_back(num);

	//if (numMembersInState <= 0)
	//	numMembersInState = state.size();
	//else
	//{
	//	if (state.size() != numMembersInState)
	//	{
	//		if (state.size() == 0)
	//		{
	//			if (str.find("win") != std::string::npos)
	//				stateId = SC2DetailedState::WIN_STATE;
	//			else if (str.find("loss") != std::string::npos)
	//				stateId = SC2DetailedState::LOSS_STATE;
	//			else if (str.find("tie") != std::string::npos)
	//				stateId = SC2DetailedState::TIE_STATE;
	//			else
	//				return false;
	//		}
	//		else
	//			return false;
	//	}
	//	else
	//		stateId = SC2DetailedState(state).GetStateId();
	//}
}

void SC2Agent::RetrieveTableValues(std::string str, doubleVec & vals)
{
	int start = str.find(']') + 1;
	std::string onlyNum = "";
	for (int i = start; i < str.length(); i++)
		onlyNum += str[i];

	std::istringstream ss(onlyNum);
	float num = 0;
	while (ss >> num)
		vals.emplace_back(num);
}

void SC2Agent::RetrieveNumFromStr(std::string str, intVec & vals)
{
	std::istringstream ss(str);
	float num = 0;
	while (ss >> num)
		vals.emplace_back(num);
}

float SC2Agent::FindNextNum(const std::string & str)
{
	for (int i = 0; i < str.length(); ++i)
	{
		if (isdigit(str[i]))
		{
			float num = atof(&str.c_str()[i]);
			if (i > 0 && str[i - 1] == '-')
				num *= -1;

			return num;
		}
	}

	return Globals::NEG_INFTY;
}
State * SC2Agent::CreateStartState(std::string type) const
{
	int maxNum = 0;
	int r = 0;
	while (maxNum < s_Q_TABLE.size())
	{
		r += rand();
		maxNum += RAND_MAX;
	}

	int idx = static_cast<int> (r % s_Q_TABLE.size());
	auto it = s_Q_TABLE.begin();
	std::advance(it, idx);
	return new OnlineSolverState(it->first);
}

Belief * SC2Agent::InitialBelief(const State * start, std::string type) const
{
	State * state = Copy(start);

	if (type == "FullyObserved")
	{
		return new FullyObservedBelief(state, this);
	}
	else
	{
		std::vector<State*> particles;
		particles.emplace_back(state);
		return new ParticleBelief(particles, this);
	}
}

bool SC2Agent::Step(State& s, double random, int action, double& reward, OBS_TYPE& obs) const
{
	// termination if self army location is in enemy army base location and power and health of it is 0
	SC2DetailedState s1(s);


	// set next state and current reward
	doubleVec randomNums;
	CreateRandomVec(randomNums, SC2DetailedState::s_NUM_VARS_IN_STATE);
	auto test = s_TRANSITION_LUT[s.state_id];
	auto sPrime = s_TRANSITION_LUT[s.state_id][action];
	int idx = -1;
	while (random > 0)
	{
		++idx;
		random -= sPrime[idx].second;
	}

	s.state_id = sPrime[idx].first;
	if (!SC2DetailedState::IsTerminal(s.state_id))
		int a = 9;

	SC2DetailedState s2(s);

	if (s_REWARD_LUT.find(s.state_id) != s_REWARD_LUT.end())
		reward = s_REWARD_LUT[s.state_id];
	else
		reward = 0;

	// update observation
	obs = s.state_id;

	return SC2DetailedState::IsTerminal(s.state_id);
}


bool SC2Agent::LegalAction(OBS_TYPE observation, int action) const
{
	return true;
}


void SC2Agent::DisplayParameters(std::ofstream & out) const
{
	out << "\n\nSC2Agent Model Details:\n";
	out << "num fully observed variables: " + std::to_string(SC2DetailedState::s_NUM_BITS_FULLY_OBSERVED_PART) + "\n";
}

void SC2Agent::PrintState(const State & s, std::ostream & out) const
{
	SC2DetailedState state(s.state_id);
	out << state.text();
}

void SC2Agent::PrintBelief(const Belief & belief, std::ostream & out) const
{
}

void SC2Agent::PrintObs(const State & state, OBS_TYPE obs, std::ostream & out) const
{
	SC2DetailedState obsState(obs);
	out << obsState.text();
}

void SC2Agent::PrintAction(int action, std::ostream & out) const
{
	out << action;
}

void SC2Agent::InitState() 
{
	// TODO : there is not init state form simulator yet
}

bool SC2Agent::RcvStateIMP(intVec & data, State * s, double & reward, OBS_TYPE & obs) const
{
	int buffer[40];
	int length = s_udpSimulator.Read(reinterpret_cast<char *>(buffer));

	SC2DetailedState observation(buffer);

	obs = observation.GetObsId();
	s->state_id = obs;

	return false;
}

} //end ns despot