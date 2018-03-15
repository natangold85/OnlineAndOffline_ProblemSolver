#include "nxnGridOffline.h"

#include <iostream>		// cout
#include <string>		// string
#include <algorithm>	// algorithms
#include <fstream>      // std::ofstream
#include <sstream>		// ostringstream
#include <iomanip>		// set_percision
#include <memory>		// shared_ptr


static const std::string s_WinState = "Win";
static const std::string s_LossState = "Loss";

/// value in move states for non-valid move
static const int NVALID_MOVE = -1;

/// number of moves of an object from a specific location (stay, num moves, moving toward robot)
static const int s_NUM_OPTIONS_MOVE = NUM_DIRECTIONS + 2;


inline int Distance(int a, int b, int gridSize)
{
	int xDiff = a % gridSize - b % gridSize;
	int yDiff = a / gridSize - b / gridSize;

	return xDiff * xDiff + yDiff * yDiff;
}


long State::State2Idx(State & state, int gridSize)
{
	return -1;
}

State State::Idx2State(long state, int gridSize)
{
	return -1;
}

bool State::operator==(const State & state) const
{
	return m_locations == state.m_locations && m_directions == state.m_directions;
}

bool State::ValidDirections2Locations(int gridSize) const
{
	bool valid = true;
	for (int obj = 0; obj < m_locations.size(); ++obj)
	{
		valid &= Move_Properties::ValidLocation2Direction(m_locations[obj], m_directions[obj], gridSize);
	}
	return valid;
}
//long Observation::Observation2Idx(Observation & observation, int gridSize)
//{
//
//}
//
//Observation Observation::Idx2Observation(long observation, int gridSize)
//{
//
//}

inline int Abs(int x)
{
	return x * (x >= 0) - x * (x < 0);
}

inline void Swap(int & a, int & b)
{
	int tmp = a;
	a = b;
	b = tmp;
}

static double CalcPAUB(std::vector<double> & pVec)
{
	if (pVec.size() == 0)
		return 0.0;
	if (pVec.size() == 1)
		return pVec[0];
	else if (pVec.size() == 2)
		return pVec[0] + pVec[1] - pVec[0] * pVec[1];
	else if (pVec.size() == 3)
		return pVec[0] + pVec[1] + pVec[2] - pVec[0] * pVec[1] - pVec[1] * pVec[2] - pVec[0] * pVec[2] + pVec[0] * pVec[1] * pVec[2];
	// need to continue if i want to allow more than 3 enemies
}

nxnGridOffline::nxnGridOffline(int gridSize, int targetIdx, Self_Obj& self, bool isFullyObs, double discount)
	: m_gridSize(gridSize)
	, m_targetIdx(targetIdx)
	, m_discount(discount)
	, m_isFullyObs(isFullyObs)
	, m_self(self)
	, m_enemyVec()
	, m_nonInvolvedVec()
	, m_shelterVec()
{
}


void nxnGridOffline::UpdateSelf(Self_Obj && obj)
{
	m_self = obj;
}

void nxnGridOffline::AddObj(Attack_Obj&& obj)
{
	m_enemyVec.emplace_back(std::forward<Attack_Obj>(obj));
}

void nxnGridOffline::AddObj(Movable_Obj&& obj)
{
	m_nonInvolvedVec.emplace_back(std::forward<Movable_Obj>(obj));
}

void nxnGridOffline::AddObj(ObjInGrid&& obj)
{
	m_shelterVec.emplace_back(std::forward<ObjInGrid>(obj));
}

void nxnGridOffline::SetLocationSelf(Coordinate & newLocation)
{
	m_self.SetLocation( newLocation );
}

void nxnGridOffline::SetLocationEnemy(Coordinate & newLocation, int idxEnemy)
{

	m_enemyVec[idxEnemy].SetLocation(newLocation);
}

void nxnGridOffline::SetLocationNonInv(Coordinate & newLocation, int idxNonInv)
{
	m_nonInvolvedVec[idxNonInv].SetLocation(newLocation);
}

void nxnGridOffline::SetLocationShelter(Coordinate & newLocation, int idxShelter)
{
	m_shelterVec[idxShelter].SetLocation(newLocation);
}

void nxnGridOffline::SetTarget(int idx)
{
	m_targetIdx = idx;
}

void nxnGridOffline::SetGridSize(int gridSize)
{
	m_gridSize = gridSize;
}

long nxnGridOffline::StateCount2StateIdx(long stateCount) const
{
	// TODO translate state count created by sarsop to state idx


	//State state(CountMovableObj());
	//// add moving objects to state
	//long add1ToCount = stateCount + 1;
	//FindStateCount(state, add1ToCount, 0);
	//
	//// insert shelter location to state
	//if (m_shelterVec.size() > 0)
	//{
	//	Coordinate s = m_shelterVec[0].GetLocation();
	//	state.emplace_back(s.GetIdx(m_gridSize));
	//}

	// return State2Idx(state, m_gridSize);
	return -1;
}

void nxnGridOffline::FindStateCount(State & state, long & count, int currIdx) const
{
	//// stopping condition when finish running on all objects
	//if (state.size() == currIdx)
	//{
	//	--count;
	//}
	//else
	//{
	//	// run on all possible states (without repetitions)
	//	for (int i = 0; i < m_gridSize * m_gridSize & count != 0; ++i)
	//	{
	//		state[currIdx] = i;
	//		if (NoRepeatsIdxLocation(state, currIdx))
	//		{
	//			FindStateCount(state, count, currIdx + 1);
	//		}
	//	}

	//	if (IsEnemy(currIdx) & count != 0)
	//	{
	//		state[currIdx] = m_gridSize * m_gridSize;
	//		FindStateCount(state, count, currIdx + 1);
	//	}
	//}
}

//long nxnGridOffline::State2Idx(State & state, int gridSize)
//{
//	long idxLocation = 0;
//	long idxDirection = 0;
//	// add 1 for num states for dead objects
//	int numStates = gridSize * gridSize + 1;
//	// idx = s[0] * numstates^n + s[1] * numstates^(n - 1) ...  + s[n] * numstates^(0)
//	for (int i = 0; i < state.size(); ++i)
//	{
//		idxLocation *= numStates;
//		idxLocation += state.m_locations[i];
//	}
//
//	return idx;
//}
//
//nxnGridOffline::stateVec nxnGridOffline::Idx2State(long stateIdx)
//{
//	stateVec state(CountMovableObj());
//	int numStates = m_gridSize * m_gridSize + 1;
//
//	// don't treat shelter location
//	stateIdx /= numStates;
//	// running on all varied objects and concluding from the obs num the observed state
//	for (int i = CountMovableObj() - 1; i >= 0; --i)
//	{
//		state[i] = stateIdx % numStates;
//		stateIdx /= numStates;
//	}
//
//	return state;
//}


void nxnGridOffline::ObservationsAndRewards(outFile & fptr)
{
	std::string buffer;
	// calculate observations
	CalcObs(buffer);
	// add rewards
	buffer += "\n\nR: * : * : * : * " + std::to_string(s_REWARD_STEP);
	buffer += "\nR: * : * : " + s_WinState + " : * " + std::to_string(s_REWARD_WIN);
	buffer += "\nR: * : * : " + s_LossState + " : * " + std::to_string(s_REWARD_LOSS);
	// undo reward from transition from endState to another
	buffer += "\nR: * : " + s_WinState + " : " + s_WinState + " : * 0";
	buffer += "\nR: * : " + s_LossState + " : " + s_LossState + " : * 0";

	//save to file
	fptr << buffer;
	if (fptr.bad())
	{
		std::cerr << "Error Writing to file\n"; exit(1);
	}
}

void nxnGridOffline::WriteAllStates(std::string& buffer)
{
	State state(CountMovableObj());
	WriteAllStatesRec(state, 0, buffer);
}
void nxnGridOffline::WriteAllObservations(std::string& buffer)
{
	observation_t obs(CountMovableObj());
	WriteAllObservationsRec(obs, 0, buffer);
}

void nxnGridOffline::CalcStatesAndObs(const char * type, std::string& buffer)
{
	State state(CountMovableObj());
	//CalcS_ORec(state, 0, type, buffer);
}

void nxnGridOffline::WriteAllStatesRec(State& state, int currIdx, std::string & buffer)
{
	// stopping condition when finish running on all objects
	if (state.size() == currIdx)
	{
		// insert state to buffer
		buffer += GetStringState(state) + " ";
	}
	else
	{
		// run on all possible movement direction
		for (int m = 0; m < NUM_DIRECTIONS; ++m)
		{
			state.m_directions[currIdx] = (DIRECTIONS)m;
			// run on all possible locations (without repetitions)
			for (int l = 0; l < m_gridSize * m_gridSize; ++l)
			{
				if (Move_Properties::ValidLocation2Direction(l, (DIRECTIONS)m, m_gridSize))
				{
					state.m_locations[currIdx] = l;
					WriteAllStatesRec(state, currIdx + 1, buffer);
				}
			}
		}

		if (IsEnemy(currIdx))
		{
			state.m_directions[currIdx] = NO_DIRECTION;
			state.m_locations[currIdx] = m_gridSize * m_gridSize;
			WriteAllStatesRec(state, currIdx + 1, buffer);
		}
	}
}

void nxnGridOffline::WriteAllObservationsRec(observation_t & observation, int currIdx, std::string& buffer)
{
	// stopping condition when finish running on all objects
	if (observation.size() == currIdx)
	{
		// insert state to buffer
		buffer += GetStringObservation(observation, m_gridSize) + " ";
	}
	else
	{
		// run on all possible locations (without repetitions)
		for (int l = 0; l < m_gridSize * m_gridSize; ++l)
		{
			observation[currIdx] = l;
			WriteAllObservationsRec(observation, currIdx + 1, buffer);
		}

		if (IsEnemy(currIdx))
		{
			observation[currIdx] = m_gridSize * m_gridSize;
			WriteAllObservationsRec(observation, currIdx + 1, buffer);
		}
		// write non-observed state
		observation[currIdx] = m_gridSize * m_gridSize + 1;
		WriteAllObservationsRec(observation, currIdx + 1, buffer);
	}
}

//void nxnGridOffline::CalcS_ORec(State& state, int currIdx, const char * type, std::string & buffer)
//{
//	// stopping condition when finish running on all objects
//	if (state.size() == currIdx)
//	{
//		// insert state to buffer
//		buffer += GetStringState(state, type) + " ";
//	}
//	else
//	{
//		// run on all possible states (without repetitions)
//		for (int i = 0; i < m_gridSize * m_gridSize; ++i)
//		{
//			state[currIdx] = i;
//			if (NoRepeats(state, currIdx))
//			{
//				CalcS_ORec(state, currIdx + 1, type, buffer);
//			}
//		}
//
//		if (IsEnemy(currIdx))
//		{
//			state[currIdx] = m_gridSize * m_gridSize;
//			CalcS_ORec(state, currIdx + 1, type, buffer);
//		}
//
//		// add non_observed location for observation (state[non observed] = self location)
//		if (*type == 'o' & currIdx > 0)
//		{
//			state[currIdx] = state[0];
//			CalcS_ORec(state, currIdx + 1, type, buffer);
//		}
//	}
//}

//bool nxnGridOffline::NoRepetitionCheckAndCorrect(State & state, std::vector<intVec> & moveStates, intVec & arrOfIdx) const
//{
//	//if any move state equal to the robot location change location to previous location
//	for (int i = 1; i < state.size(); ++i)
//	{
//		if (state.m_locations[i] == state.m_locations[0])
//		{
//			state.m_locations[i] = moveStates[i - 1][0];
//		}
//	}
//
//	//if one of the state equal to another return the possible state to the previous location
//	for (int i = 1; i < state.size(); ++i)
//	{
//		for (int j = 1; j < state.size(); ++j)
//		{
//			if (state.m_locations[i] == state.m_locations[j] && i != j && state.m_locations[j] != m_gridSize * m_gridSize)
//			{
//				if (arrOfIdx[i - 1] == 0)
//				{
//					state.m_locations[j] = moveStates[(j - 1)][0];
//					arrOfIdx[j - 1] = 0;
//					return false;
//				}
//				else
//				{
//					state.m_locations[i] = moveStates[(i - 1)][0];
//					arrOfIdx[i - 1] = 0;
//					return false;
//				}
//			}
//		}
//	}
//
//	return true;
//}

//bool nxnGridOffline::NoRepeatsLocation(State& state, int loc) const
//{
//	for (int i = 0; i < state.size(); ++i)
//	{
//		if (state.m_locations[i] == loc)
//		{
//			return false;
//		}
//	}
//	return true;
//}

//bool nxnGridOffline::NoRepeatsAll(State & state) const
//{
//	for (int i = 0; i < state.size(); ++i)
//	{
//		if (!NoRepeatsIdxLocation(state, i))
//		{
//			return false;
//		}
//	}
//	return true;
//}
//
//bool nxnGridOffline::NoRepeatsIdxLocation(State & state, int idx) const
//{
//	if (state.m_locations[idx] == m_gridSize * m_gridSize)
//	{
//		return true;
//	}
//
//	for (int i = 0; i < idx; ++i)
//	{
//		if (state.m_locations[i] == state.m_locations[idx])
//		{
//			return false;
//		}
//	}
//	return true;
//}

void nxnGridOffline::CalcStartState(std::string& buffer)
{
	State state(CountMovableObj());

	// create init state
	State initState(CountMovableObj());

	// insert direction of movement (all object start static)
	for (int i = 0; i < initState.size(); ++i)
		initState.m_directions[i] = NO_DIRECTION;

	int objIdx = 0;
	initState.m_locations[objIdx] = m_self.GetLocation().GetIdx(m_gridSize);
	++objIdx;
	for (int i = 0; i < m_enemyVec.size(); ++i, ++objIdx)
		initState.m_locations[objIdx] = m_enemyVec[i].GetLocation().GetIdx(m_gridSize);
	for (int i = 0; i < m_nonInvolvedVec.size(); ++i, ++objIdx)
		initState.m_locations[objIdx] = m_nonInvolvedVec[i].GetLocation().GetIdx(m_gridSize);
	
	// calculate probability for each state
	CalcStartStateRec(state, initState, 0, buffer);

	// add probability of win and loss states
	buffer += "0 0 ";
}

void nxnGridOffline::CalcStartStateRec(State & state, State & initState, int currIdx, std::string & buffer)
{
	// stopping condition when finish running on all objects
	if (state.size() == currIdx)
	{
		// insert prob to init to buffer
		if (state == initState)
			buffer += "1 ";
		else
			buffer += "0 ";
	}
	else
	{
		// run on all possible locations and if the location is valid run on the next object
		for (int m = 0; m < NUM_DIRECTIONS; ++m)
		{
			state.m_directions[currIdx] = (DIRECTIONS)m;
			for (int l = 0; l < m_gridSize * m_gridSize; ++l)
			{
				if (Move_Properties::ValidLocation2Direction(l, (DIRECTIONS)m, m_gridSize))
				{
					state.m_locations[currIdx] = l;
					CalcStartStateRec(state, initState, currIdx + 1, buffer);
				}
			}
		}

		// add the option of dead enemy if the object is enbemy
		if (IsEnemy(currIdx))
		{
			state.m_directions[currIdx] = NO_DIRECTION;
			state.m_locations[currIdx] = m_gridSize * m_gridSize;
			CalcStartStateRec(state, initState, currIdx + 1, buffer);
		}
	}
}

void nxnGridOffline::AddTargetPositionRec(State & state, int currIdx, std::string & buffer)
{
	if (currIdx == state.size())
	{
		buffer += "T: * : " + GetStringState(state) + " : " + s_WinState + " 1.0000\n";
	}
	else
	{
		for (int m = 0; m < NUM_DIRECTIONS; ++m)
		{
			state.m_directions[currIdx] = (DIRECTIONS)m;
			for (int l = 0; l < m_gridSize * m_gridSize; ++l)
			{
				if (Move_Properties::ValidLocation2Direction(l, (DIRECTIONS)m, m_gridSize))
				{
					state.m_locations[currIdx] = l;
					AddTargetPositionRec(state, currIdx + 1, buffer);
				}
			}
		}
		if (IsEnemy(currIdx))
		{
			state.m_directions[currIdx] = NO_DIRECTION;
			state.m_locations[currIdx] = m_gridSize * m_gridSize;
			AddTargetPositionRec(state, currIdx + 1, buffer);
		}
	}

}

void nxnGridOffline::CreateShleterVec(intVec & shelters) const
{
	for (auto v : m_shelterVec)
		shelters.push_back(v.GetLocation().GetIdx(m_gridSize));
}

bool nxnGridOffline::IsNonInvDead(State & state) const
{
	bool isDead = false;

	// searching for dead non-involved
	for (size_t i = 0; i < m_nonInvolvedVec.size(); ++i)
		isDead |= (state.m_locations[i + 1 + m_enemyVec.size()] == m_gridSize * m_gridSize);

	return isDead;
}

bool nxnGridOffline::IsNonInvDead(intVec & locations) const
{
	bool isDead = false;

	// searching for dead non-involved
	for (size_t i = 0; i < m_nonInvolvedVec.size(); ++i)
		isDead |= (locations[i + 1 + m_enemyVec.size()] == m_gridSize * m_gridSize);

	return isDead;
}

int nxnGridOffline::FindNearestShelter(int location) const
{
	// assumption : support only 1 shelter
	return m_shelterVec[0].GetLocation().GetIdx(m_gridSize);
}

double nxnGridOffline::PositionSingleState(State & newState, State & currentState, intVec & shelters, double prob, std::string & action, std::string & buffer) const
{
	std::string prefix = "T: " + action + " : " + GetStringState(currentState) + " : ";

	std::vector<double> individualProb2Kill;
	intVec & locations = currentState.m_locations;
	for (int e = 0; e < m_enemyVec.size(); ++e)
	{
		if (locations[e + 1] != m_gridSize * m_gridSize)
		{
			Attack::shootOutcomes result;
			m_enemyVec[e].AttackOffline(locations, e + 1, 0, shelters, m_gridSize, result);
			for (auto v : result)
			{
				if (v.first[0] == m_gridSize * m_gridSize)
					individualProb2Kill.emplace_back(v.second);
			}
		}
	}

	double pToDead = CalcPAUB(individualProb2Kill);
	// if ptoDead is not 0 calculate new whole probability and return pToDead * pLeftProb
	if (pToDead > 0.0)
	{
		double remember = prob;
		prob *= 1 - pToDead;
		pToDead *= remember;
	}

	// insert move states to buffer
	AddMoveStatesRec(newState, prob, 1, prefix, buffer);

	return pToDead;
}

void nxnGridOffline::AddStateToBuffer(std::string& buffer, pairMap & itr, int numLocations)
{
	buffer += std::to_string(itr.first[0]);

	for (int i = 1; i < itr.first.size(); ++i)
	{
		// if object is dead insert dead to buffer
		if (itr.first[i] == numLocations)
		{
			buffer += "xD";
		}
		else
		{
			buffer += "x" + std::to_string(itr.first[i]);
		}
	}
	buffer += " " + Dbl2Str(itr.second) + "\n";
}

std::string nxnGridOffline::GetStringState(State & state) const
{
	std::string currentState = "S" + std::to_string(state.m_locations[0]) + "_" + std::to_string(state.m_directions[0]);

	for (int i = 1; i < state.size(); ++i)
	{
		if (state.m_locations[i] != m_gridSize * m_gridSize)
		{
			currentState += "x" + std::to_string(state.m_locations[i]) + "_" + std::to_string(state.m_directions[i]);
		}
		else
		{
			currentState += "xD";
		}
	}
	return std::move(currentState);
}

std::string nxnGridOffline::GetStringObservation(observation_t & obs, int gridSize)
{
	std::string currentState = "O" + std::to_string(obs[0]);

	for (int i = 1; i < obs.size(); ++i)
	{
		if (obs[i] < gridSize * gridSize)
			currentState += "x" + std::to_string(obs[i]);
		else if (obs[i] == gridSize * gridSize)
			currentState += "xD";
		else // non observed
			currentState += "xN";
	}
	return std::move(currentState);
}

bool nxnGridOffline::SearchForShelter(int location) const
{
	for (auto v : m_shelterVec)
	{
		if (v.GetLocation().GetIdx(m_gridSize) == location)
		{
			return true;
		}
	}
	return false;
}

bool nxnGridOffline::InBoundary(int state, int advanceFactor, int gridSize)
{
	int x = state % gridSize;
	int y = state / gridSize;
	int xdiff = advanceFactor % gridSize;
	int ydiff = advanceFactor / gridSize;

	return ((x + xdiff) >= 0) & ((x + xdiff) < gridSize) & ((y + ydiff) >= 0) & ((y + ydiff) < gridSize);
}

bool nxnGridOffline::InBoundary(int location, int xChange, int yChange, int gridSize)
{
	int x = location % gridSize + xChange;
	int y = location / gridSize + yChange;
	// return true if the location is in boundary
	return x >= 0 & x < gridSize & y >= 0 & y < gridSize;
}

void nxnGridOffline::AddMoveStatesRec(State & state, double prob, int currIdx, std::string & prefix, std::string & buffer) const
{
	if (currIdx == state.size())
	{
		buffer += prefix + GetStringState(state) + " " + Dbl2Str(prob) + "\n";
	}
	else
	{	
		// if object is dead there is no move state possible
		if (state.m_locations[currIdx] == m_gridSize * m_gridSize)
		{
			AddMoveStatesRec(state, prob, currIdx + 1, prefix, buffer);
			return;
		}

		// get movement possibles
		const std::shared_ptr<Move_Properties> movement = GetObjMovement(currIdx);
		
		std::map<int, double> possibleLocs;
		movement->GetPossibleLocations(state.m_locations[currIdx], state.m_directions[currIdx], m_gridSize, possibleLocs);

		int rememberLoc = state.m_locations[currIdx];
		for (auto move : possibleLocs)
		{
			state.m_locations[currIdx] = move.first;
			std::map<DIRECTIONS, double> possibleDirections;
			movement->GetPossibleDirections(move.first, state.m_directions[currIdx], state.m_locations[0], m_gridSize, possibleDirections);
			
			DIRECTIONS rememberDir = state.m_directions[currIdx];
			for (auto dir : possibleDirections)
			{
				state.m_directions[currIdx] = dir.first;
				AddMoveStatesRec(state, prob * dir.second * move.second, currIdx + 1, prefix, buffer);
			}
			state.m_directions[currIdx] = rememberDir;
		}

		state.m_locations[currIdx] = rememberLoc;
	}
}





State nxnGridOffline::MoveToIdx(State state, std::vector<intVec> & moveStates, intVec arrOfIdx) const
{
	// insert the current moveState to state
	for (int i = 0; i < state.size() - 1; ++i)
	{
		int currState = moveStates[i][arrOfIdx[i]];
		state.m_locations[i + 1] = currState * (NVALID_MOVE != currState) + moveStates[i][0] * (NVALID_MOVE == currState);
	}

	// correct the state vec in case of repetitions
	//while (!NoRepetitionCheckAndCorrect(state, moveStates, arrOfIdx));

	return state;
}


void nxnGridOffline::CalcObs(std::string& buffer)
{
	State state(CountMovableObj());
	CalcObsRec(state, 0, buffer);

	buffer += "\nO: * : " + s_WinState + " : " "oWin 1.0";
	buffer += "\nO: * : " + s_LossState + " : " "oLoss 1.0";
}
void nxnGridOffline::CalcObsRec(State& state, int currIdx, std::string & buffer)
{
	if (currIdx == state.size())
	{
		// arriving here when state is initialize to a state. run on this state calculation of observations
		if (m_isFullyObs)
			buffer += "O: * : " + GetStringState(state) + " : " + GetStringObservation(state.m_locations, m_gridSize) + " 1\n";
		else
			CalcObsSingleState(state, buffer);

		buffer += "\n";
	}
	else
	{
		// run on all possible locations. if there location is valid call for the calculation of the next object
		for (int m = 0; m < NUM_DIRECTIONS; ++m)
		{
			state.m_directions[currIdx] = (DIRECTIONS)m;
			for (int l = 0; l < m_gridSize * m_gridSize; ++l)
			{
				if (Move_Properties::ValidLocation2Direction(l, (DIRECTIONS)m, m_gridSize))
				{
					state.m_locations[currIdx] = l;
					CalcObsRec(state, currIdx + 1, buffer);
				}
			}
		}
		if (IsEnemy(currIdx))
		{
			state.m_directions[currIdx] = NO_DIRECTION;
			state.m_locations[currIdx] = m_gridSize * m_gridSize;
			CalcObsRec(state, currIdx + 1, buffer);
		}
	}

}

void nxnGridOffline::CalcObsSingleState(State & state, std::string& buffer)
{
	std::string prefix = "O: * : " + GetStringState(state) + " : ";
	
	observation_t observation(state.size());
	mapProb pMap;

	CalcObsMapRec(observation, state, pMap, 1.0, 1);

	// add observations to buffer
	int gridSize = m_gridSize;
	std::for_each(pMap.begin(), pMap.end(), [&buffer, &prefix, gridSize](pairMap itr)
	{	buffer += prefix + GetStringObservation(itr.first, gridSize) + " " + Dbl2Str(itr.second) + "\n"; });
}

void nxnGridOffline::CalcObsMapRec(observation_t & observedState, State & state, mapProb& pMap, double pCurr, int currObj)
{
	// stopping condition: arriving to the end of the state vec
	if (currObj == state.size())
	{
		// insert p to map
		pMap[observedState] += pCurr;
	}
	else
	{
		intVec observableLocations;
		m_self.GetObservation()->InitObsAvailableLocations(state.m_locations[0], state.m_locations[currObj], state.m_locations, m_gridSize, observableLocations);
		for (auto obsLoc : observableLocations)
		{
			observedState[currObj] = obsLoc;
			double pObs = m_self.GetObservation()->GetProbObservation(state.m_locations[0], state.m_locations[currObj], m_gridSize, obsLoc);
			CalcObsMapRec(observedState, state, pMap, pCurr * pObs, currObj + 1);
		}
	}
}

inline bool nxnGridOffline::IsEnemy(int idx) const
{
	return idx - 1 < m_enemyVec.size();
}

std::shared_ptr<Move_Properties> nxnGridOffline::GetObjMovement(int objIdx) const
{
	if (objIdx == 0)
		return m_self.GetMovement();

	--objIdx;
	if (objIdx < m_enemyVec.size())
		return m_enemyVec[objIdx].GetMovement();

	objIdx -= m_enemyVec.size();
	if (objIdx < m_nonInvolvedVec.size())
		return m_nonInvolvedVec[objIdx].GetMovement();
}

inline int nxnGridOffline::CountMovableObj() const
{
	return 1 + m_enemyVec.size() + m_nonInvolvedVec.size();
}

int nxnGridOffline::CountEnemies() const
{
	return m_enemyVec.size();
}

int nxnGridOffline::CountNInv() const
{
	return m_nonInvolvedVec.size();
}

int nxnGridOffline::CountShelters() const
{
	return m_shelterVec.size();
}

int nxnGridOffline::GetGridSize() const
{
	return m_gridSize;
}

inline bool nxnGridOffline::AnyDead(State& state) const
{
	for (int i = 0; i < m_enemyVec.size(); ++i)
	{
		if (state.m_locations[i + 1] == m_gridSize * m_gridSize)
		{
			return true;
		}
	}
	return false;
}

// translate to string with higher precision
std::string nxnGridOffline::Dbl2Str(double d)
{
	std::stringstream ss;
	ss << std::fixed << std::setprecision(10) << d;              //convert double to string w fixed notation, hi precision
	std::string s = ss.str();                                    //output to std::string
	s.erase(s.find_last_not_of('0') + 1, std::string::npos);     //remove trailing 000s    (123.1200 => 123.12,  123.000 => 123.)

	double a = strtod(s.c_str(), NULL);
	return (s[s.size() - 1] == '.') ? s.substr(0, s.size() - 1) : s; //remove dangling decimal (123. => 123)
}


std::ostream& operator<<(std::ostream& o, const nxnGridOffline& pomdp)
{
	o << "\ngridSize : " << pomdp.m_gridSize <<
		"\nnumber of enemies : " << pomdp.m_enemyVec.size() <<
		"\nnumber of non involved : " << pomdp.m_nonInvolvedVec.size() <<
		"\nnumber of shelters : " << pomdp.m_shelterVec.size() << "\n";
	return o;
}
