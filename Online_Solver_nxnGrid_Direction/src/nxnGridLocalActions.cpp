//#include <string>
//#include <math.h>
//
//#include "..\include\despot\solver\pomcp.h"
//#include "nxnGridLocalActions.h"
//#include "Coordinate.h"
//
//namespace despot 
//{
//
///// changes surrounding a point
//static const int s_numMoves = 8;
//static const int s_lutDirections[s_numMoves][2] = { { 0, 1 }, { 0, -1 }, { 1, 0 }, { -1, 0 }, { 1, 1 }, { 1, -1 }, { -1, 1 }, { -1, -1 } };
//
///// string array for all actions (enum as idx)
//static std::vector<std::string> s_ACTION_STR(nxnGridLocalActions::NUM_BASIC_ACTIONS + 1);
//
///// lut connecting coordinate change to move action
//static std::vector<Coordinate> s_ACTION_CHANGE(nxnGridLocalActions::NUM_BASIC_ACTIONS);
//
///// observation for loss and win (does not important)
//static int OB_LOSS = 0;
//static int OB_WIN = 0;
//
///* =============================================================================
//* inline Functions
//* =============================================================================*/
//
///// return absolute value of a number
//inline int Abs(int num)
//{
//	return num * (num >= 0) - num * (num < 0);
//}
//
///// return min(a,b)
//inline int Min(int a, int b)
//{
//	return a * (a <= b) + b * (b < a);
//}
//
///// return min(a,b)
//inline int Max(int a, int b)
//{
//	return a * (a >= b) + b * (b > a);
//}
//
///// return min(a,b)
//inline int Distance(int a, int b, int gridSize)
//{
//	int xDiff = a % gridSize - b % gridSize;
//	int yDiff = a / gridSize - b / gridSize;
//	return xDiff * xDiff + yDiff * yDiff;
//}
//
//
///* =============================================================================
//* nxnGridGlobalActions Functions
//* =============================================================================*/
//
//nxnGridLocalActions::nxnGridLocalActions(int gridSize, int target, Self_Obj & self, std::vector<locationVec> & objectsInitLoc)
//: nxnGrid(gridSize, target, self, objectsInitLoc)
//{
//	// init vector for string of actions
//	s_ACTION_STR[STAY] = "Stay";
//
//	s_ACTION_STR[NORTH] = "North";
//	s_ACTION_STR[SOUTH] = "South";
//	s_ACTION_STR[WEST] = "West";
//	s_ACTION_STR[EAST] = "East";
//
//	s_ACTION_STR[NORTH_WEST] = "North-West";
//	s_ACTION_STR[NORTH_EAST] = "North-East";
//	s_ACTION_STR[SOUTH_WEST] = "South-West";
//	s_ACTION_STR[SOUTH_EAST] = "South-East";
//
//	s_ACTION_CHANGE[NORTH] = Coordinate(0, -1);
//	s_ACTION_CHANGE[SOUTH] = Coordinate(0, 1);
//	s_ACTION_CHANGE[WEST] = Coordinate(-1, 0);
//	s_ACTION_CHANGE[EAST] = Coordinate(1, 0);
//
//	s_ACTION_CHANGE[NORTH_WEST] = Coordinate(-1, -1);
//	s_ACTION_CHANGE[NORTH_EAST] = Coordinate(1, -1);
//	s_ACTION_CHANGE[SOUTH_WEST] = Coordinate(-1, 1);
//	s_ACTION_CHANGE[SOUTH_EAST] = Coordinate(1, 1);
//
//	s_numBasicActions = NUM_BASIC_ACTIONS;
//	s_numEnemyRelatedActions = 1;
//}
//
//bool nxnGridLocalActions::Step(State& s, double randomSelfAction, int a, OBS_TYPE lastObs, double& reward, OBS_TYPE& obs) const
//{
//	locationVec state;
//	nxnGridState::IdxToState(&s, state);
//	enum ACTION action = static_cast<enum ACTION>(a);
//
//	// drawing more random numbers for each variable
//	double randomSelfObservation = rand();
//	randomSelfObservation /=  RAND_MAX;
//
//	std::vector<double> randomObjectMoves;
//	CreateRandomVec(randomObjectMoves, CountMovingObjects() - 1);
//	
//	std::vector<double> randomEnemiesAttacks;
//	CreateRandomVec(randomEnemiesAttacks, m_enemyVec.size());
//
//	reward = REWARD_STEP;
//
//	// run on all enemies and check if the robot was killed
//	for (int i = 0; i < m_enemyVec.size(); ++i)
//	{
//		if (state[i + 1] != m_gridSize * m_gridSize && CalcIfDead(i, state, randomEnemiesAttacks[i]))
//		{
//			reward = REWARD_LOSS;
//			return true;
//		}
//	}
//
//	// if we are at the target end game with a win
//	if (m_targetIdx == state[0])
//	{	
//		reward = REWARD_WIN;
//		return true;
//	}
//
//	// run on actions
//	if (action > STAY)
//	{
//		if (action < NUM_BASIC_ACTIONS) // action == move
//		{
//			bool isValidMove = MakeMove(state, randomSelfAction, action);
//			reward += REWARD_ILLEGAL_MOVE * (!isValidMove);
//		}
//		else // action = attack
//		{
//			int enemyIdx = action - NUM_BASIC_ACTIONS;
//
//			locationVec observedState;
//			nxnGridState::IdxToState(lastObs, observedState);
//			int obsEnemyLoc = observedState[enemyIdx];
//
//			if (state[enemyIdx] == m_gridSize * m_gridSize | obsEnemyLoc == observedState[0])
//			{
//				reward += REWARD_ILLEGAL_MOVE;
//			}
//			else
//			{
//				Attack(state, obsEnemyLoc, randomSelfAction, reward);
//				for (size_t i = 0; i < m_nonInvolvedVec.size(); ++i)
//				{
//					if (state[i + 1 + m_enemyVec.size()] == m_gridSize * m_gridSize)
//					{
//						reward = REWARD_KILL_NINV;
//						return true;
//					}
//				}
//				reward += REWARD_KILL_ENEMY * (state[enemyIdx] == m_gridSize * m_gridSize);
//			}
//		}
//	}
//	// set next position of the objects on grid
//	SetNextPosition(state, randomObjectMoves);
//	// update observation
//	obs = FindObservation(state, randomSelfObservation);
//	//update state
//	nxnGridState& stateClass = static_cast<nxnGridState&>(s);
//	stateClass.UpdateState(state);
//	return false;
//}
//
//int nxnGridLocalActions::NumActions() const
//{
//	// num basic actions + attack for each enemy
//	return NUM_BASIC_ACTIONS + m_enemyVec.size();
//};
//void nxnGridLocalActions::PrintAction(int action, std::ostream & out) const
//{
//	out << s_ACTION_STR[action] << std::endl;
//}
//
//void nxnGridLocalActions::AddActionsToEnemy()
//{
//	s_ACTION_STR.push_back("Attack enemy #" + std::to_string(m_enemyVec.size()));
//}
//
//void nxnGridLocalActions::AddActionsToShelter()
//{
//}
//
//bool nxnGridLocalActions::MakeMove(locationVec & state, double random, ACTION action) const
//{
//	Coordinate target(state[0] % m_gridSize, state[0] / m_gridSize);
//	target += s_ACTION_CHANGE[action];
//
//	if (target.ValidLocation(m_gridSize))
//	{
//		std::map<int, double> possibleLocs;
//		m_self.GetMovement()->GetPossibleMoves(state[0], m_gridSize, state, possibleLocs, target.GetIdx(m_gridSize));
//
//		int loc = -1;
//		for (auto v : possibleLocs)
//		{
//			loc = v.first;
//			random -= v.second;
//			if (random <= 0.0)
//				break;
//		}
//
//		state[0] = loc;
//	}
//
//	return false;
//}
//void nxnGridLocalActions::Attack(locationVec & state, int target, double random, double & reward) const
//{
//	if (m_self.GetAttack()->InRange(state[0], target, m_gridSize))
//	{
//		locationVec shelters(m_shelters.size());
//		for (size_t i = 0; i < m_shelters.size(); ++i)
//			shelters[i] = m_shelters[i].GetLocation().GetIdx(m_gridSize);
//
//		m_self.AttackOnline(state[0], target, state, shelters, m_gridSize, random);
//		reward += REWARD_FIRE;
//	}
//	else
//		MoveToLocation(state, target, random);
//}
//
//void nxnGridLocalActions::MoveToLocation(locationVec & state, int location, double random) const
//{
//	std::map<int, double> possibleLocations;
//	m_self.GetMovement()->GetPossibleMoves(state[0], m_gridSize, state, possibleLocations, location);
//
//	int loc = -1;
//	for (auto v : possibleLocations)
//	{
//		loc = v.first;
//		random -= v.second;
//		if (random <= 0.0)
//			break;
//	}
//
//	state[0] = loc;
//}
//
//
//bool nxnGridLocalActions::EnemyRelatedAction(int action) const
//{
//	return action >= NUM_BASIC_ACTIONS;
//}
//
//} //end ns despot