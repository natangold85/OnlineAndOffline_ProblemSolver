#include <string>
#include <math.h>
#include <limits.h>

#include "nxnGridGlobalActions.h"
#include "Coordinate.h"

namespace despot 
{

bool nxnGridGlobalActions::s_isMoveFromEnemy = false;

/* =============================================================================
* nxnGridLocalActions Functions
* =============================================================================*/

nxnGridGlobalActions::nxnGridGlobalActions(int gridSize, int target, Self_Obj & self, std::vector<intVec> & objectsInitLoc, bool isMoveFromEnemyExist)
: nxnGrid(gridSize, target, self, objectsInitLoc)
{
	// init vector for string of actions
	s_actionsStr.clear();
	s_actionsStr.emplace_back("Move To Target");
	s_isMoveFromEnemy = isMoveFromEnemyExist;
}

bool nxnGridGlobalActions::Step(State& s, double randomSelfAction, int action, double & reward, OBS_TYPE& obs) const
{
	nxnGridDetailedState state(s);

	// drawing more random numbers for each variable
	double randomSelfObservation = RandomNum();

	std::vector<double> randomObjectMoves;
	CreateRandomVec(randomObjectMoves, CountMovingObjects() - 1);

	std::vector<double> randomEnemiesAttacks;
	CreateRandomVec(randomEnemiesAttacks, m_enemyVec.size());

	reward = REWARD_STEP;

	// run on all enemies and check if the robot was killed
	for (int i = 0; i < m_enemyVec.size(); ++i)
	{
		if (!Attack::IsDead(state[i + 1], m_gridSize) && CalcIfKilledByEnemy(state, i, randomEnemiesAttacks[i]))
		{
			reward = REWARD_LOSS;
			return true;
		}
	}

	// if we are at the target end game with a win
	if (m_targetIdx == state[0])
	{
		reward = REWARD_WIN;
		return true;
	}

	// run on actions
	if (action == MOVE_TO_TARGET)
	{
		MoveToTarget(state, randomSelfAction);
	}
	else if (action == MOVE_TO_SHELTER & m_shelters.size() > 0)
	{
		MoveToShelter(state, randomSelfAction);
	}
	else // actions related to enemies
	{
		int enemyAction = (action - NumBasicActions()) % NumEnemyActions();
		int enemyIdx = ((action - NumBasicActions()) / NumEnemyActions()) + 1;

		int realEnemyLoc = state[enemyIdx];

		// if enemy is dead or non observed move is illegal
		if (Attack::IsDead(realEnemyLoc, m_gridSize) || !state.IsEnemyObserved(enemyIdx - 1))
			reward += REWARD_ILLEGAL_MOVE;
		else if (enemyAction == ATTACK)
		{
			Attack(state, realEnemyLoc, randomSelfAction, reward);

			if (state.IsNonInvDead(m_gridSize))
			{
				reward = REWARD_KILL_NINV;
				return true;
			}

			// TODO : needs to reward also if non-targeted enemy is killed
			reward += REWARD_KILL_ENEMY * Attack::IsDead(state[enemyIdx], m_gridSize);
		}
		else // action = movefromenemy
			MoveFromEnemy(state, realEnemyLoc, randomSelfAction);
	}


	// set next position of the objects on grid
	SetNextPosition(state, randomObjectMoves);

	// update observation
	obs = UpdateObservation(state, randomSelfObservation);
	//update state
	s.state_id = state.GetStateId();
	return false;
}

int nxnGridGlobalActions::NumBasicActions() const
{
	return 1 + (m_shelters.size() > 0);
}

int nxnGridGlobalActions::NumEnemyActions() const
{
	return 1 + s_isMoveFromEnemy;
}

int nxnGridGlobalActions::NumActions() const
{
	// for any enemy add attack and move from enemy
	return NumBasicActions() + m_enemyVec.size() * NumEnemyActions();
};

ValuedAction nxnGridGlobalActions::GetMinRewardAction() const
{
	return ValuedAction(rand() % NumActions(), -10.0);
}

void nxnGridGlobalActions::AddActionsToEnemy()
{
	int enemyNum = m_enemyVec.size();
	s_actionsStr.push_back("Attack enemy #" + std::to_string(enemyNum));
	if (s_isMoveFromEnemy)
		s_actionsStr.push_back("Move from enemy #" + std::to_string(enemyNum));
}

void nxnGridGlobalActions::AddActionsToShelter()
{
	// insert single move to shelter after first action (move to target)
	if (m_shelters.size() == 1)
		s_actionsStr.insert(s_actionsStr.begin() + 1, "Move to shelter");
}

void nxnGridGlobalActions::MoveToTarget(nxnGridDetailedState & state, double random) const
{
	MoveToLocation(state, m_targetIdx, random);
}

void nxnGridGlobalActions::MoveToShelter(nxnGridDetailedState & state, double random) const
{
	// go to the nearest shelter
	int shelterLoc = NearestShelter(state[0]);
	
	if (shelterLoc != state[0])
		MoveToLocation(state, shelterLoc, random);
}

void nxnGridGlobalActions::Attack(nxnGridDetailedState & state, int target, double random, double & reward) const
{
	if (m_self.GetAttack()->InRange(state[0], target, m_gridSize))
	{
		intVec shelters(GetSheltersVec());

		m_self.GetAttack()->AttackOnline(state.LocationVec(), 0, target, shelters, m_gridSize, random);
		reward += REWARD_FIRE;
	}
	else
		MoveToLocation(state, target, random);
}

void nxnGridGlobalActions::MoveFromEnemy(nxnGridDetailedState & state, int obsEnemyLoc, double random) const
{
	Coordinate enemy(obsEnemyLoc % m_gridSize, obsEnemyLoc / m_gridSize);
	int newLoc = MoveFromLocation(state, enemy);

	if (newLoc == state[0])
		return;

	std::map<int, double> possibleLocations;
	intVec nonValidLocations;
	GetNonValidLocations(state, 0, nonValidLocations);
	m_self.GetMovement()->GetPossibleMoves(state[0], m_gridSize, nonValidLocations, possibleLocations, newLoc);

	int loc = -1;
	for (auto v : possibleLocations)
	{
		loc = v.first;
		random -= v.second;
		if (random <= 0.0)
			break;
	}

	state[0] = loc;
}

void nxnGridGlobalActions::MoveToLocation(nxnGridDetailedState & state, int location, double random) const
{
	std::map<int, double> possibleLocations;
	intVec nonValidLocations;
	GetNonValidLocations(state, 0, nonValidLocations);
	m_self.GetMovement()->GetPossibleMoves(state[0], m_gridSize, nonValidLocations, possibleLocations, location);

	int loc = -1;
	for (auto v : possibleLocations)
	{
		loc = v.first;
		random -= v.second;
		if (random <= 0.0)
			break;
	}

	state[0] = loc;
}

int nxnGridGlobalActions::MoveFromLocation(nxnGridDetailedState & state, Coordinate & goFrom) const
{
	Coordinate self(state[0] % m_gridSize, state[0] / m_gridSize);

	int maxLocation = -1;
	int maxDist = INT_MIN;

	// run on all possible move locations and find the farthest point from goFrom
	for (auto loc : Move_Properties::s_directionsLUT)
	{
		Coordinate move(self);
		move += loc;

		// if move is on grid calculate distance
		if (move.ValidLocation(m_gridSize))
		{
			int currDist = goFrom.Distance(move);
			if (currDist > maxDist)
			{
				maxLocation = move.GetIdx(m_gridSize);
				maxDist = currDist;
			}
		}
	}

	return maxLocation;
}

int nxnGridGlobalActions::NearestShelter(int loc) const
{
	Coordinate location(loc % m_gridSize, loc / m_gridSize);
	int minDist = INT_MAX;
	int nearestShelter = -1;

	for (int s = 0; s < m_shelters.size(); ++s)
	{
		auto sLoc = m_shelters[s].GetLocation();
		int currDist = location.Distance(sLoc);
		if (currDist < minDist)
		{
			currDist = minDist;
			nearestShelter = sLoc.GetIdx(m_gridSize);
		}
	}

	return nearestShelter;
}

int nxnGridGlobalActions::EnemyRelatedActionIdx(int action) const
{
	return ((action - NumBasicActions()) / NumEnemyActions());
}

bool nxnGridGlobalActions::EnemyRelatedAction(int action) const
{
	return action >= NumBasicActions();
}

bool nxnGridGlobalActions::LegalAction(OBS_TYPE observedState, int action) const
{	
	bool legalAction = false;

	nxnGridDetailedState obsState(observedState);
	// if not related to enemy idx will be negative
	int enemyIdxAction = (action - NumBasicActions()) / NumEnemyActions();

	if (enemyIdxAction < 0) 
		legalAction = true;
	else if (!Attack::IsDead(obsState[enemyIdxAction], m_gridSize) && !Observation::IsNonObserved(obsState[enemyIdxAction], m_gridSize))
		legalAction = true;

	return legalAction;
}

int nxnGridGlobalActions::randLegalAction(OBS_TYPE observation) const
{
	boolVec obsEnemies;
	nxnGridDetailedState::GetEnemyObservedVec(observation, obsEnemies);
	int action;
	do
	{
		action = rand() % NumActions();
	} while (action > NumBasicActions() && !obsEnemies[EnemyRelatedActionIdx(action)]);

	return action;
}

} //end ns despot