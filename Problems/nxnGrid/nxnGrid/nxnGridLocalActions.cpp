#include <string>
#include <math.h>

#include "nxnGridLocalActions.h"
#include "Coordinate.h"

namespace despot 
{

/* =============================================================================
* nxnGridLocalActions Functions
* =============================================================================*/

nxnGridLocalActions::nxnGridLocalActions(int gridSize, int target, Self_Obj & self, std::vector<intVec> & objectsInitLoc)
: nxnGrid(gridSize, target, self, objectsInitLoc)
{
	s_actionsStr = Move_Properties::s_directionNamesLUT;
}

bool nxnGridLocalActions::Step(State& s, double randomSelfAction, int a, OBS_TYPE lastObs, double& reward, OBS_TYPE& obs) const
{
	nxnGridDetailedState state(s);
	enum ACTION action = static_cast<enum ACTION>(a);

	// drawing more random numbers for each variable
	double randomSelfObservation = rand();
	randomSelfObservation /=  RAND_MAX;

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
	if (action < NO_DIRECTION)	// action == move
	{
		bool isValidMove = MakeMove(state, randomSelfAction, action);
		reward += REWARD_ILLEGAL_MOVE * (!isValidMove);
	}
	else if (action > NUM_BASIC_ACTIONS)// action = attack
	{
		int enemyIdx = action - NUM_BASIC_ACTIONS;

		int realEnemyLoc = state[enemyIdx];
		int obsEnemyLoc = nxnGridDetailedState::GetObservedObjLocation(lastObs, enemyIdx);
			
		if (Attack::IsDead(realEnemyLoc, m_gridSize) || Observation::IsNonObserved(obsEnemyLoc, m_gridSize))
			reward += REWARD_ILLEGAL_MOVE;
		else
		{
			Attack(state, obsEnemyLoc, randomSelfAction, reward);
			if (state.IsNonInvDead(m_gridSize))
			{
				reward = REWARD_KILL_NINV;
				return true;
			}

			reward += REWARD_KILL_ENEMY * Attack::IsDead(state[enemyIdx], m_gridSize);
		}
	}

	// set next position of the objects on grid
	SetNextPosition(state, randomObjectMoves);
	// update observation
	obs = FindObservation(state, randomSelfObservation);
	//update state
	s.state_id = state.GetStateId();

	return false;
}

int nxnGridLocalActions::NumActions() const
{
	// num basic actions + attack for each enemy
	return NUM_BASIC_ACTIONS + m_enemyVec.size();
};

void nxnGridLocalActions::AddActionsToEnemy()
{
	s_actionsStr.push_back("Attack enemy #" + std::to_string(m_enemyVec.size()));
}

void nxnGridLocalActions::AddActionsToShelter()
{
}

bool nxnGridLocalActions::MakeMove(nxnGridDetailedState & state, double random, ACTION action) const
{
	Coordinate target(state[0] % m_gridSize, state[0] / m_gridSize);
	target += Move_Properties::s_directionsLUT[action];

	if (target.ValidLocation(m_gridSize))
	{
		std::map<int, double> possibleLocs;
		intVec nonValidLocations;
		GetNonValidLocations(state, 0, nonValidLocations);
		m_self.GetMovement()->GetPossibleMoves(state[0], m_gridSize, nonValidLocations, possibleLocs, target.GetIdx(m_gridSize));

		int loc = -1;
		for (auto v : possibleLocs)
		{
			loc = v.first;
			random -= v.second;
			if (random <= 0.0)
				break;
		}

		state[0] = loc;
	}

	return false;
}
void nxnGridLocalActions::Attack(nxnGridDetailedState & state, int target, double random, double & reward) const
{
	if (m_self.GetAttack()->InRange(state[0], target, m_gridSize))
	{
		intVec shelters(GetSheltersVec());

		m_self.GetAttack()->AttackOnline(state.LocationVec(), state[0], target, shelters, m_gridSize, random);
		reward += REWARD_FIRE;
	}
	else
		MoveToLocation(state, target, random);
}

void nxnGridLocalActions::MoveToLocation(nxnGridDetailedState & state, int location, double random) const
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

int nxnGridLocalActions::EnemyRelatedActionIdx(int action) const
{
	return action - NUM_BASIC_ACTIONS;
}

bool nxnGridLocalActions::EnemyRelatedAction(int action) const
{
	return action >= NUM_BASIC_ACTIONS;
}


bool nxnGridLocalActions::LegalAction(OBS_TYPE observedState, int action) const
{
	bool legalAction = false;

	// if not related to enemy idx will be negative
	nxnGridDetailedState obsState(observedState);
	int enemyIdxAction = action - NUM_BASIC_ACTIONS;

	if (enemyIdxAction < 0)
	{
		Coordinate selfLoc(obsState[0] % m_gridSize, obsState[0] / m_gridSize);
		selfLoc += Move_Properties::s_directionsLUT[action];
		if (selfLoc.ValidLocation(m_gridSize))
			legalAction = true;
	}
	if (!Attack::IsDead(obsState[enemyIdxAction], m_gridSize) && !Observation::IsNonObserved(obsState[enemyIdxAction], m_gridSize))
		legalAction = true;

	return action;
}

} //end ns despot