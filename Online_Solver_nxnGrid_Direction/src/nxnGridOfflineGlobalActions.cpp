#include "nxnGridOfflineGlobalActions.h"

#include <iostream>
static const std::string s_WinState = "Win";
static const std::string s_LossState = "Loss";

/// value in move states for non-valid move
static const int NVALID_MOVE = -1;
/// all directions which are also all possible moves
static const int s_NUM_DIRECTIONS = 8;
/// lut for all direction changes
static int s_lutDirections[s_NUM_DIRECTIONS][2] = { { 0, 1 }, { 0, -1 }, { 1, 0 }, { -1, 0 }, { 1, 1 }, { 1, -1 }, { -1, 1 }, { -1, -1 } };

inline int Distance(int a, int b, int gridSize)
{
	int xDiff = a % gridSize - b % gridSize;
	int yDiff = a / gridSize - b / gridSize;

	return xDiff * xDiff + yDiff * yDiff;
}

inline int Abs(int x)
{
	return x * (x >= 0) - x * (x < 0);
}

nxnGridOfflineGlobalActions::nxnGridOfflineGlobalActions(int gridSize, int targetIdx, Self_Obj& self, bool isFullyObs, double discount)
: nxnGridOffline(gridSize, targetIdx, self, isFullyObs, discount)
{
}

void nxnGridOfflineGlobalActions::SaveInPomdpFormat(outFile & fptr)
{
	std::string buffer("");
	//add comments and init lines(state observations etc.) to file
	CommentsAndInitLines(fptr);

	// add position with and without moving of the robot
	AddAllActions(fptr);

	// add observations and rewards
	ObservationsAndRewards(fptr);
}

int nxnGridOfflineGlobalActions::GetNumActions() const
{
	return 1 + (m_shelterVec.size() != 0) + m_enemyVec.size() * NUM_ENEMY_ACTIONS;
}

void nxnGridOfflineGlobalActions::CommentsAndInitLines(outFile & file)
{
	std::string buffer;

	// add comments
	buffer += "# pomdp file:\n";
	buffer += "# grid size: " + std::to_string(m_gridSize) + "  target idx: " + std::to_string(m_targetIdx);
	buffer += "\n# SELF:\n# self initial location: " + std::to_string(m_self.GetLocation().GetIdx(m_gridSize));
	buffer += "\n# with move properties: " + m_self.GetMovement()->String();
	buffer += "\n# with attack: " + m_self.GetAttack()->String();
	buffer += "\n# with observation: " + m_self.GetObservation()->String();
	buffer += "\n\n# ENEMIES:";
	for (auto v : m_enemyVec)
	{
		buffer += "\n\n# enemy initial location: " + std::to_string(v.GetLocation().GetIdx(m_gridSize));
		buffer += "\n# with move properties: " + v.GetMovement()->String();
		buffer += "\n# with attack: " + v.GetAttack()->String();
	}

	buffer += "\n\n# NON-INVOLVED:";
	for (auto v : m_nonInvolvedVec)
	{
		buffer += "\n\n# non- involved initial location: " + std::to_string(v.GetLocation().GetIdx(m_gridSize));
		buffer += "\n# with move properties: " + v.GetMovement()->String();
	}

	buffer += "\n\n# SHELTERS:";
	for (auto v : m_shelterVec)
	{
		buffer += "\n# shelter location: " + std::to_string(v.GetLocation().GetIdx(m_gridSize));
	}

	// add init lines
	buffer += "\n\ndiscount: " + std::to_string(m_discount);
	buffer += "\nvalues: reward\nstates: ";

	// add states names
	WriteAllStates(buffer);
	buffer += s_WinState + " " + s_LossState + "\n";
	buffer += "actions: MoveToTarget ";
	if (m_shelterVec.size() > 0)
		buffer += "MoveToShelter";

	for (int e = 0; e < m_enemyVec.size(); ++e)
	{
		buffer += " Attack" + std::to_string(e + 1);
		//buffer += " MoveFromEnemy" + std::to_string(e + 1);
	}
	buffer += "\n";

	// add observations names
	buffer += "observations: ";
	WriteAllObservations(buffer);
	buffer += "o" + s_WinState + " o" + s_LossState + "\n";
	buffer += "\n\nstart: \n";

	// add start states probability
	CalcStartState(buffer);
	buffer += "\n\n";
	//save to file
	file << buffer;
	if (file.bad()) 
	{ std::cerr << "Error Writing to file\n"; exit(1); }
}

void nxnGridOfflineGlobalActions::AddAllActions(outFile & fptr)
{
	std::string buffer;

	// add move to win state from states when the robot is in target
	State state(CountMovableObj());

	state.m_locations[0] = m_targetIdx;
	for(int m = 0; m < NUM_DIRECTIONS; ++m)
	{
		if (Move_Properties::ValidLocation2Direction(m_targetIdx, (DIRECTIONS)m, m_gridSize))
		{
			state.m_directions[0] = (DIRECTIONS)m;
			AddTargetPositionRec(state, 1, buffer);
			buffer += "\n";
		}
	}

	// add actions for all states
	AddActionsAllStates(buffer);

	//save to file
	fptr << buffer;
	if (fptr.bad())
	{ std::cerr << "Error Writing to file\n"; exit(1); }
}

void nxnGridOfflineGlobalActions::AddActionsAllStates(std::string & buffer)
{
	State state(CountMovableObj());
	intVec shelters;
	CreateShleterVec(shelters);

	AddActionsRec(state, shelters, 0, buffer);

	buffer += "\n\nT: * : " + s_WinState + " : " + s_WinState + " 1";
	buffer += "\nT: * : " + s_LossState + " : " + s_LossState + " 1\n\n";
}

void nxnGridOfflineGlobalActions::AddActionsRec(State & state, intVec & shelters, int currObj, std::string & buffer)
{
	if (currObj == state.size())
	{
		// if we are allready in the target idx do nothing
		if (state.m_locations[0] == m_targetIdx)
			return;

		/// return state given stateIdx
		AddMoveToTarget(state, shelters, buffer);

		// if exist shelter add move to shelter action
		if (m_shelterVec.size() > 0)
			AddMoveToShelter(state, shelters, buffer);

		for (int e = 0; e < m_enemyVec.size(); ++e)
		{
			AddAttack(state, e + 1, shelters, buffer);
			//AddMoveFromEnemy(state, e, shelters, buffer);
		}
	}
	else
	{
		for (int m = 0; m < NUM_DIRECTIONS; ++m)
		{
			state.m_directions[currObj] = (DIRECTIONS)m;
			for (int l = 0; l < m_gridSize * m_gridSize; ++l)
			{
				if (Move_Properties::ValidLocation2Direction(l, (DIRECTIONS)m, m_gridSize))
				{
					state.m_locations[currObj] = l;
					AddActionsRec(state, shelters, currObj + 1, buffer);
				}
			}
		}
		// if the object is enemy add cases of when the enemy is dead
		if (IsEnemy(currObj))
		{
			state.m_directions[currObj] = NO_DIRECTION;
			state.m_locations[currObj] = m_gridSize * m_gridSize;
			AddActionsRec(state, shelters, currObj + 1, buffer);
		}
	}
}

void nxnGridOfflineGlobalActions::AddAttack(State & state, int objIdx, intVec & shelters, std::string & buffer) const
{
	std::string action = "Attack" + std::to_string(objIdx);
	
	double pLoss = 0.0;

	// if enemy dead do nothing
	if (state.m_locations[objIdx] == m_gridSize * m_gridSize)
	{
		pLoss = PositionSingleState(state, state, shelters, 1.0, action, buffer);
		if (pLoss > 0.0)
			buffer += "T: " + action + " : " + GetStringState(state) + " : " + s_LossState + " " + Dbl2Str(pLoss) + "\n";
		buffer += "\n";
		return;
	}

	Coordinate self(state.m_locations[0] % m_gridSize, state.m_locations[0] / m_gridSize);
	Coordinate enemy(state.m_locations[objIdx] % m_gridSize, state.m_locations[objIdx] / m_gridSize);

	if (m_self.GetAttack()->GetRange() >= self.RealDistance(enemy))
	{
		std::vector<std::pair<intVec, double>> shootOutcomes;

		// creating a vecotr of shoot outcomes
		m_self.AttackOffline(state.m_locations, 0, objIdx, shelters, m_gridSize, shootOutcomes);

		for (auto v : shootOutcomes)
		{
			// if non-involved dead transfer to loss state else calculate move states
			if (!IsNonInvDead(v.first))
			{
				State newState(state);
				newState.m_locations = v.first;
				// if enemy is dead make him wo direction
				MakeDeadEnemyStatic(newState);
				pLoss += PositionSingleState(newState, state, shelters, v.second, action, buffer);
			}
			else
				pLoss += v.second;
		}
	}
	else
		pLoss += MoveToLocation(state, shelters, state.m_locations[objIdx], action, buffer);

	if (pLoss > 0.0)
		buffer += "T: " + action + " : " + GetStringState(state) + " : " + s_LossState + " " + Dbl2Str(pLoss) + "\n";

	buffer += "\n";
}

void nxnGridOfflineGlobalActions::AddMoveToTarget(State & state, intVec & shelters, std::string & buffer) const
{
	std::string action = "MoveToTarget";
	
	double pLoss = 0.0;

	std::vector<std::pair<int, double>> moveOutComes;
	pLoss += MoveToLocation(state, shelters, m_targetIdx, action, buffer);

	if (pLoss > 0.0)
		buffer += "T: " + action + " : " + GetStringState(state) + " : " + s_LossState + " " + Dbl2Str(pLoss) + "\n";

	buffer += "\n";
}

void nxnGridOfflineGlobalActions::AddMoveToShelter(State & state, intVec & shelters, std::string & buffer) const
{
	std::string action = "MoveToShelter";
	
	double pLoss = 0.0;

	if (m_shelterVec.size() > 0 && !SearchForShelter(state.m_locations[0]))
	{
		int shelterLoc = FindNearestShelter(state.m_locations[0]);
		pLoss += MoveToLocation(state, shelters, shelterLoc, action, buffer);
	}
	else
		pLoss += PositionSingleState(state, state, shelters, 1, action, buffer);

	if (pLoss > 0.0)
		buffer += "T: " + action + " : " + GetStringState(state) + " : " + s_LossState + " " + Dbl2Str(pLoss) + "\n";

	buffer += "\n";
}

void nxnGridOfflineGlobalActions::AddMoveFromEnemy(State & state, int objIdx, intVec & shelters, std::string & buffer) const
{
	//std::string action = "MoveFromEnemy" + std::to_string(objIdx);

	//double pLoss = 0.0;

	//// if enemy dead or not exist do nothing
	//if (state[objIdx] == m_gridSize * m_gridSize)
	//{
	//	pLoss = PositionSingleState(state, state, shelters, 1.0, action, buffer);
	//	if (pLoss > 0.0)
	//		buffer += "T: " + action + " : " + GetStringState(state) + " : " + s_LossState + " " + Dbl2Str(pLoss) + "\n";
	//	buffer += "\n";
	//	return;
	//}

	//int newLoc = MoveFromLocation(state, state.m_locations[objIdx]);

	//State newState(state);

	//// if current location of self is not the farthest point from the enemy make move
	//if (newLoc != state[0])
	//{
	//	std::map<int, double> possibleMoves;
	//	m_self.GetMovement()->GetPossibleMoves(state[0], m_gridSize, state, possibleMoves, newLoc);

	//	intVec newState(state);
	//	for (auto v : possibleMoves)
	//	{
	//		newState[0] = v.first;
	//		pLoss += PositionSingleState(newState, state, shelters, v.second, action, buffer);
	//	}

	//}
	//else
	//	pLoss += PositionSingleState(state, state, shelters, 1.0, action, buffer);


	//if (pLoss > 0.0)
	//	buffer += "T: " + action + " : " + GetStringState(state) + " : " + s_LossState + " " + Dbl2Str(pLoss) + "\n";

	//buffer += "\n";
}

double nxnGridOfflineGlobalActions::MoveToLocation(State & state, intVec & shelters, int location, std::string & action, std::string & buffer) const
{
	// get possible locations according to current direction and location
	std::map<int, double> possibleLocations;
	m_self.GetMovement()->GetPossibleLocations(state.m_locations[0], state.m_directions[0], m_gridSize, possibleLocations);

	double pLoss = 0.0;
	State newState(state);
	for (auto loc : possibleLocations)
	{
		newState.m_locations[0] = loc.first;
		std::map<DIRECTIONS, double> possibleDirections;
		m_self.GetMovement()->GetPossibleDirections(loc.first, state.m_directions[0], location, m_gridSize, possibleDirections);

		for (auto direction : possibleDirections)
		{
			newState.m_directions[0] = direction.first;				
			pLoss += PositionSingleState(newState, state, shelters, loc.second * direction.second, action, buffer);
		}
	}

	return pLoss;
}

void nxnGridOfflineGlobalActions::MakeDeadEnemyStatic(State & state) const
{
	for (int i = 0; i < m_enemyVec.size(); ++i)
		if (state.m_locations[i + 1] == m_gridSize * m_gridSize)
			state.m_directions[i + 1] = NO_DIRECTION;
}

int nxnGridOfflineGlobalActions::MoveFromLocation(State & state, int fromLocation) const
{
	//std::pair<int, int> self = std::make_pair(state[0] % m_gridSize, state[0] / m_gridSize);

	//int maxLocation = state[0];
	//int maxDist = Distance(fromLocation, state[0], m_gridSize);

	//// run on all possible move locations and find the farthest point from fromLocation
	//for (size_t i = 0; i < s_NUM_DIRECTIONS; ++i)
	//{
	//	std::pair<int, int> move = std::make_pair(state[0] % m_gridSize, state[0] / m_gridSize);
	//	move.first += s_lutDirections[i][0];
	//	move.second += s_lutDirections[i][1];

	//	// if move is not on grid continue for next move
	//	if (move.first < 0 | move.first >= m_gridSize | move.second < 0 | move.second >= m_gridSize)
	//		continue;

	//	// if move is valid calculate distance
	//	int currLocation = move.first + move.second * m_gridSize;
	//	if (NoRepeatsLocation(state, currLocation))
	//	{
	//		int currDist = Distance(fromLocation, currLocation, m_gridSize);
	//		// insert location to maxlocation if curr distance is max dist
	//		if (currDist > maxDist)
	//		{
	//			maxLocation = currLocation;
	//			maxDist = currDist;
	//		}

	//	}
	//}
	//return maxLocation;
	return 0;
}

int nxnGridOfflineGlobalActions::MoveToLocationIMP(State & state, int goTo) const
{
	//int selfLocation = state[0];

	//int xDiff = goTo % m_gridSize - selfLocation % m_gridSize;
	//int yDiff = goTo / m_gridSize - selfLocation / m_gridSize;

	//int changeToInsertX = xDiff != 0 ? xDiff / Abs(xDiff) : 0;
	//int changeToInsertY = yDiff != 0 ? (yDiff / Abs(yDiff)) * m_gridSize : 0;

	//int move = selfLocation + changeToInsertX + changeToInsertY;

	//// if the best move is valid return it else if there is only one direction to advance return -1
	//if (NoRepeatsLocation(state, move))
	//	return move;
	//else if (changeToInsertX == 0 | changeToInsertY == 0)
	//	return -1;

	//// try move to in the axis in which we are farther than goTo
	//int secondMove;
	//if (Distance(goTo, selfLocation + changeToInsertX, m_gridSize) < Distance(goTo, selfLocation + changeToInsertY, m_gridSize))
	//{
	//	move = selfLocation + changeToInsertX;
	//	secondMove = selfLocation + changeToInsertY;
	//}
	//else
	//{
	//	secondMove = selfLocation + changeToInsertX;
	//	move = selfLocation + changeToInsertY;
	//}

	//if (NoRepeatsLocation(state, move))
	//	return move;

	//if (NoRepeatsLocation(state, secondMove))
	//	return secondMove;

	return -1;
}
