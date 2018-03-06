#include "nxnGridOffline.h"
#include "nxnGridOfflineLocalActions.h"

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

nxnGridOfflineLocalActions::nxnGridOfflineLocalActions(int gridSize, int targetIdx, Self_Obj& self, bool isFullyObs, double discount)
: nxnGridOffline(gridSize, targetIdx, self, isFullyObs, discount)
{
}

void nxnGridOfflineLocalActions::SaveInPomdpFormat(FILE *fptr)
{
	std::string buffer("");
	//add comments and init lines(state observations etc.) to file
	CommentsAndInitLines(fptr);

	// add position with and without moving of the robot
	AddAllActions(fptr);

	// add observations and rewards
	ObservationsAndRewards(fptr);
}

int nxnGridOfflineLocalActions::GetNumActions() const
{
	return s_NUM_DIRECTIONS + 1 + m_enemyVec.size();
}

void nxnGridOfflineLocalActions::CommentsAndInitLines(FILE *fptr)
{
	std::string buffer;

	// add comments
	buffer += "# pomdp file:\n";
	buffer += "# grid size: " + std::to_string(m_gridSize) + "  target idx: " + std::to_string(m_targetIdx);
	buffer += "\n# SELF:\n# self initial location: " + std::to_string(m_self.GetLocation().GetIdx(m_gridSize));
	buffer += "\n# with movement: " + m_self.GetMovement()->String();
	buffer += "\n# with attack: " + m_self.GetAttack()->String();
	buffer += "\n# with observation: " + m_self.GetObservation()->String();
	buffer += "\n\n# ENEMIES:";
	for (auto v : m_enemyVec)
	{
		buffer += "\n\n# enemy initial location: " + std::to_string(v.GetLocation().GetIdx(m_gridSize));
		buffer += "\n# with movement: " + m_self.GetMovement()->String();
		buffer += "\n# with attack: " + v.GetAttack()->String();
	}

	buffer += "\n\n# NON-INVOLVED:";
	for (auto v : m_nonInvolvedVec)
	{
		buffer += "\n\n# non- involved initial location: " + std::to_string(v.GetLocation().GetIdx(m_gridSize));
		buffer += "\n# with movement: " + m_self.GetMovement()->String();
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
	CalcStatesAndObs("s", buffer);
	buffer += s_WinState + " " + s_LossState + "\n";
	buffer += "actions: Stay North South West East NorthWest NorthEast SouthWest SouthEast Attack\n";

	// add observations names
	buffer += "observations: ";
	CalcStatesAndObs("o", buffer);
	buffer += "o" + s_WinState + " o" + s_LossState + "\n";
	buffer += "\n\nstart: \n";

	// add start states probability
	CalcStartState(buffer);
	buffer += "\n\n";
	//save to file
	auto err = fputs(buffer.c_str(), fptr);
	if (err < 0) { std::cerr << "Error Writing to file\n"; exit(1); }
}

void nxnGridOfflineLocalActions::AddAllActions(FILE * fptr)
{
	std::string buffer;

	// add move to win state from states when the robot is in target
	intVec state(CountMovableObj());
	state[0] = m_targetIdx;
	AddTargetPositionRec(state, 1, buffer);
	buffer += "\n";

	// add actions for all states
	AddActionsAllStates(buffer);

	// save to file
	auto err = fputs(buffer.c_str(), fptr);
	if (err < 0) { std::cerr << "Error Writing to file\n"; exit(1); }
}

void nxnGridOfflineLocalActions::AddActionsAllStates(std::string & buffer)
{
	intVec state(CountMovableObj());
	std::string action = "*";

	std::vector<int> shelters;
	CreateShleterVec(shelters);

	AddActionsRec(state, shelters, 0, buffer);

	buffer += "\n\nT: * : " + s_WinState + " : " + s_WinState + " 1";
	buffer += "\nT: * : " + s_LossState + " : " + s_LossState + " 1\n\n";
}

void nxnGridOfflineLocalActions::AddActionsRec(intVec & state, intVec & shelters, int currObj, std::string & buffer)
{
	if (currObj == state.size())
	{
		// if we are allready in the target idx do nothing
		if (state[0] == m_targetIdx)
			return;

		AddAllMoves(state, shelters, buffer);
		AddAttack(state, shelters, buffer);
	}
	else
	{
		for (int i = 0; i < m_gridSize * m_gridSize; ++i)
		{
			state[currObj] = i;
			if (NoRepeats(state, currObj))
			{
				AddActionsRec(state, shelters, currObj + 1, buffer);
			}
		}
		// if the object is enemy add cases of when the enemy is dead
		if (IsEnemy(currObj))
		{
			state[currObj] = Attack::DeadLoc(m_gridSize);
			AddActionsRec(state, shelters, currObj + 1, buffer);
		}
	}
}

void nxnGridOfflineLocalActions::AddAllMoves(intVec & state, intVec & shelters, std::string & buffer) const
{
	// add stay action
	std::string stayAction("Stay");
	double pLoss = PositionSingleState(state, state, shelters, 1.0, stayAction, buffer);
	if (pLoss > 0.0)
		buffer += "T: " + stayAction + " : " + GetStringState(state) + " : " + s_LossState + " " + Dbl2Str(pLoss) + "\n";

	// add moves
	AddSingleMove(state, shelters, "North", Coordinate(0, -1), buffer);
	AddSingleMove(state, shelters, "South", Coordinate(0, 1), buffer);
	AddSingleMove(state, shelters, "West", Coordinate(-1, 0), buffer);
	AddSingleMove(state, shelters, "East", Coordinate(1, 0), buffer);

	AddSingleMove(state, shelters, "NorthWest", Coordinate(-1, -1), buffer);
	AddSingleMove(state, shelters, "NorthEast", Coordinate(1, -1), buffer);

	AddSingleMove(state, shelters, "SouthWest", Coordinate(-1, 1), buffer);
	AddSingleMove(state, shelters, "SouthEast", Coordinate(1, 1), buffer);
}

void nxnGridOfflineLocalActions::AddSingleMove(intVec & state, intVec & shelters, std::string action, Coordinate advance, std::string & buffer) const
{
	double pLoss = 0.0;
	Coordinate newLocation(state[0] % m_gridSize, state[0] / m_gridSize);
	newLocation += advance;
	
	// if the move is valid do it o.w. create transition from state to the same state
	if (newLocation.ValidLocation(m_gridSize))
	{
		int newLoc = newLocation.GetIdx(m_gridSize);
		std::map<int, double> possibleLocs;
		m_self.GetMovement()->GetPossibleMoves(state[0], m_gridSize, possibleLocs, newLoc);

		intVec newState(state);
		for (auto loc : possibleLocs)
		{
			newState[0] = loc.first;
			pLoss += PositionSingleState(newState, state, shelters, loc.second, action, buffer);
		}
	}
	else
		pLoss += PositionSingleState(state, state, shelters, 1.0, action, buffer);

	if (pLoss > 0.0)
		buffer += "T: " + action + " : " + GetStringState(state) + " : " + s_LossState + " " + Dbl2Str(pLoss) + "\n";

	buffer += "\n";
}

void nxnGridOfflineLocalActions::AddAttack(intVec & state, intVec & shelters, std::string & buffer) const
{
	std::string action = "Attack";
	double pLoss = 0.0;

	// TODO : implement this for 2 enemies

	// if enemy dead or not exist do nothing
	if (m_enemyVec.size() == 0 || Attack::IsDead(state[1], m_gridSize))
	{
		PositionSingleState(state, state, shelters, 1.0, action, buffer);
		buffer += "\n";
		return;
	}

	Coordinate self(state[0] % m_gridSize, state[0] / m_gridSize);
	Coordinate enemy(state[1] % m_gridSize, state[1] / m_gridSize);
	
	if (m_self.GetAttack()->GetRange() >= self.RealDistance(enemy))
	{
		std::vector<std::pair<intVec, double>> shootOutcomes;

		// creating a vecotr of shoot outcomes
		m_self.GetAttack()->AttackOffline(state, 0, state[1], shelters, m_gridSize, shootOutcomes);

		for (auto v : shootOutcomes)
		{
			// if non-involved dead transfer to loss state else calculate move states
			if (!IsNonInvDead(v.first))
				pLoss += PositionSingleState(v.first, state, shelters, v.second, action, buffer);
			else
				pLoss += v.second;
		}
	}
	else
		pLoss += MoveToLocation(state, shelters, state[1], action, buffer);

	if (pLoss > 0.0)
		buffer += "T: " + action + " : " + GetStringState(state) + " : " + s_LossState + " " + Dbl2Str(pLoss) + "\n";

	buffer += "\n";
}

double nxnGridOfflineLocalActions::MoveToLocation(intVec & state, intVec & shelters, int location, std::string & action, std::string & buffer) const
{
	std::map<int, double> possibleLocs;
	m_self.GetMovement()->GetPossibleMoves(state[0], m_gridSize, possibleLocs, location);

	intVec newState(state);
	double pLoss = 0.0;

	for (auto move : possibleLocs)
	{
		newState[0] = move.first;
		pLoss += PositionSingleState(newState, state, shelters, move.second, action, buffer);
	}

	return pLoss;
}

int nxnGridOfflineLocalActions::MoveToLocationIMP(intVec & state, int goTo) const
{
	int selfLocation = state[0];

	int xDiff = goTo % m_gridSize - selfLocation % m_gridSize;
	int yDiff = goTo / m_gridSize - selfLocation / m_gridSize;

	int changeToInsertX = xDiff != 0 ? xDiff / Abs(xDiff) : 0;
	int changeToInsertY = yDiff != 0 ? (yDiff / Abs(yDiff)) * m_gridSize : 0;

	int move = selfLocation + changeToInsertX + changeToInsertY;

	// if the best move is valid return it else if there is only one direction to advance return -1
	if (NoRepeatsLocation(state, move))
		return move;
	else if (changeToInsertX == 0 | changeToInsertY == 0)
		return -1;

	// try move to in the axis in which we are farther than goTo
	int secondMove;
	if (Distance(goTo, selfLocation + changeToInsertX, m_gridSize) > Distance(goTo, selfLocation + changeToInsertY, m_gridSize))
	{
		move = selfLocation + changeToInsertX;
		secondMove = selfLocation + changeToInsertY;
	}
	else
	{
		secondMove = selfLocation + changeToInsertX;
		move = selfLocation + changeToInsertY;
	}

	if (NoRepeatsLocation(state, move))
		return move;

	if (NoRepeatsLocation(state, secondMove))
		return secondMove;

	return -1;
}
