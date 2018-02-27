#include "Move_Properties.h"

inline int Abs(int a)
{
	return a * (a >= 0) - a * (a < 0);
}

static int Distance(int loc1, int loc2, int gridSize)
{
	int xDiff = loc1 % gridSize - loc2 % gridSize;
	int yDiff = loc1 / gridSize - loc2 / gridSize;

	return xDiff * xDiff + yDiff * yDiff;
}

// init all LUT for moves. keep order because of dependencies in init

std::vector<std::string> Move_Properties::s_directionNames(InitDirectionNamesLUT());

direction2Coord_t Move_Properties::s_availableDirectionsLUT(InitAvailableDirectionsLUT());
coord2Direction_t Move_Properties::s_coordChange2DirectionLUT(InitCoord2DirectionsLUT());

changeOfDirection2Direction_t Move_Properties::s_changeOfDirection2DirectionLUT(InitChangeOfDirection2Direction());

desiredMove2ChangeOfDirection_t Move_Properties::s_changeOfDirectionForMoveLUT(ChangeOfDirectionForMoveLUT());

std::vector<std::string> Move_Properties::InitDirectionNamesLUT()
{
	std::vector<std::string> names(NUM_DIRECTIONS);
	names[SOUTH] = "S";
	names[NORTH] = "N";
	names[EAST] = "E";
	names[WEST] = "W";

	names[SOUTH_EAST] = "SE";
	names[NORTH_EAST] = "NE";
	names[SOUTH_WEST] = "SW";
	names[NORTH_WEST] = "NW";
	names[NO_DIRECTION] = "WO";

	return names;
}

direction2Coord_t Move_Properties::InitAvailableDirectionsLUT()
{
	direction2Coord_t lut(NUM_DIRECTIONS);
	
	lut[SOUTH].X() = 0;
	lut[SOUTH].Y() = 1;

	lut[NORTH].X() = 0;
	lut[NORTH].Y() = -1;

	lut[EAST].X() = 1;
	lut[EAST].Y() = 0;

	lut[WEST].X() = -1;
	lut[WEST].Y() = 0;

	lut[SOUTH_EAST].X() = 1;
	lut[SOUTH_EAST].Y() = 1;

	lut[NORTH_EAST].X() = 1;
	lut[NORTH_EAST].Y() = -1;

	lut[SOUTH_WEST].X() = -1;
	lut[SOUTH_WEST].Y() = 1;

	lut[NORTH_WEST].X() = -1;
	lut[NORTH_WEST].Y() = -1;

	lut[NO_DIRECTION].X() = 0;
	lut[NO_DIRECTION].Y() = 0;

	return lut;
}

coord2Direction_t Move_Properties::InitCoord2DirectionsLUT()
{
	coord2Direction_t lut;

	for (int i = 0; i < NUM_DIRECTIONS; ++i)
		lut[s_availableDirectionsLUT[i]] = (DIRECTIONS)i;

	return lut;
}

desiredMove2ChangeOfDirection_t Move_Properties::ChangeOfDirectionForMoveLUT()
{
	desiredMove2ChangeOfDirection_t lut;

	Coordinate stop(0,0);
	for (int targetDir = 0; targetDir < NUM_DIRECTIONS; ++targetDir)
	{
		DIRECTIONS targetDirection = (DIRECTIONS)targetDir;
		Coordinate target = s_availableDirectionsLUT[targetDir];
		
		for (int origDir = 0; origDir < NUM_DIRECTIONS; ++origDir)
		{
			DIRECTIONS currentDirection = (DIRECTIONS)origDir;

			directionPair_t key(currentDirection, targetDirection);
			changeOfDirectionPair_t changes;

			if (origDir != NO_DIRECTION)
			{
				int distancesToTarget[NUM_CHANGES_OF_DIRECTION] = { 0 };
				for (int change = 0; change < NUM_CHANGES_OF_DIRECTION; ++change)
				{
					DIRECTIONS directionForChange = s_changeOfDirection2DirectionLUT[std::make_pair(currentDirection, (CHANGE_OF_DIRECTION)change)];
					Coordinate coord = s_availableDirectionsLUT[directionForChange];
					distancesToTarget[change] = target.Distance(coord);
				}

				// find best 2 moves
				int secondMinIdx = -1, secondMinDist = 10;
				int minIdx = -1, minDist = 10;
				for (int i = 0; i < NUM_CHANGES_OF_DIRECTION; ++i)
				{
					if (distancesToTarget[i] < secondMinDist)
					{
						if (distancesToTarget[i] < minDist)
						{
							minDist = distancesToTarget[i];
							minIdx = i;
						}
						else
						{
							secondMinDist = distancesToTarget[i];
							secondMinIdx = i;
						}
					}
				}

				changes.first = (CHANGE_OF_DIRECTION)minIdx;
				changes.second = (CHANGE_OF_DIRECTION)secondMinIdx;
			}
			else
			{
				changes.first = STOP;
				changes.second = STOP;
			}

			lut[key] = changes;
		}
	}

	return lut;
}

changeOfDirection2Direction_t Move_Properties::InitChangeOfDirection2Direction()
{
	changeOfDirection2Direction_t lut;
	
	// left turn lut
	lut[std::make_pair(SOUTH, LEFT)] = SOUTH_EAST;
	lut[std::make_pair(SOUTH_EAST, LEFT)] = EAST;
	lut[std::make_pair(EAST, LEFT)] = NORTH_EAST;
	lut[std::make_pair(NORTH_EAST, LEFT)] = NORTH;
	lut[std::make_pair(NORTH, LEFT)] = NORTH_WEST;
	lut[std::make_pair(NORTH_WEST, LEFT)] = WEST;
	lut[std::make_pair(WEST, LEFT)] = SOUTH_WEST;
	lut[std::make_pair(SOUTH_WEST, LEFT)] = SOUTH;

	lut[std::make_pair(NO_DIRECTION, LEFT)] = NO_DIRECTION;
	
	// right turn lut (the opposite as left turn) (insert to different map so not go other these fields as well)
	changeOfDirection2Direction_t rightTurn;
	for (auto leftTurn : lut)
		rightTurn[std::make_pair(leftTurn.second, RIGHT)] = leftTurn.first.first;

	for (auto right : rightTurn)
		lut[right.first] = right.second;

	// continue
	for (int dir = 0; dir < NUM_DIRECTIONS; ++dir)
	{
		DIRECTIONS curr = (DIRECTIONS)dir;
		lut[std::make_pair(curr, CONTINUE)] = curr;
	}

	//stop
	for (int dir = 0; dir < NUM_DIRECTIONS; ++dir)
	{
		DIRECTIONS curr = (DIRECTIONS)dir;
		lut[std::make_pair(curr, STOP)] = NO_DIRECTION;
	}

	return lut;
}

Move_Properties::Move_Properties(double pSpeed)
: m_pSpeed(pSpeed)
{
}

bool Move_Properties::ValidLocation2Direction(int location, DIRECTIONS dir, int gridSize)
{
	Coordinate loc(location% gridSize, location / gridSize);
	loc += s_availableDirectionsLUT[dir];

	return loc.X() >= 0 && loc.X() < gridSize && loc.Y() >= 0 && loc.Y() < gridSize;
}

bool Move_Properties::NoBlockingLocation(const locationVec & blockingObjLocations, int move)
{
	int l = 0;
	for (; l < blockingObjLocations.size() - 1 & blockingObjLocations[l] != move; ++l);

	return !(blockingObjLocations[l] == move);
}

void Move_Properties::GetRandomMoves(locationVec & randomMoves, const locationVec & blockingObjLocations, int objLocation, int gridSize)
{
	Coordinate loc(objLocation % gridSize, objLocation / gridSize);
	for (auto change : s_availableDirectionsLUT)
	{
		change += loc;
		int changeLocation = change.GetIdx(gridSize);
		if (change.ValidLocation(gridSize) & NoBlockingLocation(blockingObjLocations, changeLocation))
			randomMoves.emplace_back(changeLocation);
	}

}

DIRECTIONS  Move_Properties::ChangeDirectionToTarget(int location, DIRECTIONS direction, int target, int gridSize)
{
	int xDiff = target % gridSize - location % gridSize;
	int yDiff = target / gridSize - location / gridSize;
	
	int absXDiff = abs(xDiff);
	if (absXDiff > 1)
		xDiff = xDiff / absXDiff;

	int absYDiff = abs(yDiff);
	if (absYDiff > 1)
		yDiff = yDiff / absYDiff;

	Coordinate directionVec(xDiff, yDiff);
	DIRECTIONS desiredDirection = s_coordChange2DirectionLUT[directionVec];

	// if there curr direction is no_direction return desired direction
	if (direction == NO_DIRECTION)
		return desiredDirection;
	// find turn that get us closer to desired direction
	
	directionPair_t key(direction, desiredDirection);
	changeOfDirectionPair_t change = s_changeOfDirectionForMoveLUT[key];
	
	// if best available direction is valid return it
	DIRECTIONS bestAvailableDirection = s_changeOfDirection2DirectionLUT[std::make_pair(direction, change.first)];	
	
	if (ValidLocation2Direction(location, bestAvailableDirection, gridSize))
		return bestAvailableDirection;

	DIRECTIONS secondAvailableDirection = s_changeOfDirection2DirectionLUT[std::make_pair(direction, change.second)];
	if (ValidLocation2Direction(location, bestAvailableDirection, gridSize))
		return bestAvailableDirection;

	return NO_DIRECTION;
}

void Move_Properties::GetPossibleLocations(int location, DIRECTIONS direction, int gridSize, std::map<int, double> & possibleLocations) const
{
	Coordinate coord(location % gridSize, location / gridSize);
	coord += s_availableDirectionsLUT[direction];
	int newLoc = coord.GetIdx(gridSize);

	possibleLocations[newLoc] = m_pSpeed;
	if (m_pSpeed < 1)
		possibleLocations[location] += 1 - m_pSpeed;
}

std::string Move_Properties::String() const
{
	return " p(move) = " + std::to_string(m_pSpeed);
}

SimpleMoveProperties::SimpleMoveProperties(double speed, double pSuccess)
: Move_Properties(speed)
, m_pSuccess(pSuccess)
{}



void SimpleMoveProperties::GetPossibleDirections(int location, DIRECTIONS direction, int target, int gridSize, std::map<DIRECTIONS, double> & possibleDirections) const
{
	DIRECTIONS desiredDir = ChangeDirectionToTarget(location, direction, target, gridSize);
	possibleDirections[desiredDir] = m_pSuccess;
	
	if (m_pSuccess < 1)
	{
		if (ValidLocation2Direction(location, direction, gridSize))
			possibleDirections[direction] += 1 - m_pSuccess;
		else
			possibleDirections[NO_DIRECTION] += 1 - m_pSuccess;
	}
}

std::string SimpleMoveProperties::String() const
{
	std::string ret = "Simple Move: p(success) = ";
	ret += std::to_string(m_pSuccess) + Move_Properties::String();
	return ret;
}

TargetDerivedMoveProperties::TargetDerivedMoveProperties(double speed, double pTowardTarget, double pStop, double pRandomChangeOfDirection)
: Move_Properties(speed)
, m_pStop(pStop)
, m_pTowardTarget(pTowardTarget)
, m_pRandomChangeOfDirection(pRandomChangeOfDirection)
, m_pContinue(1 - pStop - pTowardTarget - pRandomChangeOfDirection)
{
}

void TargetDerivedMoveProperties::GetPossibleDirections(int location, DIRECTIONS direction, int target, int gridSize, std::map<DIRECTIONS, double> & possibleDirections) const
{
	DIRECTIONS desiredDir = ChangeDirectionToTarget(location, direction, target, gridSize);	
	possibleDirections[desiredDir] += m_pTowardTarget;
	
	possibleDirections[NO_DIRECTION] += m_pStop;

	if (ValidLocation2Direction(location, direction, gridSize))
		possibleDirections[direction] += m_pContinue;
	else
		possibleDirections[NO_DIRECTION] += m_pContinue;

	double nonValTurns = 0;
	double pChangeOneDir = m_pRandomChangeOfDirection / 2;

	DIRECTIONS leftTurnDirection = s_changeOfDirection2DirectionLUT[std::make_pair(direction, LEFT)];
	if (ValidLocation2Direction(location, leftTurnDirection, gridSize))
		possibleDirections[leftTurnDirection] += pChangeOneDir;
	else
		nonValTurns += pChangeOneDir;
	
	DIRECTIONS rightTurnDirection = s_changeOfDirection2DirectionLUT[std::make_pair(direction, RIGHT)];
	if (ValidLocation2Direction(location, rightTurnDirection, gridSize))
		possibleDirections[rightTurnDirection] += pChangeOneDir;
	else
		nonValTurns += pChangeOneDir;

	if (nonValTurns > 0)
		possibleDirections[NO_DIRECTION] += nonValTurns;
}

std::string TargetDerivedMoveProperties::String() const
{
	std::string ret = "Target Derived Move: p(toTarget) = ";
	ret += std::to_string(m_pTowardTarget) + " p(stop) = " + std::to_string(m_pStop) + " p(random Direction Change) = " + std::to_string(m_pRandomChangeOfDirection)
		+ " p(continue) = " + std::to_string(m_pContinue) + Move_Properties::String();

	return ret;
}


NaiveMoveProperties::NaiveMoveProperties(double speed, double pRandomChangeOfDirection, double pStop)
: Move_Properties(speed)
, m_pStop(pStop)
, m_pRandomChangeOfDirection(pRandomChangeOfDirection)
, m_pContinue(1 - pStop - pRandomChangeOfDirection)
{}

void NaiveMoveProperties::GetPossibleDirections(int location, DIRECTIONS direction, int target, int gridSize, std::map<DIRECTIONS, double> & possibleDirections) const
{
	possibleDirections[NO_DIRECTION] += m_pStop;
	
	if (ValidLocation2Direction(location, direction, gridSize))
		possibleDirections[direction] += m_pContinue;
	else
		possibleDirections[NO_DIRECTION] += m_pContinue;

	double nonValTurns = 0;
	double pChangeOneDir = m_pRandomChangeOfDirection / 2;

	DIRECTIONS leftTurnDirection = s_changeOfDirection2DirectionLUT[std::make_pair(direction, LEFT)];
	if (ValidLocation2Direction(location, leftTurnDirection, gridSize))
		possibleDirections[leftTurnDirection] += pChangeOneDir;
	else
		nonValTurns += pChangeOneDir;

	DIRECTIONS rightTurnDirection = s_changeOfDirection2DirectionLUT[std::make_pair(direction, RIGHT)];
	if (ValidLocation2Direction(location, leftTurnDirection, gridSize))
		possibleDirections[rightTurnDirection] += pChangeOneDir;
	else
		nonValTurns += pChangeOneDir;

	if (nonValTurns > 0)
		possibleDirections[NO_DIRECTION] += nonValTurns;
}

std::string NaiveMoveProperties::String() const
{
	std::string ret = "naive Move: p(Stop) = ";
	ret += std::to_string(m_pStop) + " p(random Direction Change) = " + std::to_string(m_pRandomChangeOfDirection)
		+ " p(continue) = " + std::to_string(m_pContinue) + Move_Properties::String();

	return ret;
}