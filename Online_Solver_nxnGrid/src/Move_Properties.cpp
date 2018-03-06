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


std::vector<Coordinate> Move_Properties::s_directionsLUT(InitDirectionsLUT());
std::vector<std::string> Move_Properties::s_directionNamesLUT(InitDirectionsNamesLUT());

//for lut init
std::vector<Coordinate> Move_Properties::InitDirectionsLUT()
{
	std::vector<Coordinate> lut(NUM_DIRECTIONS);

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

std::vector<std::string> Move_Properties::InitDirectionsNamesLUT()
{
	std::vector<std::string> names(NUM_DIRECTIONS);
	names[SOUTH] = "South";
	names[NORTH] = "North";
	names[EAST] = "East";
	names[WEST] = "West";

	names[SOUTH_EAST] = "South_East";
	names[NORTH_EAST] = "North_East";
	names[SOUTH_WEST] = "South_West";
	names[NORTH_WEST] = "North_West";
	names[NO_DIRECTION] = "No_Direction";

	return names;
}


void Move_Properties::GetRandomMoves(intVec & randomMoves, int objLocation, int gridSize)
{
	Coordinate loc(objLocation % gridSize, objLocation / gridSize);
	for (auto change : s_directionsLUT)
	{
		change += loc;
		int changeLocation = change.GetIdx(gridSize);
		if (change.ValidLocation(gridSize))
			randomMoves.emplace_back(changeLocation);
	}

}

int Move_Properties::MoveToTarget(int location, int target, int gridSize)
{
	// assumption : target and location is inside grid and grid is square so move to target will allways be in grid
	int xDiff = target % gridSize - location % gridSize;
	int yDiff = target / gridSize - location / gridSize;

	int changeToInsertX = xDiff != 0 ? xDiff / Abs(xDiff) : 0;
	int changeToInsertY = yDiff != 0 ? (yDiff / Abs(yDiff)) * gridSize : 0;

	int bestMove = location + changeToInsertX + changeToInsertY;

	return bestMove;
}

SimpleMoveProperties::SimpleMoveProperties(double pSuccess)
: m_pSuccess(pSuccess)
{}

void SimpleMoveProperties::GetPossibleMoves(int location, int gridSize, std::map<int, double> & possibleLocations, int target) const
{	
	int targetMove = MoveToTarget(location, target, gridSize);

	if (targetMove >= 0)
	{
		possibleLocations[targetMove] = m_pSuccess;
		possibleLocations[location] = 1 - m_pSuccess;
	}
	else
		possibleLocations[location] = 1;
}

std::string SimpleMoveProperties::String() const
{
	std::string ret = "Simple Move: p(success) = ";
	ret += std::to_string(m_pSuccess) + " p(stay) = " + std::to_string(1 - m_pSuccess);
	return ret;
}

GeneralDirectionMoveProperties::GeneralDirectionMoveProperties(double pSuccess, double pDirectStraightSuccess, double pDirectDiagonalSuccess)
: m_pSuccess(pSuccess)
, m_pDirectStraightSuccess(pDirectStraightSuccess)
, m_pDirectDiagonalSuccess(pDirectDiagonalSuccess)
{}

void GeneralDirectionMoveProperties::GetPossibleMoves(int location, int gridSize, std::map<int, double> & possibleLocations, int target) const
{
	int targetMove = MoveToTarget(location, target, gridSize);

	if (targetMove >= 0)
	{
		Coordinate target(targetMove % gridSize, targetMove / gridSize);
		Coordinate objLoc(location % gridSize, location / gridSize);
		
		Coordinate diff = objLoc - target;
		int allDiff = Abs(diff.X()) + Abs(diff.Y());

		if (allDiff == 1)
		{ // straight move
			int diagonalFirst = location;
			int diagonalSecond = location;
			double pGeneralDirection = m_pSuccess - m_pDirectStraightSuccess;

			if (Abs(diff.X()) > 0)
			{ // change in y for mistakes
				// if move is valid insert it to move
				Coordinate changes(target);
				
				changes.Y() += 1;
				if (changes.ValidLocation(gridSize))
					diagonalFirst = changes.GetIdx(gridSize);
				
				changes.Y() -= 2;
				if (changes.ValidLocation(gridSize))
					diagonalSecond = changes.GetIdx(gridSize);

			}
			else // change in x for mistakes
			{
				// if move is valid insert it to move
				Coordinate changes(target);

				changes.X() += 1;
				if (changes.ValidLocation(gridSize))
					diagonalFirst = changes.GetIdx(gridSize);

				changes.X() -= 2;
				if (changes.ValidLocation(gridSize))
					diagonalSecond = changes.GetIdx(gridSize);

			}

			possibleLocations[location] = 1 - m_pSuccess;
			possibleLocations[targetMove] = m_pDirectStraightSuccess;
			possibleLocations[diagonalFirst] += pGeneralDirection;
			possibleLocations[diagonalSecond] += pGeneralDirection;
		}
		else
		{ // diagonal move
			int straightX = target.X() + objLoc.Y() * gridSize;
			int straightY = objLoc.X() + target.Y() * gridSize;

			double pGeneralDirection = m_pSuccess - m_pDirectDiagonalSuccess;

			possibleLocations[location] = 1 - m_pSuccess;
			possibleLocations[targetMove] = m_pDirectDiagonalSuccess;
			possibleLocations[straightX] += pGeneralDirection;
			possibleLocations[straightY] += pGeneralDirection;
		}
		
	}
	else
		possibleLocations[location] = 1;
}

std::string GeneralDirectionMoveProperties::String() const
{
	std::string ret = "Simple Move: p(success) = ";
	ret += std::to_string(m_pSuccess) + " p(direct straight success) = " + std::to_string(m_pDirectStraightSuccess) + " p(direct diagonal success) = " + std::to_string(m_pDirectDiagonalSuccess);
	return ret;
}


LowLevelMoveProperties::LowLevelMoveProperties(double pSuccess)
: m_pSuccess(pSuccess)
{}

void LowLevelMoveProperties::GetPossibleMoves(int location, int gridSize, std::map<int, double> & possibleLocations, int target) const
{
	possibleLocations[target] = m_pSuccess;
	possibleLocations[location] = 1 - m_pSuccess;
}

std::string LowLevelMoveProperties::String() const
{
	std::string ret = "target Low Level Move: p(success) = ";
	ret += std::to_string(m_pSuccess) + " p(stay) = " + std::to_string(1 - m_pSuccess);
	return ret;
}


TargetDerivedMoveProperties::TargetDerivedMoveProperties()
	: m_pRandomMove(0.0)
	, m_pStay(1.0)
	, m_pTowardTarget(0.0)
{
}

TargetDerivedMoveProperties::TargetDerivedMoveProperties(double stay, double towardTarget)
	: m_pRandomMove(1 - stay - towardTarget)
	, m_pStay(stay)
	, m_pTowardTarget(towardTarget)
{
}

void TargetDerivedMoveProperties::GetPossibleMoves(int location, int gridSize, std::map<int, double> & possibleLocations, int target) const
{
	double pStay = m_pStay;
	int targetMove = MoveToTarget(location, target, gridSize);

	// if move is possible insert it to result and consider it as blocking location else add its prob to stay
	if (targetMove >= 0)
		possibleLocations[targetMove] = m_pTowardTarget;
	else
		pStay += m_pTowardTarget;

	possibleLocations[location] = pStay;

	if (m_pRandomMove == 0.0)
		return;

	intVec randomMoves;
	GetRandomMoves(randomMoves, location, gridSize);

	double pForEach = m_pRandomMove / randomMoves.size();
	for (auto move : randomMoves)
		possibleLocations[move] += pForEach;

	// return blocking objects to its regular size
}

std::string TargetDerivedMoveProperties::String() const
{
	std::string ret = "target Derived Move: p(toTarget) = ";
	ret += std::to_string(m_pTowardTarget) + " p(stay) = " + std::to_string(m_pStay) + " p(randomMove) = " + std::to_string(m_pRandomMove);

	return ret;
}

NaiveMoveProperties::NaiveMoveProperties(double stay)
: m_pRandomMove(1 - stay)
, m_pStay(stay)
{}

void NaiveMoveProperties::GetPossibleMoves(int location, int gridSize, std::map<int, double> & possibleLocations, int target) const
{
	possibleLocations[location] = m_pStay;

	intVec randomMoves;
	GetRandomMoves(randomMoves, location, gridSize);

	double pForEach = m_pRandomMove / randomMoves.size();
	for (auto move : randomMoves)
		possibleLocations[move] = pForEach;
}

std::string NaiveMoveProperties::String() const
{
	std::string ret = "naive Move: p(Stay) = ";
	ret += std::to_string(m_pStay) + " p(randomMove) = " + std::to_string(m_pRandomMove);

	return ret;
}