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

std::vector<Coordinate> InitAvailableMovesLUT()
{
	return std::vector<Coordinate> { { 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 }, { 1, 1 }, { 1, -1 }, { -1, 1 }, { -1, -1} };
}

std::vector<Coordinate> Move_Properties::s_availableMovesLUT(InitAvailableMovesLUT());

bool Move_Properties::NoBlockingLocation(const intVec & blockingObjLocations, int move)
{
	int l = 0;
	for (; l < blockingObjLocations.size() - 1 & blockingObjLocations[l] != move; ++l);

	return !(blockingObjLocations[l] == move);
}
void Move_Properties::GetRandomMoves(intVec & randomMoves, const intVec & blockingObjLocations, int objLocation, int gridSize)
{
	Coordinate loc(objLocation % gridSize, objLocation / gridSize);
	for (auto change : s_availableMovesLUT)
	{
		change += loc;
		int changeLocation = change.GetIdx(gridSize);
		if (change.ValidLocation(gridSize) & NoBlockingLocation(blockingObjLocations, changeLocation))
			randomMoves.emplace_back(changeLocation);
	}

}

int Move_Properties::MoveToTarget(int location, int target, int gridSize, intVec & blockingObjLocations)
{
	int xDiff = target % gridSize - location % gridSize;
	int yDiff = target / gridSize - location / gridSize;

	int changeToInsertX = xDiff != 0 ? xDiff / Abs(xDiff) : 0;
	int changeToInsertY = yDiff != 0 ? (yDiff / Abs(yDiff)) * gridSize : 0;

	int bestMove = location + changeToInsertX + changeToInsertY;

	// if the best move is valid return it else if there is only one direction to advance return -1
	if (NoBlockingLocation(blockingObjLocations, bestMove))
		return bestMove;
	else if (changeToInsertX == 0 | changeToInsertY == 0)
		return -1;

	// try move to in the axis in which we are farther than goTo
	int secondMove;
	if (Distance(target, location + changeToInsertX, gridSize) < Distance(target, location + changeToInsertY, gridSize))
	{
		bestMove = location + changeToInsertX;
		secondMove = location + changeToInsertY;
	}
	else
	{
		secondMove = location + changeToInsertX;
		bestMove = location + changeToInsertY;
	}

	if (NoBlockingLocation(blockingObjLocations, bestMove))
		return bestMove;

	if (NoBlockingLocation(blockingObjLocations, secondMove))
		return secondMove;

	return -1;
}

SimpleMoveProperties::SimpleMoveProperties(double pSuccess)
: m_pSuccess(pSuccess)
{}

void SimpleMoveProperties::GetPossibleMoves(int location, int gridSize, intVec & blockingObjLocs, std::map<int, double> & possibleLocations, int target) const
{	
	int targetMove = MoveToTarget(location, target, gridSize, blockingObjLocs);

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

void GeneralDirectionMoveProperties::GetPossibleMoves(int location, int gridSize, intVec & blockingObjLocs, std::map<int, double> & possibleLocations, int target) const
{
	int targetMove = MoveToTarget(location, target, gridSize, blockingObjLocs);

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
				if (changes.ValidLocation(gridSize) && NoBlockingLocation(blockingObjLocs, changes.GetIdx(gridSize)))
					diagonalFirst = changes.GetIdx(gridSize);
				
				changes.Y() -= 2;
				if (changes.ValidLocation(gridSize) && NoBlockingLocation(blockingObjLocs, changes.GetIdx(gridSize)))
					diagonalSecond = changes.GetIdx(gridSize);

			}
			else // change in x for mistakes
			{
				// if move is valid insert it to move
				Coordinate changes(target);

				changes.X() += 1;
				if (changes.ValidLocation(gridSize) && NoBlockingLocation(blockingObjLocs, changes.GetIdx(gridSize)))
					diagonalFirst = changes.GetIdx(gridSize);

				changes.X() -= 2;
				if (changes.ValidLocation(gridSize) && NoBlockingLocation(blockingObjLocs, changes.GetIdx(gridSize)))
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

			if (!NoBlockingLocation(blockingObjLocs, straightX))
				straightX = location;

			if (!NoBlockingLocation(blockingObjLocs, straightY))
				straightY = location;

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

void LowLevelMoveProperties::GetPossibleMoves(int location, int gridSize, intVec & blockingObjLocs, std::map<int, double> & possibleLocations, int target) const
{
	if (NoBlockingLocation(blockingObjLocs, target))
	{
		possibleLocations[target] = m_pSuccess;
		possibleLocations[location] = 1 - m_pSuccess;
	}
	else
		possibleLocations[location] = 1;
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

void TargetDerivedMoveProperties::GetPossibleMoves(int location, int gridSize, intVec & blockingObjLocs, std::map<int, double> & possibleLocations, int target) const
{
	int blockingObjSize = blockingObjLocs.size();
	double pStay = m_pStay;
	int targetMove = MoveToTarget(location, target, gridSize, blockingObjLocs);

	// if move is possible insert it to result and consider it as blocking location else add its prob to stay
	if (targetMove >= 0)
		possibleLocations[targetMove] = m_pTowardTarget;
	else
		pStay += m_pTowardTarget;

	possibleLocations[location] = pStay;

	if (m_pRandomMove == 0.0)
		return;

	intVec randomMoves;
	GetRandomMoves(randomMoves, blockingObjLocs, location, gridSize);

	double pForEach = m_pRandomMove / randomMoves.size();
	for (auto move : randomMoves)
		possibleLocations[move] += pForEach;

	// return blocking objects to its regular size
	blockingObjLocs.resize(blockingObjSize);
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

void NaiveMoveProperties::GetPossibleMoves(int location, int gridSize, intVec & blockingObjLocs, std::map<int, double> & possibleLocations, int target) const
{
	possibleLocations[location] = m_pStay;

	intVec randomMoves;
	GetRandomMoves(randomMoves, blockingObjLocs, location, gridSize);

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