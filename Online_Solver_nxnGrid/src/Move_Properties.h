#ifndef MOVE_PROPERTIES_H
#define MOVE_PROPERTIES_H

#include <map>
#include <string>

#include "Coordinate.h"

///properties of movement for object on grid
class Move_Properties
{
public:
	using intVec = std::vector<int>;

	Move_Properties() = default;
	~Move_Properties() = default;
	Move_Properties(const Move_Properties &) = default;
	Move_Properties& operator=(const Move_Properties&) = default;

	virtual void GetPossibleMoves(int location, int gridSize, intVec & blockingObjLocs, std::map<int, double> & possibleLocations, int target = -1) const = 0;
	virtual std::string String() const = 0;
protected:
	static bool NoBlockingLocation(const intVec & blockingObjLocations, int move);
	static void GetRandomMoves(intVec & randomMoves,const intVec & blockingObjLocations, int objLocation, int gridSize);
	static int MoveToTarget(int location, int target, int gridSize, intVec & blockingObjLocations);

	static std::vector<Coordinate> s_availableMovesLUT;
};

class SimpleMoveProperties : public Move_Properties
{
public:
	explicit SimpleMoveProperties(double pSuccess = 1.0);
	~SimpleMoveProperties() = default;
	SimpleMoveProperties(const SimpleMoveProperties &) = default;
	SimpleMoveProperties& operator=(const SimpleMoveProperties&) = default;

	virtual void GetPossibleMoves(int location, int gridSize, intVec & blockingObjLocs, std::map<int, double> & possibleLocations, int target) const override;
	virtual std::string String() const override;
private:
	double m_pSuccess;
};

class GeneralDirectionMoveProperties : public Move_Properties
{
public:
	explicit GeneralDirectionMoveProperties(double pSuccess, double pDirectStraightSuccess, double pDirectDiagonalSuccess);
	~GeneralDirectionMoveProperties() = default;
	GeneralDirectionMoveProperties(const GeneralDirectionMoveProperties &) = default;
	GeneralDirectionMoveProperties& operator=(const GeneralDirectionMoveProperties&) = default;

	virtual void GetPossibleMoves(int location, int gridSize, intVec & blockingObjLocs, std::map<int, double> & possibleLocations, int target) const override;
	virtual std::string String() const override;
private:
	double m_pSuccess;
	double m_pDirectStraightSuccess;
	double m_pDirectDiagonalSuccess;
};

class LowLevelMoveProperties : public Move_Properties
{
public:
	explicit LowLevelMoveProperties(double pSuccess = 1.0);
	~LowLevelMoveProperties() = default;
	LowLevelMoveProperties(const LowLevelMoveProperties &) = default;
	LowLevelMoveProperties& operator=(const LowLevelMoveProperties&) = default;

	virtual void GetPossibleMoves(int location, int gridSize, intVec & blockingObjLocs, std::map<int, double> & possibleLocations, int target) const override;
	virtual std::string String() const override;
private:
	double m_pSuccess;
};

class TargetDerivedMoveProperties : public Move_Properties
{
public:
	explicit TargetDerivedMoveProperties();
	explicit TargetDerivedMoveProperties(double stay, double towardTarget);
	~TargetDerivedMoveProperties() = default;
	TargetDerivedMoveProperties(const TargetDerivedMoveProperties &) = default;
	TargetDerivedMoveProperties& operator=(const TargetDerivedMoveProperties&) = default;

	virtual void GetPossibleMoves(int location, int gridSize, intVec & blockingObjLocs, std::map<int, double> & possibleLocations, int target) const override;
	virtual std::string String() const override;
	
private:
	double m_pRandomMove;
	double m_pStay;
	double m_pTowardTarget;
};

class NaiveMoveProperties : public Move_Properties
{
public:
	explicit NaiveMoveProperties(double stay = 1.0);
	~NaiveMoveProperties() = default;
	NaiveMoveProperties(const NaiveMoveProperties &) = default;
	NaiveMoveProperties& operator=(const NaiveMoveProperties&) = default;

	virtual void GetPossibleMoves(int location, int gridSize, intVec & blockingObjLocs, std::map<int, double> & possibleLocations, int target) const override;
	virtual std::string String() const override;

private:
	double m_pRandomMove;
	double m_pStay;

};
# endif //MOVE_PROPERTIES_H