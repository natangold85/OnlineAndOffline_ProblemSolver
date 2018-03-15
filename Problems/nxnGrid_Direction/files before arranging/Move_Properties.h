#ifndef MOVE_PROPERTIES_H
#define MOVE_PROPERTIES_H

#include <map>
#include <string>

#include "Coordinate.h"

enum DIRECTIONS { SOUTH, NORTH, EAST, WEST, SOUTH_EAST, NORTH_EAST, SOUTH_WEST, NORTH_WEST, NO_DIRECTION, NUM_DIRECTIONS };
enum CHANGE_OF_DIRECTION { LEFT, RIGHT, STOP, CONTINUE, NUM_CHANGES_OF_DIRECTION };

/*LUT TYPES*/
using direction2Coord_t = std::vector<Coordinate>;
using coord2Direction_t = std::map<Coordinate, DIRECTIONS>;
using changeOfDirection2Direction_t = std::map<std::pair<DIRECTIONS, CHANGE_OF_DIRECTION>, DIRECTIONS>;

using directionPair_t = std::pair<DIRECTIONS, DIRECTIONS>;
using changeOfDirectionPair_t = std::pair<CHANGE_OF_DIRECTION, CHANGE_OF_DIRECTION>;
// desired move (target direction and src direction) to change of direction possibility (best and second best)
using desiredMove2ChangeOfDirection_t = std::map<directionPair_t, changeOfDirectionPair_t>;

///properties of movement for object on grid
class Move_Properties
{
public:
	/*LUT INIT FUNCTIONS*/
	static std::vector<std::string> InitDirectionNamesLUT();
	
	static direction2Coord_t InitAvailableDirectionsLUT();
	static coord2Direction_t InitCoord2DirectionsLUT();
	static desiredMove2ChangeOfDirection_t ChangeOfDirectionForMoveLUT();

	static changeOfDirection2Direction_t InitChangeOfDirection2Direction();

	/*LUT MEMBERS*/
	static std::vector<std::string> s_directionNames;
	static direction2Coord_t s_availableDirectionsLUT;
	static coord2Direction_t s_coordChange2DirectionLUT;
	static desiredMove2ChangeOfDirection_t s_changeOfDirectionForMoveLUT;
	static changeOfDirection2Direction_t s_changeOfDirection2DirectionLUT;

public:
	using locationVec = std::vector<int>;

	Move_Properties(double m_pSpeed);
	~Move_Properties() = default;
	Move_Properties(const Move_Properties &) = default;
	Move_Properties& operator=(const Move_Properties&) = default;

	virtual void GetPossibleDirections(int location, DIRECTIONS direction, int target, int gridSize, std::map<DIRECTIONS, double> & possibleDirections) const = 0;
	virtual void GetPossibleLocations(int location, DIRECTIONS direction, int gridSize, std::map<int, double> & possibleLocations) const;

	virtual std::string String() const;

	static bool ValidLocation2Direction(int location, DIRECTIONS dir, int gridSize);
protected:
	static bool NoBlockingLocation(const locationVec & blockingObjLocations, int move);
	static void GetRandomMoves(locationVec & randomMoves,const locationVec & blockingObjLocations, int objLocation, int gridSize);
	
	static DIRECTIONS ChangeDirectionToTarget(int location, DIRECTIONS direction, int target, int gridSize);

	/*MEMBERS*/
	// movement
	double m_pSpeed;
};

class SimpleMoveProperties : public Move_Properties
{
public:
	explicit SimpleMoveProperties(double speed, double pSuccess = 1.0);
	~SimpleMoveProperties() = default;
	SimpleMoveProperties(const SimpleMoveProperties &) = default;
	SimpleMoveProperties& operator=(const SimpleMoveProperties&) = default;

	virtual void GetPossibleDirections(int location, DIRECTIONS direction, int target, int gridSize, std::map<DIRECTIONS, double> & possibleDirections) const override;

	virtual std::string String() const override;
private:
	// direction
	double m_pSuccess;
};

class TargetDerivedMoveProperties : public Move_Properties
{
public:
	explicit TargetDerivedMoveProperties(double speed, double pTowardTarget, double pStop, double m_pRandomChangeOfDirection);
	~TargetDerivedMoveProperties() = default;
	TargetDerivedMoveProperties(const TargetDerivedMoveProperties &) = default;
	TargetDerivedMoveProperties& operator=(const TargetDerivedMoveProperties&) = default;

	virtual void GetPossibleDirections(int location, DIRECTIONS direction, int target, int gridSize, std::map<DIRECTIONS, double> & possibleDirections) const override;

	virtual std::string String() const override;
	
private:
	// direction
	double m_pTowardTarget;
	double m_pRandomChangeOfDirection;
	double m_pContinue;
	double m_pStop;
};

class NaiveMoveProperties : public Move_Properties
{
public:
	explicit NaiveMoveProperties(double speed, double m_pRandomChangeOfDirection, double m_pStop);
	~NaiveMoveProperties() = default;
	NaiveMoveProperties(const NaiveMoveProperties &) = default;
	NaiveMoveProperties& operator=(const NaiveMoveProperties&) = default;

	virtual void GetPossibleDirections(int location, DIRECTIONS direction, int target, int gridSize, std::map<DIRECTIONS, double> & possibleDirections) const override;

	virtual std::string String() const override;

private:
	// direction
	double m_pRandomChangeOfDirection;
	double m_pStop;
	double m_pContinue;
};
# endif //MOVE_PROPERTIES_H