#ifndef MOVE_PROPERTIES_H
#define MOVE_PROPERTIES_H

#include <map>
#include <string>

#include "Coordinate.h"

// directions available for moves
enum DIRECTIONS { SOUTH, NORTH, EAST, WEST, SOUTH_EAST, NORTH_EAST, SOUTH_WEST, NORTH_WEST, NO_DIRECTION, NUM_DIRECTIONS };

///properties of movement for object on grid
class Move_Properties
{
public:
	using intVec = std::vector<int>;

	/*LUT*/
	static std::vector<Coordinate> s_directionsLUT;
	static std::vector<std::string> s_directionNamesLUT;

	static std::vector<Coordinate> InitDirectionsLUT();
	static std::vector<std::string> InitDirectionsNamesLUT();
protected:

};


# endif //MOVE_PROPERTIES_H