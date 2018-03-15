#include "Move_Properties.h"

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