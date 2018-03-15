#include "ObjInGrid.h"

ObjInGrid::ObjInGrid(Coordinate& location)
: m_location(location)
{
}

const Coordinate & ObjInGrid::GetLocation() const
{
	return m_location;
}

void ObjInGrid::SetLocation(Coordinate & newLocation)
{
	m_location = newLocation;
}
