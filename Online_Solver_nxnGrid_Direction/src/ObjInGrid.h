#pragma once

#include "Coordinate.h"

/// a base class for objects in grid for pomdp project. can be used as shelter
class ObjInGrid
{
public:
	explicit ObjInGrid() = default;
	explicit ObjInGrid(Coordinate& location);
	virtual ~ObjInGrid() = default;
	ObjInGrid(const ObjInGrid&) = default;
	ObjInGrid& operator=(const ObjInGrid&) = default;

	/// Get location of object
	const Coordinate &GetLocation() const;
	/// Set location of object
	void SetLocation(Coordinate &newLocation);
private:
	Coordinate m_location;
};



