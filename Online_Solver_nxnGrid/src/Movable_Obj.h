#ifndef MOVABLE_OBJ_H
#define MOVABLE_OBJ_H

#include <memory>      // shared_ptr

#include "ObjInGrid.h"
#include "Move_Properties.h"
#include "Coordinate.h"

///class of movable objects in grid for pomdp writer. can be use as non-involved objects
/// non copyable object
class Movable_Obj : public ObjInGrid
{
public:
	explicit Movable_Obj() = default;
	explicit Movable_Obj(Coordinate& location, std::shared_ptr<Move_Properties> movement);
	virtual ~Movable_Obj() = default;
	Movable_Obj(const Movable_Obj &) = default;
	Movable_Obj& operator=(const Movable_Obj&) = default;

	///return object movement properties
	const std::shared_ptr<Move_Properties>& GetMovement() const { return m_movement; }
	///change object movement prop
	void SetMovement(std::shared_ptr<Move_Properties> m) { m_movement = m; }

private:
	std::shared_ptr<Move_Properties> m_movement;
};

# endif //MOVABLE_OBJ_H