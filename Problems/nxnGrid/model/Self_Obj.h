#ifndef SELF_OBJ_H
#define SELF_OBJ_H

#include <vector>      // vector

#include "Attack_Obj.h"
#include "Coordinate.h"
#include "Observations.h"

/// class for self object in nxnGrid
class Self_Obj : public Attack_Obj
{
public:
	using intVec = std::vector<int>;

	explicit Self_Obj() = default;
	explicit Self_Obj(Coordinate& location, std::shared_ptr<Move_Properties> movement, std::shared_ptr<Attack> attack, std::shared_ptr<Observation> obs);
	virtual ~Self_Obj() = default;
	Self_Obj(const Self_Obj&) = default;

	Observation *GetObservation() const  { return m_observation.get(); };

private:
	std::shared_ptr<Observation> m_observation;
};

# endif //SELF_OBJ_H