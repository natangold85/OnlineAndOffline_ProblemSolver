#include "Self_Obj.h"

Self_Obj::Self_Obj(Coordinate& location, std::shared_ptr<Move_Properties> movement, std::shared_ptr<Attack> attack, std::shared_ptr<Observation> observation)
: Attack_Obj(location, movement, attack)
, m_observation(observation)
{
}

void Self_Obj::AttackOnline(locationVec & objLocations, int objIdx, int targetIdx, const locationVec & shelters, int gridSize, double random) const
{
	m_attack->AttackOnline(objLocations, objIdx, targetIdx, shelters, gridSize, random);
}

void Self_Obj::AttackOffline(const locationVec & objLocations, int objIdx, int targetIdx, const locationVec & shelters, int gridSize, Attack::shootOutcomes & result) const
{
	m_attack->AttackOffline(objLocations, objIdx, targetIdx, shelters, gridSize, result);
}