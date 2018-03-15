#include "Attack_Obj.h"

Attack_Obj::Attack_Obj(Coordinate& location, std::shared_ptr<Move_Properties> movement, std::shared_ptr<Attack> attack)
: Movable_Obj(location, movement)
, m_attack(attack)
{
}

void Attack_Obj::AttackOnline(locationVec & state, int objIdx, int targetIdx, const locationVec & shelters, int gridSize, double random) const
{
	// calculation of attack
	locationVec prevState(state);
	m_attack->AttackOnline(state, objIdx, targetIdx, shelters, gridSize, random);
	
	// for enemy attack:
	// run on state and compare to previous state if other object than self object killed revive them
	for (int i = 1; i < state.size(); ++i)
	{
		state[i] = state[i] == gridSize * gridSize ? prevState[i] : state[i];
	}
};

void Attack_Obj::AttackOffline(const locationVec & objLocations, int objIdx, int targetIdx, const locationVec & shelters, int gridSize, Attack::shootOutcomes & result) const
{
	// calculation of attack
	m_attack->AttackOffline(objLocations, objIdx, targetIdx, shelters, gridSize, result);
};


