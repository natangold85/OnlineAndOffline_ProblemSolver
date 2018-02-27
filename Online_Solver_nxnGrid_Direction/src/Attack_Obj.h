#include <fstream>      // std::ofstream
#include <memory>      // unique_ptr
#include <vector>      // vector

#include "Movable_Obj.h"
#include "Move_Properties.h"
#include "Coordinate.h"
#include "Attacks.h"

#ifndef ATTACK_OBJ_H
#define ATTACK_OBJ_H

/// attack object on grid
class Attack_Obj : public Movable_Obj
{
	using locationVec = std::vector<int>;
public:
	explicit Attack_Obj() = default;
	explicit Attack_Obj(Coordinate& location, std::shared_ptr<Move_Properties> movement, std::shared_ptr<Attack> attack);
	virtual ~Attack_Obj() = default;
	Attack_Obj(const Attack_Obj&) = default;
	Attack_Obj& operator=(const Attack_Obj&) = default;

	// calculations of attacks
	virtual void AttackOnline(locationVec & state, int objIdx, int targetIdx, const locationVec & shelterLoc, int gridSize, double random) const;
	virtual void AttackOffline(const locationVec & state, int objIdx, int targetIdx, const locationVec & shelters, int gridSize, Attack::shootOutcomes & result) const;

	Attack *GetAttack() const  { return m_attack.get(); };

protected:
	std::shared_ptr<Attack> m_attack;
};

# endif //ATTACK_OBJ_H