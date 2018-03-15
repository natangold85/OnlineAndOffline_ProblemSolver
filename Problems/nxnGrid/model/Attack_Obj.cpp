#include "Attack_Obj.h"



Attack_Obj::Attack_Obj(Coordinate& location, std::shared_ptr<Move_Properties> movement, std::shared_ptr<Attack> attack)
: Movable_Obj(location, movement)
, m_attack(attack)
{
}


