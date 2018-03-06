#include "Self_Obj.h"

Self_Obj::Self_Obj(Coordinate& location, std::shared_ptr<Move_Properties> movement, std::shared_ptr<Attack> attack, std::shared_ptr<Observation> observation)
: Attack_Obj(location, movement, attack)
, m_observation(observation)
{
}