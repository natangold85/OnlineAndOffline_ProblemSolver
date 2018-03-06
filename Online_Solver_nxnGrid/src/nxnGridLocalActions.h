#ifndef NXNGRID_LOCALACTIONS_H
#define NXNGRID_LOCALACTIONS_H

#pragma once
#include <string>

#include "..\include\despot\core\pomdp.h"

#include "nxnGrid.h"
#include "Self_Obj.h"
#include "Coordinate.h"

#include "Move_Properties.h"

namespace despot 
{

/* =============================================================================
* Global Actions class
* =============================================================================*/
/// the class is non thread safe. 
/// the static members are specialized to one type of nxnGrid so only one problem can simultaneously run
class nxnGridLocalActions : public nxnGrid
{	
public:
	///	enum of all basic actions (direction of movement derived from move_properties)

	enum ACTION { NUM_BASIC_ACTIONS = NUM_DIRECTIONS };

	explicit nxnGridLocalActions(int gridSize, int traget, Self_Obj & self, std::vector<intVec> & objectsInitLoc);
	~nxnGridLocalActions() = default;
	
	// Functions for self use

	/// take one step for a given state and action return true if the simulation terminated, update reward and observation
	virtual bool Step(State& s, double random, int action, OBS_TYPE lastObs, double & reward, OBS_TYPE& obs) const override;

	virtual int NumActions() const override;

	/// return the min reward valued action (needed for the despot algorithm)
	virtual ValuedAction GetMinRewardAction() const override { return ValuedAction(NO_DIRECTION, REWARD_MIN); }

private:
	// add actions to specific enemy
	virtual void AddActionsToEnemy() override;
	virtual void AddActionsToShelter() override;

	// ACTIONS FUNCTION:
	/// make move and update state return true if the move is valid (regardless if the move was successful or not) 
	bool MakeMove(DetailedState & state, double random, ACTION action) const;
	/// make attack and update state
	void Attack(DetailedState & state, int target, double random, double & reward) const;

	void MoveToLocation(DetailedState & state, int location, double random) const;

	virtual int EnemyRelatedActionIdx(int action) const override;

	virtual bool EnemyRelatedAction(int action) const override;

	virtual bool LegalAction(const DetailedState & observedState, int action) const override;
};

} // end ns despot

#endif	// NXNGRID_LOCALACTIONS_H