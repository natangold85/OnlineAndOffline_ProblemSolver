#ifndef ATTACKS_H
#define ATTACKS_H

#include <string>
#include <vector>

#include "Coordinate.h"


/// basic attack for attack object
class Attack
{
public:
	using intVec = std::vector<int>;
	using stateAndProb = std::pair<intVec, double>;
	using shootOutcomes = std::vector<stateAndProb>;

	explicit Attack() = default;
	virtual ~Attack() = default;

	/// return non-observed location
	static int DeadLoc(int gridSize) { return gridSize * gridSize; };
	/// return true if location is non-observed location
	static bool IsDead(int location, int gridSize) { return location == DeadLoc(gridSize); };

	virtual void AttackOnline(intVec & objectsLoc, int attackerLoc, int targetIdx, const intVec & shelters, int gridSize, double random) const = 0;
	virtual void AttackOffline(const intVec & objectsLoc, int attackerLoc, int targetIdx, const intVec & shelters, int gridSize, shootOutcomes & result) const = 0;
	
	virtual bool InRange(int location, int otheObjLocation, int gridSize) const = 0;
	virtual double GetRange() const = 0;
	virtual double GetPHit() const = 0;

	virtual std::string String() const = 0;
};

class DirectAttack : public Attack
{
public:
	DirectAttack(double m_range, double pHit);
	~DirectAttack() = default;

	virtual void AttackOnline(intVec & objectsLoc, int attackerIdx, int targetLoc, const intVec & shelters, int gridSize, double random) const override;
	virtual void AttackOffline(const intVec & objectsLoc, int attackerIdx, int targetLoc, const intVec & shelters, int gridSize, shootOutcomes & result) const override;
	
	virtual bool InRange(int location, int otheObjLocation, int gridSize) const;
	virtual double GetRange() const { return m_range; };
	virtual double GetPHit() const { return m_pHit; };

	virtual std::string String() const override;
private:
	// return result of attacks given attacker, target, locations and grid size
	void CalcAttackResult(const intVec & objectsLoc, int attackerIdx, int targetIdx, const intVec & shelters, int gridSize, intVec & potentialIdxHit) const;

	virtual float CalcAzimuthShoot(Coordinate & attacker, Coordinate & target) const;

	//static void CalcChanges(Coordinate obj1, Coordinate obj2, std::pair<double, double> & change);
	//double CalcDiversion(intVec & state, intVec & shelters, Coordinate & hit, int gridSize, Coordinate & prevShotLocation, shootOutcomes & result) const;

	static bool InFrame(Coordinate point, int gridSize);
	static bool SearchForShelter(intVec shelters, int location);

	double m_range;
	double m_pHit;
	
	static const int NUM_DIVERSIONS = 2;
};

# endif //ATTACKS_H