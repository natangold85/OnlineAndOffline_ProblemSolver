#ifndef ATTACKS_H
#define ATTACKS_H


#include <string>
#include <vector>

#include "Coordinate.h"


/// basic attack for attack object
class Attack
{
public:
	using locationVec = std::vector<int>;
	using stateAndProb = std::pair<locationVec, double>;
	using shootOutcomes = std::vector<stateAndProb>;

	explicit Attack() = default;
	virtual ~Attack() = default;

	virtual void AttackOnline(locationVec & objectsLoc, int attackerIdx, int targetIdx, const locationVec & shelters, int gridSize, double random) const = 0;
	virtual void AttackOffline(const locationVec & objectsLoc, int attackerIdx, int targetIdx, const locationVec & shelters, int gridSize, shootOutcomes & result) const = 0;
	
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

	virtual void AttackOnline(locationVec & objectsLoc, int attackerIdx, int targetIdx, const locationVec & shelters, int gridSize, double random) const override;
	virtual void AttackOffline(const locationVec & objectsLoc, int attackerIdx, int targetIdx, const locationVec & shelters, int gridSize, shootOutcomes & result) const override;
	
	virtual bool InRange(int location, int otheObjLocation, int gridSize) const;
	virtual double GetRange() const { return m_range; };
	virtual double GetPHit() const { return m_pHit; };

	virtual std::string String() const override;
private:
	// return result of attacks given attacker, target, locations and grid size
	void CalcAttackResult(const locationVec & objectsLoc, int attackerIdx, int targetIdx, const locationVec & shelters, int gridSize, locationVec & potentialIdxHit) const;

	virtual float CalcAzimuthShoot(Coordinate & attacker, Coordinate & target) const;

	static bool InFrame(Coordinate point, int gridSize);
	static bool SearchForShelter(locationVec shelters, int location);

	double m_range;
	double m_pHit;
};

class DirectAttackWithNoise : public DirectAttack
{
public:
	DirectAttackWithNoise(double m_range, double pHit, double radNoise);
	~DirectAttackWithNoise() = default;

	virtual std::string String() const override;
private:

	float CalcAzimuthShoot(Coordinate & attacker, Coordinate & target) const;
	float RadNoise() const;

	double m_radNoise;
};

# endif //ATTACKS_H