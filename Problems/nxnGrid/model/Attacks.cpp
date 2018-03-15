#include <cmath>		//sqrt
#include <algorithm>    // std::sort

#include "Attacks.h"

inline int Abs(int num)
{
	return num * (num >= 0) - num * (num < 0);
}

inline int Sign(int num)
{
	return 1 * (num >= 0) - 1 * (num < 0);
}

inline Coordinate Abs(Coordinate num)
{
	num.X() = Abs(num.X());
	num.Y() = Abs(num.Y());
	return num;
}

inline Coordinate Sign(Coordinate num)
{
	num.X() = Sign(num.X());
	num.Y() = Sign(num.Y());
	return num;
}

inline std::pair<double, double> & operator+=(std::pair<double, double> & a, std::pair<double, double> & b)
{
	a.first += b.first;
	a.second += b.second;
	return a;
}

inline std::pair<double, double> operator+(std::pair<double, double> & a, std::pair<double, double> & b)
{
	std::pair<double, double> ret(a);
	ret.first += b.first;
	ret.second += b.second;

	return ret;
}

inline double Distance(std::pair<double, double> & a, std::pair<double, double> & b)
{
	std::pair<double, double> diff = std::make_pair(a.first - b.first, a.second - b.second);
	return sqrt(diff.first * diff.first + diff.second * diff.second);
}


inline void Swap(int &a, int &b)
{
	int c = a;
	a = b;
	b = c;
}

DirectAttack::DirectAttack(double range, double pHit)
	: m_range(range)
	, m_pHit(pHit)
{
}

void DirectAttack::AttackOnline(intVec & objectsLoc, int attackerIdx, int targetLoc, const intVec & shelters, int gridSize, double random) const
{
	// calculate attack result
	intVec potentialHit;
	CalcAttackResult(objectsLoc, attackerIdx, targetLoc, shelters, gridSize, potentialHit);

	// calculate result according to random
	if (potentialHit.size() == 0)
		return;
	else if (potentialHit.size() == 1)
	{
		if (random < m_pHit)
			objectsLoc[potentialHit[0]] = DeadLoc(gridSize);
	}
	else // hits atleast 2 obj
	{
		// TODO : implement 3 potential hits
		double bothDead = m_pHit * m_pHit;
		double singleDead = m_pHit * (1 - m_pHit);
		if (random < bothDead)
		{
			objectsLoc[potentialHit[0]] = DeadLoc(gridSize);
			objectsLoc[potentialHit[1]] = DeadLoc(gridSize);
		}
		else if (random < bothDead + singleDead)
			objectsLoc[potentialHit[0]] = DeadLoc(gridSize);
		else if (random < bothDead + singleDead * 2)
			objectsLoc[potentialHit[1]] = DeadLoc(gridSize);
	}
}

void DirectAttack::AttackOffline(const intVec & objectsLoc, int attackerIdx, int targetLoc, const intVec & shelters, int gridSize, shootOutcomes & result) const
{
	// calculate outcome of attack
	intVec potentialHit;
	CalcAttackResult(objectsLoc, attackerIdx, targetLoc, shelters, gridSize, potentialHit);

	if (potentialHit.size() == 0)
		result.emplace_back(objectsLoc, 1);
	else if (potentialHit.size() == 1)
	{
		result.emplace_back(objectsLoc, 1 - m_pHit);
		
		intVec locWithDead(objectsLoc);
		locWithDead[potentialHit[0]] = DeadLoc(gridSize);
		
		result.emplace_back(locWithDead, m_pHit);
	}
	else // TODO calculate more than two hits possible in one location
	{
		// insert prob of 4 options of hitting 2 objects
		double miss = 1 - m_pHit;

		result.emplace_back(objectsLoc, miss * miss);
		
		intVec deadLoc(objectsLoc);
		deadLoc[potentialHit[0]] = DeadLoc(gridSize);
		result.emplace_back(deadLoc, miss * m_pHit);
		
		deadLoc[potentialHit[1]] = DeadLoc(gridSize);
		result.emplace_back(deadLoc, m_pHit * m_pHit);
		
		deadLoc[potentialHit[0]] = objectsLoc[potentialHit[0]];
		result.emplace_back(deadLoc, miss * m_pHit);
	}
}

bool DirectAttack::InRange(int location, int otherObjLocation, int gridSize) const
{
	Coordinate self(location % gridSize, location / gridSize);
	Coordinate object(otherObjLocation % gridSize, otherObjLocation / gridSize);

	return self.RealDistance(object) <= m_range;
}

std::string DirectAttack::String() const
{
	std::string ret = "Direct Attack: range = ";
	ret += std::to_string(m_range) + " pHit = " + std::to_string(m_pHit);
	return ret;
}

void DirectAttack::CalcAttackResult(const intVec & objectsLoc, int attackerIdx, int targetLoc, const intVec & shelters, int gridSize, intVec & potentialIdxHit) const
{
	Coordinate attacker(objectsLoc[attackerIdx] % gridSize, objectsLoc[attackerIdx] / gridSize);
	Coordinate target(targetLoc % gridSize, targetLoc / gridSize);

	float azimuth = CalcAzimuthShoot(attacker, target);
	float cosAzimuth = cos(azimuth);
	float sinAzimuth = sin(azimuth);

	std::pair<double, double> currFireLocation(attacker.X(), attacker.Y());
	Coordinate fireRoundedLocation(attacker);
	int idxLocation = fireRoundedLocation.GetIdx(gridSize);

	bool isHit = false;
	while (!isHit)
	{
		// run on all objects and see if one of the object is on the path of fire
		for (int i = 0; i < objectsLoc.size(); ++i)
		{
			if (idxLocation == objectsLoc[i] && i != attackerIdx)
			{
				potentialIdxHit.emplace_back(i);
				isHit = true;
			}
		}

		currFireLocation.first += cosAzimuth;
		currFireLocation.second += sinAzimuth;

		fireRoundedLocation.X() = round(currFireLocation.first);
		fireRoundedLocation.Y() = round(currFireLocation.second);
		// if arrived to the limits of map attack failed
		if (!fireRoundedLocation.ValidLocation(gridSize) || attacker.RealDistance(fireRoundedLocation) > m_range)
			return;

		idxLocation = fireRoundedLocation.GetIdx(gridSize);
		// if arrived to shelter, shelter absorbed the bullet and attack failed (until entered option for more complex shelters) (because current location is not protected by shelter calculate shelter location at the end of loop just after change in idx)
		for (auto s : shelters)
		{
			if (idxLocation == s)
				return;
		}
	}
}

float DirectAttack::CalcAzimuthShoot(Coordinate & attacker, Coordinate & target) const
{
	Coordinate attackVector(target - attacker);

	float desiredRadian = atan2(attackVector.Y(), attackVector.X());
	return desiredRadian;
}

bool DirectAttack::InFrame(Coordinate point, int gridSize)
{
	return (point.X() >= 0) & (point.X() < gridSize) & (point.Y() >= 0) & (point.Y() < gridSize);
}

bool DirectAttack::SearchForShelter(intVec shelters, int location)
{
	for (auto v : shelters)
	{
		if (v == location)
			return true;
	}

	return false;
}

