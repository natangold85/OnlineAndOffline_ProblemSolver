#ifndef OBSERVATIONS_H
#define OBSERVATIONS_H

#include <vector>

/// abstract observation class
class Observation
{
public:
	using locationVec = std::vector<int>;

	explicit Observation() = default;
	virtual ~Observation() = default;

	/// return true if observed location is in range of self object
	virtual bool InRange(int selfLoc, int obsObjLoc, int gridSize) const = 0;
	/// get probability for observation given observing object location(self loc), observed object location(objLoc) grid size and observation
	virtual double GetProbObservation(int selfLoc, int objLoc, int gridSize, int observation) const = 0;
	/// get observation given locations, grid size and random number
	virtual int GetObservationObject(int selfLoc, int objLoc, int gridSize, double randomNum) const = 0;
	/// initialize available locations of observation given locations and grid size
	virtual void InitObsAvailableLocations(int selfLoc, int objLoc, const locationVec & state, int gridSize, locationVec & observableLocations) const = 0;

	virtual std::string String() const = 0;

	static int NonObservedLocation(int gridSize);
};

class ObservationByDistance : public Observation
{
public:
	explicit ObservationByDistance(double distanceFactor, double nonObserved = 1.0);

	virtual bool InRange(int selfLoc, int obsObjLoc, int gridSize) const;
	virtual double GetProbObservation(int selfLoc, int objLoc, int gridSize, int observation) const override;
	virtual int GetObservationObject(int selfLoc, int objLoc, int gridSize, double randomNum) const override;
	virtual void InitObsAvailableLocations(int selfLoc, int objLoc, const locationVec & state, int gridSize, locationVec & observableLocations) const override;

	virtual std::string String() const override;
private:
	double m_distanceFactor;
	double m_observationDivergence;
	double m_nonObserved;
};

# endif //OBSERVATIONS_H