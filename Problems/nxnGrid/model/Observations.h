#ifndef OBSERVATIONS_H
#define OBSERVATIONS_H

#include <vector>

/// abstract observation class
class Observation
{
public:
	using locationProb = std::pair <int, double>;
	using observableLocations = std::vector<locationProb>;

	explicit Observation() = default;
	virtual ~Observation() = default;

	/// return non-observed location
	static int NonObservedLoc(int gridSize) { return gridSize * gridSize + 1; };
	/// return true if location is non-observed location
	static bool IsNonObserved(int location, int gridSize) { return location == NonObservedLoc(gridSize); };

	/// return true if observed location is in range of self object
	virtual bool InRange(int selfLoc, int obsObjLoc, int gridSize) const = 0;
	/// get probability for observation given observing object location(self loc), observed object location(objLoc) grid size and observation
	virtual double GetProbObservation(int selfLoc, int objLoc, int gridSize, int observation) const = 0;
	/// initialize available locations of observation given locations and grid size
	virtual void InitObsAvailableLocations(int selfLoc, int objLoc, int gridSize, observableLocations & observationLocs) const = 0;

	virtual std::string String() const = 0;
};

class ObservationByDistance : public Observation
{
public:
	explicit ObservationByDistance(double distanceFactor, double nonObserved = 1.0);

	virtual bool InRange(int selfLoc, int obsObjLoc, int gridSize) const;
	virtual double GetProbObservation(int selfLoc, int objLoc, int gridSize, int observation) const override;
	virtual void InitObsAvailableLocations(int selfLoc, int objLoc, int gridSize, observableLocations & observationLocs) const override;

	virtual std::string String() const override;
private:
	double GetProbSuccessfulObservation(int selfLoc, int objLoc, int gridSize) const;
	double DeadSuccessfulObservation() const;

	double m_distanceFactor;
	double m_observationDivergence;
	double m_nonObserved;
};

# endif //OBSERVATIONS_H