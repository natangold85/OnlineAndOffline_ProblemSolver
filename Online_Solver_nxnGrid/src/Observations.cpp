#include "Observations.h"
#include "Coordinate.h"
#include "Attacks.h"

#include <string>

// constant for 
static double s_SQRT2 = sqrt(2);

ObservationByDistance::ObservationByDistance(double distanceFactor, double nonObserved)
: m_distanceFactor(distanceFactor)
, m_nonObserved(nonObserved)
, m_observationDivergence(1 - nonObserved)
{
}

bool ObservationByDistance::InRange(int selfLoc, int obsObjLoc, int gridSize) const
{
	// observed range is in all grid
	return true;
}

double ObservationByDistance::GetProbObservation(int selfLoc, int objLoc, int gridSize, int observation) const
{
	int nonObsLoc = NonObservedLoc(gridSize);
	// only 2 possibilities for valid observation(non observed = self location)
	if ((observation != objLoc) & (observation != nonObsLoc))
		return 0.0;

	// calculate prob to success observation
	double pSuccess = GetProbSuccessfulObservation(selfLoc, objLoc, gridSize);

	return pSuccess * (observation == objLoc) + (1 - pSuccess) * (observation == nonObsLoc);
}

double ObservationByDistance::GetProbSuccessfulObservation(int selfLoc, int objLoc, int gridSize) const
{
	if (Attack::IsDead(objLoc, gridSize))
	{
		return DeadSuccessfulObservation();
	}

	Coordinate self(selfLoc % gridSize, selfLoc / gridSize);
	Coordinate obj(objLoc % gridSize, objLoc / gridSize);

	// put weight on closer targets
	double dist = self.RealDistance(obj);
	double ratio = dist / (s_SQRT2 * gridSize);

	// calculate prob to success observation
	return ratio * m_distanceFactor + (1 - ratio);
}

double  ObservationByDistance::DeadSuccessfulObservation() const
{
	// TODO : talk to moshe regarding successful observation of death
	return 1.0;
}

void ObservationByDistance::InitObsAvailableLocations(int selfLoc, int objLoc, int gridSize, observableLocations & obsLocations) const
{
	//TODO: until divergence insertion only 2 possible location (observed and non-observed)
	double probSuccess = GetProbSuccessfulObservation(selfLoc, objLoc, gridSize);
	obsLocations.emplace_back(objLoc, probSuccess);

	// insert non-observed location if the object is not dead
	if (probSuccess < 1)
		obsLocations.emplace_back(NonObservedLoc(gridSize), 1 - probSuccess);
}

std::string ObservationByDistance::String() const
{
	std::string ret = "Observation by distance: distance factor = ";
	ret += std::to_string(m_distanceFactor) + ", rest prob: non observed = " + std::to_string(m_nonObserved) + " divergence = " + std::to_string(m_observationDivergence);
	return ret;
}
