#include <string>
#include <math.h>



#include "nxnGrid.h"
#include "Coordinate.h"

namespace despot 
{


// init static members

// state params
int nxnGridDetailedState::s_numNonInvolved = 0;
int nxnGridDetailedState::s_numEnemies = 0;

int nxnGridDetailedState::s_gridSize = 0;
int nxnGridDetailedState::s_targetLoc = 0;

std::vector<int> nxnGridDetailedState::s_shelters;


// model params
std::vector<std::string> nxnGrid::s_actionsStr;
int nxnGrid::s_lutGridSize = -1;

std::vector<intVec> nxnGrid::s_objectsInitLocations;


// rewards
const int OnlineSolverModel::REWARD_WIN = 50.0;
const int OnlineSolverModel::REWARD_LOSS = -100.0;
const int nxnGrid::REWARD_KILL_ENEMY = 0.0;
const int nxnGrid::REWARD_KILL_NINV = REWARD_LOSS;
const int nxnGrid::REWARD_ILLEGAL_MOVE = 0;
const int nxnGrid::REWARD_MAX = REWARD_WIN;
const int nxnGrid::REWARD_MIN = min(REWARD_LOSS, REWARD_ILLEGAL_MOVE);
const int nxnGrid::REWARD_STEP = 0;
const int nxnGrid::REWARD_FIRE = 0;

// for lut
OnlineSolverModel::OnlineSolverLUT nxnGrid::s_LUT;
enum nxnGrid::CALCULATION_TYPE nxnGrid::s_calculationType = nxnGrid::WITHOUT;


/* =============================================================================
* nxnGridState Functions
* =============================================================================*/

double DetailedState::ObsProb(STATE_TYPE state_id, OBS_TYPE obs)
{
	return -1;
}

nxnGridDetailedState::nxnGridDetailedState()
: m_locations()
, m_isEnemyObserved()
{
}

nxnGridDetailedState::nxnGridDetailedState(unsigned int sizeState)
: m_locations(sizeState)
, m_isEnemyObserved(s_numEnemies)
{
}

nxnGridDetailedState::nxnGridDetailedState(const State & state)
: nxnGridDetailedState(state.state_id)
{
}

nxnGridDetailedState::nxnGridDetailedState(STATE_TYPE stateId)
: m_locations()
, m_isEnemyObserved()
{
	InitLocationsFromId(stateId, m_locations);
	InitIsObservedBoolFromId(stateId, m_isEnemyObserved);
}

void nxnGridDetailedState::InitLocationsFromId(STATE_TYPE & state_id, intVec & locations)
{
	int numObjects = 1 + s_numEnemies + s_numNonInvolved;
	locations.resize(numObjects);

	// retrieve each location according to s_NUM_BITS_LOCATION
	int bitsForSinglelocation = (1 << s_NUM_BITS_LOCATION) - 1;
	for (int obj = 0; obj < numObjects; ++obj)
	{
		locations[obj] = state_id & bitsForSinglelocation;
		state_id >>= s_NUM_BITS_LOCATION;
	}
}

void nxnGridDetailedState::InitIsObservedBoolFromId(STATE_TYPE & state_id, boolVec & isObsVec)
{
	// assumption : is observed is 1 bit per object
	isObsVec.resize(s_numEnemies);
	int currBit = 1;
	// retrieve enemy observed locations
	for (int e = 0; e < s_numEnemies; ++e)
	{
		isObsVec[e] = state_id & currBit;
		state_id >>= 1;
	}
}

int nxnGridDetailedState::GetObjLocation(STATE_TYPE state_id, int objIdx)
{
	int locationBits = (1 << s_NUM_BITS_LOCATION) - 1;
	state_id >>= objIdx * s_NUM_BITS_LOCATION;

	return state_id & locationBits;
}

bool nxnGridDetailedState::IsEnemyObserved(OBS_TYPE state_id, int enemyIdx)
{
	int numObj = 1 + s_numEnemies + s_numNonInvolved;
	
	state_id >>= numObj * s_NUM_BITS_LOCATION;
	state_id >>= enemyIdx;
	return state_id & 1;
}

void nxnGridDetailedState::GetEnemyObservedVec(OBS_TYPE obs_id, boolVec & enemyObsVec)
{
	int numObj = 1 + s_numEnemies + s_numNonInvolved;
	obs_id >>= numObj * s_NUM_BITS_LOCATION;

	InitIsObservedBoolFromId(obs_id, enemyObsVec);
}

STATE_TYPE nxnGridDetailedState::GetStateId() const
{
	// from end to start
	STATE_TYPE stateId = 0;
	for (int e = 0; e < s_numEnemies; ++e)
	{
		stateId <<= 1;
		stateId |= m_isEnemyObserved[e] & 1;
	}

	int numLocationbits = (1 << s_NUM_BITS_LOCATION) - 1;
	for (int obj = m_locations.size() - 1; obj >= 0; --obj)
	{
		stateId <<= s_NUM_BITS_LOCATION;
		stateId |= m_locations[obj] & numLocationbits;
	}

	return stateId;
}

OBS_TYPE nxnGridDetailedState::GetObsId() const
{
	return static_cast<OBS_TYPE>(GetStateId());
}

double nxnGridDetailedState::ObsProb(STATE_TYPE state_id, OBS_TYPE obs, int gridSize, const Observation & obsType)
{
	nxnGridDetailedState state(state_id);
	nxnGridDetailedState obsState(obs);

	// if observation is not including the location of the robot return 0
	if (state.m_locations[0] != obsState.m_locations[0])
		return 0.0;

	double pObs = 1.0;
	// run on all non-self objects location
	for (int i = 1; i < s_numEnemies + s_numNonInvolved; ++i)
	{
		// create possible observation of obj location
		Observation::observableLocations observableLocations;
		obsType.InitObsAvailableLocations(state.m_locations[0], state.m_locations[i], gridSize, observableLocations);

		// run on possible observable location
		bool isObserved = false;
		for (auto obsLoc : observableLocations)
		{
			if (obsLoc.first == obsState.m_locations[i])
			{
				pObs *= obsLoc.second;
				isObserved = true;
			}
		}

		// if the object is not in observable locations return 0
		if (!isObserved)
			return 0.0;
	}

	return pObs;
}

double nxnGridDetailedState::ObsProbOneObj(STATE_TYPE state_id, OBS_TYPE obs, int objIdx, int gridSize, const Observation & obsType)
{
	int selfLoc = GetObjLocation(state_id, 0);
	
	if (objIdx == 0)
		return selfLoc == GetObjLocation(obs, 0);

	int objLoc = GetObjLocation(state_id, objIdx);
	int observedLoc = GetObjLocation(obs, objIdx);
	double pObs = 0.0;

	// create possible observation of obj location
	Observation::observableLocations observableLocations;
	// send location of self of observed state for non observable location(only important location of object not self)
	obsType.InitObsAvailableLocations(selfLoc, objLoc, gridSize, observableLocations);

	// run on possible observable location
	for (auto obsLoc : observableLocations)
	{
		if (obsLoc.first == observedLoc)
			return obsLoc.second;
	}

	return 0.0;
}

void nxnGridDetailedState::InitStatic()
{
	s_numNonInvolved = 0;
	s_numEnemies = 0;
	s_gridSize = 0;
	s_shelters.resize(0);
}

STATE_TYPE nxnGridDetailedState::MaxState()
{
	int numObjects = 1 + s_numEnemies + s_numNonInvolved;
	int numBits = s_NUM_BITS_LOCATION * numObjects;

	STATE_TYPE ret = 1;
	return ret << numBits;
}

char nxnGridDetailedState::ObjIdentity(int location) const
{
	if (m_locations[0] == location)
		return 'M';

	int o = 1;

	for (; o < s_numEnemies + 1; ++o)
	{
		if (m_locations[o] == location)
		{
			if (m_isEnemyObserved[o - 1])
				return 'E';
			else
				return 'e';
		}
	}

	int toEndOfNonInv = o + s_numNonInvolved;
	for (o; o < toEndOfNonInv; ++o)
	{
		if (m_locations[o] == location)
			return 'N';
	}

	for (int s = 0; s < s_shelters.size(); ++s)
	{
		if (location == s_shelters[s])
			return 'S';
	}

	if (location == s_targetLoc)
		return 'T';

	return '_';
}

void nxnGridDetailedState::EraseNonInv()
{
	int startNonInv = 1 + s_numEnemies;
	for (int n = 0; n < s_numNonInvolved; ++n)
		EraseObject(startNonInv);
}

void nxnGridDetailedState::EraseObject(int objectIdx)
{
	m_locations.erase(m_locations.begin() + objectIdx);
}

bool nxnGridDetailedState::IsProtected(int objIdx) const
{
	bool ret = false;
	for (auto shelter : s_shelters)
		ret |= shelter == m_locations[objIdx];

	return ret;
}

std::string nxnGridDetailedState::text() const
{
	std::string ret = "(";
	for (int i = 0; i < m_locations.size(); ++i)
	{
		ret += std::to_string(m_locations[i]) + ", ";
	}
	ret += ")\n";

	PrintGrid(ret);
	return ret;
}

void nxnGridDetailedState::PrintGrid(std::string & buffer) const
{
	for (int y = 0; y < s_gridSize; ++y)
	{
		for (int x = 0; x < s_gridSize; ++x)
		{
			int loc = x + y * s_gridSize;
			buffer += ObjIdentity(loc);
		}
		buffer += "\n";
	}
}

bool nxnGridDetailedState::NoEnemies(int gridSize) const
{
	for (int e = 0; e < s_numEnemies; ++e)
		if (!Attack::IsDead(m_locations[e + 1], gridSize))
			return false;

	return true;
}

bool nxnGridDetailedState::IsNonInvDead(int gridSize) const
{
	bool anyDead = false;
	int offsetToNonInv = 1 + s_numEnemies;
	for (int n = 0; n < s_numNonInvolved && !anyDead; ++n)
		anyDead = Attack::IsDead(m_locations[offsetToNonInv + n], gridSize);

	return anyDead;
}

/* =============================================================================
* nxnGrid Functions
* =============================================================================*/

void OnlineSolverModel::ChoosePreferredAction(POMCPPrior * prior, const DSPOMDP* m, doubleVec & expectedRewards)
{
	static_cast<const nxnGrid *>(m)->ChoosePreferredAction(prior, expectedRewards);
}

int OnlineSolverModel::ChoosePreferredAction(POMCPPrior * prior, const DSPOMDP* m, double & expectedReward)
{
	return static_cast<const nxnGrid *>(m)->ChoosePreferredAction(prior, expectedReward);
}

void OnlineSolverModel::InsertState2Buffer(uintVec & buffer, State * state)
{
	nxnGridDetailedState s(state->state_id);
	
	for (int i = 0; i < s.size(); ++i)
	{
		buffer.emplace_back(s[i]);
	}
}

nxnGrid::nxnGrid(int gridSize, int target, Self_Obj & self, std::vector<intVec> & objectsInitLoc)
	: m_gridSize(gridSize)
	, m_targetIdx(target)
	, m_self(self)
	, m_enemyVec()
	, m_shelters()
	, m_nonInvolvedVec()
{
	// init size  of state for nxnGridstate
	nxnGridDetailedState::s_gridSize = gridSize;
	nxnGridDetailedState::s_targetLoc = target;

	s_objectsInitLocations = objectsInitLoc;
}


void nxnGrid::InitLUT(OnlineSolverLUT & offlineLut, int offlineGridSize, CALCULATION_TYPE ctype)
{
	s_calculationType = ctype;
	s_lutGridSize = offlineGridSize;
	s_LUT = offlineLut;
}

int nxnGrid::ChoosePreferredAction(POMCPPrior * prior, double & expectedReward) const
{
	if (nxnGrid::getLUTCalcType() != nxnGrid::WITHOUT)
	{
		doubleVec rewards;
		ChoosePreferredAction(prior, rewards);
		// if calc type != without return lut result else return random decision
		return OnlineSolverModel::FindMaxReward(rewards, expectedReward);
	}
	else
	{
		// unknown reward for random action	
		expectedReward = REWARD_LOSS;
		return randLegalAction(prior->history().LastObservation());
	}
}

void nxnGrid::ChoosePreferredAction(POMCPPrior * prior, doubleVec & expectedRewards) const
{
	if (prior->history().Size() > 0)
	{
		nxnGridDetailedState beliefState(CountMovingObjects());
		InitBeliefState(beliefState, prior->history());
		ChoosePreferredActionIMP(beliefState, expectedRewards);
	}
	else
	{
		expectedRewards = doubleVec(NumActions(), REWARD_LOSS);
		// TODO : when implementing illegal actions initialize this vector
	}
}

void nxnGrid::AddObj(Attack_Obj&& obj)
{
	m_enemyVec.emplace_back(std::forward<Attack_Obj>(obj));
	nxnGridDetailedState::s_numEnemies = m_enemyVec.size();

	AddActionsToEnemy();
}

void nxnGrid::AddObj(Movable_Obj&& obj)
{
	m_nonInvolvedVec.emplace_back(std::forward<Movable_Obj>(obj));

	nxnGridDetailedState::s_numNonInvolved = m_nonInvolvedVec.size();

}

void nxnGrid::AddObj(ObjInGrid&& obj)
{
	m_shelters.emplace_back(std::forward<ObjInGrid>(obj));
	
	nxnGridDetailedState::s_shelters.emplace_back(obj.GetLocation().GetIdx(m_gridSize));

	AddActionsToShelter();
}

double nxnGrid::ObsProb(OBS_TYPE obs, const State& state, int action) const
{
	return nxnGridDetailedState::ObsProb(state.state_id, obs, m_gridSize, *m_self.GetObservation());
}

double nxnGrid::ObsProbOneObj(OBS_TYPE obs, const State & s, int action, int objIdx) const
{
	return nxnGridDetailedState::ObsProbOneObj(s.state_id, obs, objIdx, m_gridSize, *m_self.GetObservation());
}

void nxnGrid::CreateParticleVec(std::vector<std::vector<std::pair<int, double> > > & objLocations, std::vector<State*> & particles) const
{
	nxnGridDetailedState state(CountMovingObjects());
	CreateParticleVecRec(state, objLocations, particles, 1.0, 0);
}

void nxnGrid::ChoosePreferredActionIMP(const nxnGridDetailedState & beliefState, doubleVec & expectedReward) const
{
	OnlineSolverLUT::iterator itr, itr2;

	
	switch (s_calculationType)
	{
	case ALL:
	{
		nxnGridDetailedState scaledState(beliefState.size());

		ScaleState(beliefState, scaledState);
		itr = s_LUT.find(scaledState.GetStateId());
		if (itr != s_LUT.end())
			expectedReward = itr->second;
		else
			expectedReward = doubleVec(NumActions(), REWARD_LOSS);
		break;
	}
	case WO_NINV:
	{
		// erase all non involved
		nxnGridDetailedState woNInv(beliefState);
		woNInv.EraseNonInv();

		nxnGridDetailedState scaledState(woNInv.size());
		ScaleState(woNInv, scaledState);
		itr = s_LUT.find(scaledState.GetStateId());
		if (itr != s_LUT.end())
			expectedReward = itr->second;
		else
			expectedReward = doubleVec(NumActions(), REWARD_LOSS);
		break;
	}
	case JUST_ENEMY:
	{
		// erase all non involved
		nxnGridDetailedState woNInv(beliefState);
		woNInv.EraseNonInv();
		nxnGridDetailedState scaledState(woNInv.size());

		ScaleState(woNInv, scaledState);
		itr = s_LUT.find(scaledState.GetStateId());
		if (itr != s_LUT.end())
			expectedReward = itr->second;
		else
			expectedReward = doubleVec(NumActions(), REWARD_LOSS);
		break;
	}

	case ONE_ENEMY:
	{
		// erase all non involved
		nxnGridDetailedState firstEnemy(beliefState);
		firstEnemy.EraseNonInv();
		nxnGridDetailedState secondEnemy(firstEnemy);

		firstEnemy.EraseObject(1);
		secondEnemy.EraseObject(2);
		nxnGridDetailedState scaledState(firstEnemy.size());

		ScaleState(firstEnemy, scaledState);
		itr = s_LUT.find(scaledState.GetStateId());

		// calculate reward with second enemy
		ScaleState(secondEnemy, scaledState);
		itr2 = s_LUT.find(scaledState.GetStateId());

		if (itr != s_LUT.end() & itr2 != s_LUT.end())
			Combine2EnemiesRewards(beliefState.m_locations, itr->second, itr2->second, expectedReward);
		else
			expectedReward = doubleVec(NumActions(), REWARD_LOSS);
		break;
	}
	
	case WO_NINV_STUPID:
	{
		// TODO : change implementation of stupid alg
		//beliefState.erase(beliefState.begin() + 1 + m_enemyVec.size());
		//scaledState.resize(beliefState.size());
		//ScaleState(beliefState, scaledState);
		//itr = s_LUT.find(nxnGridState::StateToIdx(scaledState, s_lutGridSize));
		//
		//// TODO : move members 1 slot right
		//if (itr != s_LUT.end())
		//	expectedReward = itr->second;
		//else
		//	expectedReward = doubleVec(NumActions(), REWARD_LOSS);
		break;
	}
	default: // calc type = WITHOUT
		expectedReward = doubleVec(NumActions(), REWARD_LOSS);
		OBS_TYPE obs(beliefState.GetObsId());

		if (REWARD_ILLEGAL_MOVE < REWARD_LOSS)
		{
			for (int a = 0; a < NumActions(); ++a)
			{
				if (!LegalAction(obs, a))
					expectedReward[a] = REWARD_ILLEGAL_MOVE;
			}
		}
		break;
	}
}

void nxnGrid::CreateParticleVecRec(nxnGridDetailedState & state, std::vector<std::vector<std::pair<int, double> > > & objLocations, std::vector<State*> & particles, double weight, int currIdx) const
{
	if (currIdx == state.size())
	{
		auto beliefState = static_cast<OnlineSolverState*>(Allocate(state.GetStateId(), weight));
		particles.push_back(beliefState);
	}
	else
	{
		for (auto loc : objLocations[currIdx])
		{
			state[currIdx] = loc.first;
			CreateParticleVecRec(state, objLocations, particles, weight * loc.second, currIdx + 1);
		}
	}
}

State * nxnGrid::CreateStartState(std::string type) const
{
	nxnGridDetailedState state(CountMovingObjects());

	// insert locations, and set enemy obs bool to flase
	state[0] = m_self.GetLocation().GetIdx(m_gridSize);
	
	for (int i = 0; i < m_enemyVec.size(); ++i)
	{
		state[i + 1] = m_enemyVec[i].GetLocation().GetIdx(m_gridSize);
		state.IsEnemyObserved(i, false);
	}

	for (int i = 0; i < m_nonInvolvedVec.size(); ++i)
		state[i + 1 + m_enemyVec.size()] = m_nonInvolvedVec[i].GetLocation().GetIdx(m_gridSize);

		
	return new OnlineSolverState(state.GetStateId());
}

Belief * nxnGrid::InitialBelief(const State * start, std::string type) const
{
	std::vector<State*> particles;
	nxnGridDetailedState state(CountMovingObjects());

	// create belief states given self location
	state[0] = m_self.GetLocation().GetIdx(m_gridSize);
	int numStates = 1;
	// assumption : init states cannot be in same locations for different object
	for (int i = 1; i < CountMovingObjects(); ++i)
		numStates *= s_objectsInitLocations[i].size();

	double stateProb = 1.0 / numStates;
	
	// set enemy obs falg to false
	for (int e = 0; e < m_enemyVec.size(); ++e)
		state.IsEnemyObserved(e, false);

	InitialBeliefStateRec(state, 1, stateProb, particles);

	if (type == "nxnGridBelief")
		return new nxnGridBelief(particles, this);
	else
		return new ParticleBelief(particles, this);
}

void nxnGrid::DisplayParameters(std::ofstream & out) const
{
	out << "\n\nModel Details:\n";
	out << "grid size: " + std::to_string(m_gridSize) + "  target idx: " + std::to_string(m_targetIdx);
	out << "\n\nSELF:\nself possible initial locations: (";
	for (auto loc : s_objectsInitLocations[0])
		out << loc << ", ";
	out << ")";
	out << "\nwith move properties: " + m_self.GetMovement()->String();
	out << "\nwith attack: " + m_self.GetAttack()->String();
	out << "\nwith observation: " + m_self.GetObservation()->String();
	
	out << "\n\nENEMIES:\n";
	for (int i = 0; i < m_enemyVec.size(); ++i)
	{
		out << "\nenemy #" << i;
		out << "\nobject possible initial locations : (";
		for (auto loc : s_objectsInitLocations[i + 1])
			out << loc << ", ";
		out << ")";
		out << "\nwith move properties: " + m_enemyVec[i].GetMovement()->String();
		out << "\nwith attack: " + m_enemyVec[i].GetAttack()->String();
	}

	out << "\n\nNON-INVOLVED:\n";
	for (int i = 0; i < m_nonInvolvedVec.size(); ++i)
	{
		out << "\nnon-involved #" << i;
		out << "\nobject possible initial locations : (";
		for (auto loc : s_objectsInitLocations[i + 1 + m_enemyVec.size()])
			out << loc << ", ";
		out << ")";
		out << "\nwith move properties: " + m_nonInvolvedVec[i].GetMovement()->String();
	}

	out << "\n\nSHELTERS:\n";
	for (int i = 0; i < m_shelters.size(); ++i)
	{
		out << "\nshelter #" << i;
		out << "\nobject possible initial locations : (";
		for (auto loc : s_objectsInitLocations[i + 1 + m_enemyVec.size() + m_nonInvolvedVec.size()])
			out << loc << ", ";
		out << ")";
	}
}

void nxnGrid::PrintState(const State & s, std::ostream & out) const
{
	nxnGridDetailedState state(s.state_id);
	out << state.text() << "\n";
}

void nxnGrid::PrintBelief(const Belief & belief, std::ostream & out) const
{
}

void nxnGrid::PrintObs(const State & state, OBS_TYPE obs, std::ostream & out) const
{
	nxnGridDetailedState obsState(obs);

	std::string txt = "(";
	for (int i = 0; i < obsState.size(); ++i)
	{
		txt += std::to_string(obsState[i]) + ", ";
	}
	txt += ")\n";

	out << txt << "\n";
}

void nxnGrid::PrintAction(int action, std::ostream & out) const
{
	out << s_actionsStr[action] << std::endl;
}

bool nxnGrid::InRange(int locationSelf, int locationObj, double range, int gridSize)
{
	Coordinate self(locationSelf % gridSize, locationSelf / gridSize);
	Coordinate enemy(locationObj % gridSize, locationObj / gridSize);

	// return true when distance is <= from range
	return self.RealDistance(enemy) <= range;
}

bool nxnGrid::CalcIfKilledByEnemy(const nxnGridDetailedState & state, int enemyIdx, double & randomNum) const
{
	bool isSelfDead = false;
	int enemyStateIdx = enemyIdx + 1;
	if (m_enemyVec[enemyIdx].GetAttack()->InRange(state[enemyStateIdx], state[0], m_gridSize))
	{
		intVec shelters(GetSheltersVec());
		intVec objLocations(state.LocationVec());

		m_enemyVec[enemyIdx].GetAttack()->AttackOnline(objLocations, enemyStateIdx, objLocations[0], shelters, m_gridSize, randomNum);
		// assumption : other objects beside self cannot be killed by enemies
		isSelfDead = Attack::IsDead(objLocations[0], m_gridSize);
	}
	return isSelfDead;
}

void nxnGrid::MoveNonProtectedShelters(const intVec & beliefState, intVec & scaledState, int oldGridSize, int newGridSize) const
{
	int startShelter = 1 + NumEnemiesInCalc() + NumNonInvInCalc();
	for (int s = 0; s < m_shelters.size(); ++s)
	{
		int shelterIdx = s + startShelter;
		
		for (int o = 0; o < startShelter; ++o)
		{
			if ((scaledState[o] == scaledState[shelterIdx]) & (beliefState[o] != beliefState[shelterIdx] & !Attack::IsDead(scaledState[o], newGridSize)))
			{
				MoveObjectLocation(beliefState, scaledState, o, oldGridSize, newGridSize);
			}
		}
	}
}

void nxnGrid::MoveObjectLocation(const intVec & beliefState, intVec & scaledState, int objIdx, int oldGridSize, int newGridSize) const
{
	// TODO : change ScaleState after allowing repetitions and using detailedstate

	//Coordinate objectRealLoc(beliefState[objIdx] % oldGridSize, beliefState[objIdx] / oldGridSize);
	//Coordinate objectLoc(scaledState[objIdx] % newGridSize, scaledState[objIdx] / newGridSize);
	//
	//int minDist = 100000;
	//int bestLocation = -1;
	//double scaleBackward = static_cast<double>(newGridSize) / oldGridSize;

	//// run on all possible moves of enemy and calculate which valid location is closest to object
	//for (size_t i = 0; i < s_numMoves; ++i)
	//{
	//	Coordinate newLoc(s_lutDirections[i][0], s_lutDirections[i][1]);
	//	newLoc += objectLoc;
	//	if (ValidLegalLocation(scaledState, newLoc, objIdx, newGridSize))
	//	{
	//		newLoc /= scaleBackward;
	//		int dist = newLoc.Distance(objectRealLoc);
	//		if (dist < minDist)
	//		{
	//			minDist = dist;
	//			bestLocation = i;
	//		}
	//	}
	//}
	//
	//// TODO: see whats happenning to shelter after the moving of the object
}

void nxnGrid::ShiftSelfFromTarget(const intVec & beliefState, intVec & scaledState, int oldGridSize, int newGridSize) const
{
	Coordinate realSelf(beliefState[0] % oldGridSize, beliefState[0] / oldGridSize);
	
	// if x > y shift on y axis else shift object on x axis
	if (realSelf.X() > realSelf.Y())
	{
		for (int i = 0; i < scaledState.size(); ++i)
		{
			if (Attack::IsDead(scaledState[i], newGridSize))
				continue;

			int yLoc = scaledState[i] / newGridSize - 1;

			// if the object is out of grid drop him from calculation
			if (yLoc < 0)
			{
				scaledState[i] = Attack::DeadLoc(newGridSize);
			}
			else
			{
				scaledState[i] -= newGridSize;
			}
		}
	}
	else
	{
		for (int i = 0; i < scaledState.size(); ++i)
		{
			if (Attack::IsDead(scaledState[i], newGridSize))
				continue;

			int xLoc = scaledState[i] % newGridSize - 1;

			// if the object is out of grid drop him from calculation
			if (xLoc < 0)
			{
				scaledState[i] = Attack::DeadLoc(newGridSize);
			}
			else
			{
				--scaledState[i];
			}
		}
	}
}

int nxnGrid::NumEnemiesInCalc() const
{
	// TODO : make this function more modular
	return m_enemyVec.size() * (s_calculationType == WO_NINV) + 1 * (s_calculationType == ONE_ENEMY);
}

int nxnGrid::NumNonInvInCalc() const 
{
	// TODO : make this function more modular
	return 0;
}

enum nxnGrid::OBJECT nxnGrid::WhoAmI(int objIdx) const
{
	return objIdx == 0 ? SELF :
		objIdx < m_enemyVec.size() + 1 ? ENEMY :
		objIdx < m_nonInvolvedVec.size() + m_enemyVec.size() + 1 ? NON_INV :
		objIdx < m_shelters.size() + m_nonInvolvedVec.size() + m_enemyVec.size() + 1 ? SHELTER : TARGET;
}

void nxnGrid::InitialBeliefStateRec(nxnGridDetailedState & state, int currObj, double stateProb, std::vector<State*> & particles) const
{
	if (currObj == CountMovingObjects())
	{
		auto beliefState = static_cast<OnlineSolverState*>(Allocate(state.GetStateId(), stateProb));
		particles.push_back(beliefState);
	}
	else
	{
		for (auto obj : s_objectsInitLocations[currObj])
		{
			state[currObj] = obj;
			InitialBeliefStateRec(state, currObj + 1, stateProb, particles);
		}
	}
}
OBS_TYPE nxnGrid::UpdateObservation(nxnGridDetailedState & state, double p) const
{
	nxnGridDetailedState obsState(CountMovingObjects());
	obsState[0] = state[0];
	// calculate observed state
	DecreasePObsRec(state, obsState, 1, 1.0, p);

	for (int i = 0; i < m_enemyVec.size(); ++i)
		state.IsEnemyObserved(i, state[i + 1] == obsState[i + 1]);

	// return the observed state
	return obsState.GetStateId();
}

void nxnGrid::SetNextPosition(nxnGridDetailedState & state, doubleVec & randomNum) const
{
	// run on all enemies
	for (int i = 0; i < m_enemyVec.size(); ++i)
	{
		CalcMovement(state, &m_enemyVec[i], randomNum[i], i + 1);
	}
	// run on all non-involved non-involved death is the end of game so don't need to check for death
	for (int i = 0; i < m_nonInvolvedVec.size(); ++i)
	{
		CalcMovement(state, &m_nonInvolvedVec[i], randomNum[i + m_enemyVec.size()], i + 1 + m_enemyVec.size());
	}
}

void nxnGrid::CalcMovement(nxnGridDetailedState & state, const Movable_Obj *object, double rand, int objIdx) const
{
	// if the p that was pulled is in the pStay range do nothing
	std::map<int, double> possibleLocations;
	intVec nonValidLocations;
	GetNonValidLocations(state, objIdx, nonValidLocations);
	object->GetMovement()->GetPossibleMoves(state[objIdx], m_gridSize, nonValidLocations, possibleLocations, state[0]);

	int loc = -1;
	for (auto v : possibleLocations)
	{
		loc = v.first;
		rand -= v.second;
		if (rand <= 0.0)
			break;
	}

	state[objIdx] = loc;
}

void nxnGrid::ScaleState(const nxnGridDetailedState & beliefState, nxnGridDetailedState & scaledState) const
{
	ScaleState(beliefState, scaledState, s_lutGridSize, m_gridSize);
}

void nxnGrid::ScaleState(const nxnGridDetailedState & beliefState, nxnGridDetailedState & scaledState, int newGridSize, int oldGridSize) const
{
	// TODO : change ScaleState after allowing repetitions and using detailedstate

	//double scale = static_cast<double>(oldGridSize) / newGridSize;

	//for (size_t i = 0; i < beliefState.size(); ++i)
	//{
	//	Coordinate location(beliefState[i] % oldGridSize, beliefState[i] / oldGridSize);
	//	location /= scale;
	//	scaledState[i] = location.X() + location.Y() * newGridSize;
	//}

	//// if target is in self location shift map to the left or upper so self won't be in target location 
	// TODO : hard coded for target location
	//if (scaledState[0] == newGridSize * newGridSize - 1)
	//	ShiftSelfFromTarget(beliefState, scaledState, oldGridSize, newGridSize);

	//
	//int numEnemies = NumEnemiesInCalc();
	//// run on enemies in calculation if the enemy is in the same spot move enemy to the nearest available location
	//for (int i = 0; i < numEnemies; ++i)
	//{
	//	if (!NoRepetitions(scaledState, i + 1, newGridSize))
	//		MoveObjectLocation(beliefState, scaledState, i + 1, oldGridSize, newGridSize);
	//}

	//// run on non-involved in calculation if the non-involved is in non-valid location move him to the nearest location
	//for (int i = 0; i < NumNonInvInCalc(); ++i)
	//{
	//	if (!NoRepetitions(scaledState, numEnemies + 1 + i, newGridSize))
	//		MoveObjectLocation(beliefState, scaledState, m_enemyVec.size() + 1, oldGridSize, newGridSize);
	//}
	//

	//MoveNonProtectedShelters(beliefState, scaledState, oldGridSize, newGridSize);
}

void nxnGrid::Combine2EnemiesRewards(const intVec & beliefState, const doubleVec & rewards1E, const doubleVec & rewards2E, doubleVec & rewards) const
{
	static int bitEnemy1 = 1;
	static int bitEnemy2 = 2;
	int liveEnemies = 0;
	int numLivingEnemies = 0;
	rewards.resize(NumActions());
	// assumption : only 2 enemies
	int numEnemyActions = NumActions() - rewards1E.size();
	// start with actions related to enemy to understand which enemy is dead and calc mean in simple actions (assumption enemy related actions are at the end of actions)
	for (int a = NumActions() - 1; a >= 0; --a)
	{
		if (EnemyRelatedAction(a))
		{
			int enemyIdx = EnemyRelatedActionIdx(a);
			if (Attack::IsDead(beliefState[enemyIdx], m_gridSize))
				rewards[a] = REWARD_MIN;
			else if (enemyIdx == 1)
			{
				rewards[a] = rewards1E[a];
				liveEnemies |= bitEnemy1;
				++numLivingEnemies;
			}
			else
			{
				rewards[a] = rewards2E[a - numEnemyActions];
				liveEnemies |= bitEnemy2;
				++numLivingEnemies;
			}
		}
		else
		{
			// when only 1 enemy is dead insert to rewards calculation only reward of the live enemy
			if (numLivingEnemies > 0)
				rewards[a] = ((rewards1E[a] * (liveEnemies & bitEnemy1) + rewards2E[a] * (liveEnemies & bitEnemy2))) / numLivingEnemies;
			else
				rewards[a] = (rewards1E[a] + rewards2E[a]) / 2;

		}
	}
}

void nxnGrid::DecreasePObsRec(const nxnGridDetailedState & originalState, nxnGridDetailedState & currState, int currIdx, double pToDecrease, double &pLeft) const
{
	if (currIdx == CountMovingObjects())
	{
		// decrease observed state probability from the left probability
		pLeft -= pToDecrease;
	}
	else
	{
		int selfLoc = originalState[0];
		int objLoc = originalState[currIdx];
		Observation::observableLocations obsLocations;
		m_self.GetObservation()->InitObsAvailableLocations(selfLoc, objLoc, m_gridSize, obsLocations);
		for (auto obs : obsLocations)
		{
			currState[currIdx] = obs.first;
			DecreasePObsRec(originalState, currState, currIdx + 1, pToDecrease * obs.second, pLeft);
			if (pLeft < 0)
				return;
		}
	}
}

int nxnGrid::CountAllObjects() const
{
	return CountMovingObjects() + m_shelters.size();
}

unsigned int nxnGrid::CountMovingObjects() const
{
	return 1 + m_enemyVec.size() + m_nonInvolvedVec.size();
}

void nxnGrid::CreateRandomVec(doubleVec & randomVec, int size)
{
	// insert to a vector numbers between 0-1
	for (int i = 0; i < size; ++i)
	{
		randomVec.emplace_back(RandomNum());
	}
}

bool nxnGrid::RcvStateIMP(intVec & buffer, State * s, double & reward, OBS_TYPE & obs) const
{
	nxnGridDetailedState state(CountMovingObjects());
	nxnGridDetailedState observation(CountMovingObjects());
	//nxnGridDetailedState observation(lastObs);
	InitStateIMP(buffer, state, observation);

	if (state[0] == m_targetIdx)
		return true;
	
	s->state_id = state.GetStateId();
	obs = observation.GetStateId();

	// calc rewards according to model
	reward = 0.0;

	if (Attack::IsDead(state[0],m_gridSize) || state.IsNonInvDead(m_gridSize))
		reward = REWARD_LOSS;
	else if (state[0] == m_targetIdx)
		reward = REWARD_WIN;

	return false;
}

void nxnGrid::InitState()
{
	OnlineSolverModel::InitState();

	if (s_isExternalSimulator)
		InitStateSimulator();
	else
		InitStateRandom();
}

/// init state according to s_objectsInitLocations
void nxnGrid::InitStateRandom()
{
	int obj = 0;
	int idx = rand() % s_objectsInitLocations[obj].size();
	int loc = s_objectsInitLocations[obj][idx];
	m_self.SetLocation(Coordinate(loc % m_gridSize, loc / m_gridSize));
	++obj;

	for (int i = 0; i < m_enemyVec.size(); ++i, ++obj)
	{
		idx = rand() % s_objectsInitLocations[obj].size();
		loc = s_objectsInitLocations[obj][idx];
		m_enemyVec[i].SetLocation(Coordinate(loc % m_gridSize, loc / m_gridSize));
	}
	
	for (int i = 0; i < m_nonInvolvedVec.size(); ++i, ++obj)
	{
		idx = rand() % s_objectsInitLocations[obj].size();
		loc = s_objectsInitLocations[obj][idx];
		m_nonInvolvedVec[i].SetLocation(Coordinate(loc % m_gridSize, loc / m_gridSize));
	}

	for (int i = 0; i < m_shelters.size(); ++i, ++obj)
	{
		idx = rand() % s_objectsInitLocations[obj].size();
		loc = s_objectsInitLocations[obj][idx];
		m_shelters[i].SetLocation(Coordinate(loc % m_gridSize, loc / m_gridSize));
		nxnGridDetailedState::s_shelters[i] = loc;
	}

}
/// init state from simulator
void nxnGrid::InitStateSimulator()
{
	nxnGridDetailedState state(CountMovingObjects());
	nxnGridDetailedState observation(CountMovingObjects());
	// TODO : need to use ecvstate to init state from siulator

	//InitStateIMP(state, observation);

	//int currObj = 0;
	//Coordinate self(state[currObj] % m_gridSize, state[currObj] / m_gridSize);
	//m_self.SetLocation(self);

	//for (int i = 0; i < m_enemyVec.size(); ++i)
	//{
	//	++currObj;
	//	Coordinate enemy;
	//	if (state[currObj] == m_gridSize * m_gridSize)
	//		enemy = Coordinate(m_gridSize, m_gridSize);
	//	else
	//		enemy = Coordinate(state[currObj] % m_gridSize, state[currObj] / m_gridSize);

	//	m_enemyVec[i].SetLocation(enemy);
	//}

	//for (int i = 0; i < m_nonInvolvedVec.size(); ++i)
	//{
	//	++currObj;
	//	Coordinate ninv;
	//	if (state[currObj] == m_gridSize * m_gridSize)
	//		ninv = Coordinate(m_gridSize, m_gridSize);
	//	else
	//		ninv = Coordinate(state[currObj] % m_gridSize, state[currObj] / m_gridSize);

	//	m_nonInvolvedVec[i].SetLocation(ninv);
	//}
}

void nxnGrid::InitStateIMP(const intVec & data, nxnGridDetailedState & state, nxnGridDetailedState & observation) const
{
	// TODO : use data to retrieve state and observation from simulator
}

void nxnGrid::InitBeliefState(nxnGridDetailedState & beliefState, const History & h) const
{
	OBS_TYPE obs = h.LastObservation();
	nxnGridDetailedState observedState(obs);
	
	beliefState[0] = observedState[0];
	
	int obj = 1;
	for (; obj < CountMovingObjects() ; ++obj)
	{
		// run on all history and initialize state with last location observed on object
		int currObs = h.Size() - 1;
		int loc = observedState[obj];
		while (Observation::IsNonObserved(loc, m_gridSize) && currObs >= 0)
		{
			loc = nxnGridDetailedState::GetObjLocation(h.Observation(currObs), obj);
			--currObs;
		}
		// if the object is non observed through all history treat it as dead
		beliefState[obj] = Observation::IsNonObserved(loc, m_gridSize) ? Attack::DeadLoc(m_gridSize) : observedState[obj];
	}
}

intVec nxnGrid::GetSheltersVec() const
{
	intVec shelters;
	for (auto v : m_shelters)
		shelters.emplace_back(v.GetLocation().GetIdx(m_gridSize));

	return shelters;
}

void nxnGrid::GetNonValidLocations(const nxnGridDetailedState & state, int objIdx, intVec & nonValLoc) const
{
	for (int obj = 0; obj < CountMovingObjects(); ++obj)
	{
		if (obj != objIdx && state.IsProtected(obj))
			nonValLoc.emplace_back(state[obj]);
	}
}

void nxnGridBelief::Update(int action, OBS_TYPE obs)
{
	history_.Add(action, obs);

	std::vector<State*> updated;
	double total_weight = 0;
	double reward;
	OBS_TYPE o;
	// Update particles
	for (int i = 0; i <particles_.size(); i++) {
		State* particle = particles_[i];

		bool terminal = model_->Step(*particle, Random::RANDOM.NextDouble(), action, reward, o);

		// copy fully observed params from observation
		double prob = model_->ObsProb(obs, *particle, action);

		if (!terminal && prob)
		{ // Terminal state is not required to be explicitly represented and may not have any observation
			particle->weight *= prob;
			total_weight += particle->weight;
			updated.push_back(particle);
		}
		else 
		{
			model_->Free(particle);
		}
	}

	particles_ = updated;

	// Resample if the particle set is empty
	if (particles_.size() == 0)
	{
		Resample(num_particles_, initial_particles_, model_, history_);
		logw << "Particle set is empty!  Resample\n";

		//Update total weight so that effective number of particles are computed correctly 
		total_weight = 0;
		for (int i = 0; i < particles_.size(); i++) 
		{
			State* particle = particles_[i];
			total_weight = total_weight + particle->weight;
		}
	}


	double weight_square_sum = 0;
	for (int i = 0; i < particles_.size(); i++) {
		State* particle = particles_[i];
		particle->weight /= total_weight;
		weight_square_sum += particle->weight * particle->weight;
	}

	// Resample if the effective number of particles is "small"
	double num_effective_particles = 1.0 / weight_square_sum;
	if (num_effective_particles < num_particles_ / 2.0) {
		std::vector<State*> new_belief = Belief::Sample(num_particles_, particles_,
			model_);
		for (int i = 0; i < particles_.size(); i++)
			model_->Free(particles_[i]);

		particles_ = new_belief;
	}
}
std::vector<State*> nxnGridBelief::Resample(int num, const std::vector<State*>& belief, const DSPOMDP* model, History history)
{
	// randomization regarding choosing particles
	double unit = 1.0 / num;
	double mass = Random::RANDOM.NextDouble(0, unit);
	int pos = 0;
	double cur = belief[0]->weight;

	// for step
	double reward;
	OBS_TYPE obs;

	auto modelCast = static_cast<const nxnGrid *>(model);
	std::vector<std::vector<std::pair<int, double>>> objLocations(modelCast->CountMovingObjects());

	int nonObservedLoc = Observation::NonObservedLoc(modelCast->GetGridSize());

	// to get num particles need to take from each observed object nth root of num particles
	double numObjLocations = pow(num, 1.0 / (modelCast->CountMovingObjects() - 1));

	// insert self location (is known)
	objLocations[0].emplace_back(nxnGridDetailedState::GetObjLocation(history.LastObservation(), 0), 1.0);

	// run on each observable object and create individualy for each object belief state
	for (int obj = 1; obj < modelCast->CountMovingObjects(); ++obj)
	{
		int startSim = history.Size();
		
		int loc = nonObservedLoc;

		// running on all history and search for the last time observed the object
		for (; startSim > 0 & loc == nonObservedLoc; --startSim)
			loc = nxnGridDetailedState::GetObjLocation(history.Observation(startSim - 1), obj);


		// if we observe the object remember state as initial condition
		if (loc != nonObservedLoc)
			++startSim;  // start simulate step after successful observation

		int trial = 0;
		int count = 0;

		while (count < numObjLocations && trial < 200 * numObjLocations)
		{
			State* particle;
			// if we didn't observed the object start from initial belief
			if (loc == nonObservedLoc)
			{
				// Pick next particle
				while (mass > cur)
				{
					pos = (pos + 1) % belief.size();
					cur += belief[pos]->weight;
				}
				trial++;

				mass += unit;

				particle = model->Copy(belief[pos]);
			}
			else
			{
				particle = model->Allocate(history.Observation(startSim), 1.0);
			}

			// Step through history
			double wgt = 1.0;

			for (int i = startSim; i < history.Size(); i++)
			{
				model->Step(*particle, Random::RANDOM.NextDouble(), history.Action(i), reward, obs);
				double prob = modelCast->ObsProbOneObj(history.Observation(i), *particle, history.Action(i), obj);
				if (prob <= 0)
				{
					model->Free(particle);
					break;
				}
				wgt *= prob;
			}

			// Add to obj available locations if survived
			if (particle->IsAllocated())
			{
				count++;
				int objLoc = nxnGridDetailedState::GetObjLocation(particle->state_id, obj);
				objLocations[obj].emplace_back(objLoc, wgt);
			}
		}
	}

	std::vector<State*> sample;
	// create sample from object possible locations
	modelCast->CreateParticleVec(objLocations, sample);

	double total_weight = 0;
	for (int i = 0; i < sample.size(); i++) {
		//sample[i]->weight = exp(sample[i]->weight - max_wgt);
		total_weight += sample[i]->weight;
	}
	for (int i = 0; i < sample.size(); i++) {
		sample[i]->weight = sample[i]->weight / total_weight;
	}

	return sample;
}

} //end ns despot