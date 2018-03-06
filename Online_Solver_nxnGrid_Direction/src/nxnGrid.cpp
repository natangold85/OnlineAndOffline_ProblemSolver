#include <string>
#include <math.h>



#include "nxnGrid.h"
#include "Coordinate.h"

namespace despot 
{
// params for tree sending
static int s_START_NEW_TRIAL = 1;
static int s_START_NEW_STEP = 2;
static int s_END_RUNNING = 3;
static int s_SIGN_END_TREE = -1;

// init static members

int DetailedState::s_numNonInvolved = 0;
int DetailedState::s_numEnemies = 0;

int DetailedState::s_gridSize = 0;
int DetailedState::s_targetLoc = 0;

std::vector<int> DetailedState::s_shelters;

int nxnGrid::s_numBasicActions = -1;
int nxnGrid::s_numEnemyRelatedActions = -1;

std::vector<locationVec> nxnGrid::s_objectsInitLocations;

UDP_Server nxnGrid::s_udpVBS;
UDP_Server nxnGrid::s_udpTree;

const double nxnGrid::REWARD_WIN = 50.0;
const double nxnGrid::REWARD_LOSS = -100.0;
const double nxnGrid::REWARD_KILL_ENEMY = 0.0;
const double nxnGrid::REWARD_KILL_NINV = REWARD_LOSS;
const double nxnGrid::REWARD_ILLEGAL_MOVE = -200.0;
const double nxnGrid::REWARD_STEP = -1.0;
const double nxnGrid::REWARD_FIRE = -1.0;

// for lut
nxnGrid::lut_t nxnGrid::s_LUT;
int nxnGrid::s_lutGridSize;
bool nxnGrid::s_isOnline = true;
bool nxnGrid::s_resampleFromLastObs = true;
bool nxnGrid::s_isVBSEvaluator = false;
bool nxnGrid::s_toSendTree = false;

enum nxnGrid::CALCULATION_TYPE nxnGrid::s_calculationType = nxnGrid::WITHOUT;
int nxnGrid::s_periodOfDecision = 1;

/* =============================================================================
* inline Functions
* =============================================================================*/

/// return absolute value of a number
inline int Abs(int num)
{
	return num * (num >= 0) - num * (num < 0);
}

/// return min(a,b)
inline int Min(int a, int b)
{
	return a * (a <= b) + b * (b < a);
}

/// return max(a,b)
inline int Max(int a, int b)
{
	return a * (a >= b) + b * (b > a);
}

/* =============================================================================
* nxnGridState Functions
* =============================================================================*/

nxnGridState::nxnGridState(STATE_TYPE state_id, double weight)
	: State(state_id, weight)
{
}

DetailedState::DetailedState(int sizeState, bool isEmpty)
: m_locations(sizeState)
, m_directions(sizeState)
{
}

DetailedState::DetailedState(const State & state)
: m_locations(InitLocationsFromId(state.state_id))
, m_directions(InitDirectionsFromId(state.state_id))
{
}

DetailedState::DetailedState(STATE_TYPE state)
: m_locations(InitLocationsFromId(state))
, m_directions(InitDirectionsFromId(state))
{
}

locationVec DetailedState::InitLocationsFromId(STATE_TYPE state_id)
{
	int numObjects = 1 + s_numEnemies + s_numNonInvolved;
	locationVec locations(numObjects);
	
	// retrieve each location according to s_NUM_BITS_LOCATION
	int bitsForSinglelocation = (1 << s_NUM_BITS_LOCATION) - 1;
	for (int obj = 0; obj < numObjects; ++obj)
	{
		locations[obj] = state_id & bitsForSinglelocation;
		state_id >>= s_NUM_BITS_LOCATION;
	}

	return locations;
}

directionVec DetailedState::InitDirectionsFromId(STATE_TYPE state_id)
{
	int numObjects = 1 + s_numEnemies + s_numNonInvolved;
	// skip to the locations bits
	state_id >>= s_NUM_BITS_LOCATION * numObjects;
	directionVec directions(numObjects);

	// retrieve each location according to s_NUM_BITS_DIRECTION
	int bitsForSingleDirection = (1 << s_NUM_BITS_DIRECTION) - 1;
	for (int obj = 0; obj < numObjects; ++obj)
	{
		directions[obj] = (DIRECTIONS)(state_id & bitsForSingleDirection);
		state_id >>= s_NUM_BITS_DIRECTION;
	}

	return directions;
}

void DetailedState::GetSelfStatus(STATE_TYPE state_id, int & selfLocation, DIRECTIONS & selfDirection)
{
	// self location is in 0 position
	int bitsForSinglelocation = (1 << s_NUM_BITS_LOCATION) - 1;
	
	selfLocation = state_id & bitsForSinglelocation;

	int bitsForSingleDirection = (1 << s_NUM_BITS_DIRECTION) - 1;
	int numObjects = 1 + s_numEnemies + s_numNonInvolved;
	// skip to the locations bits
	state_id >>= s_NUM_BITS_LOCATION * numObjects;

	selfDirection = (DIRECTIONS)(state_id & bitsForSingleDirection);
}
STATE_TYPE DetailedState::GetStateId() const
{
	STATE_TYPE locations = GetLocationsId(m_locations);
	STATE_TYPE directions = GetDirectionsId(m_directions);

	STATE_TYPE stateId = directions << (s_NUM_BITS_LOCATION * m_locations.size());
	stateId |= locations;

	return stateId;
}

STATE_TYPE DetailedState::GetLocationsId(const locationVec & locations)
{
	STATE_TYPE locationId = 0;
	for (int obj = locations.size() - 1; obj >= 0; --obj)
	{
		locationId <<= s_NUM_BITS_LOCATION;
		locationId |= locations[obj];
	}

	return locationId;
}

STATE_TYPE DetailedState::GetDirectionsId(const directionVec & directions)
{
	STATE_TYPE directionId = 0;
	for (int obj = directions.size() - 1; obj >= 0; --obj)
	{
		directionId <<= s_NUM_BITS_DIRECTION;
		directionId |= directions[obj];
	}

	return directionId;
}

void DetailedState::InitStatic()
{
	s_numNonInvolved = 0;
	s_numEnemies = 0;
	s_gridSize = 0;
	s_shelters.resize(0);
}

STATE_TYPE DetailedState::MaxState()
{
	int numObjects = 1 + s_numEnemies + s_numNonInvolved;
	int numBits = (s_NUM_BITS_LOCATION + s_NUM_BITS_DIRECTION) * numObjects;
	
	STATE_TYPE ret = 1;

	return ret << numBits;
}

char DetailedState::ObjIdentity(const locationVec & locations, int location)
{
	if (locations[0] == location)
		return 'M';
	
	int o = 1;
		
	for (; o < s_numEnemies + 1; ++o)
	{
		if (locations[o] == location)
			return o + '0';
	}
	
	int toEndOfNonInv = o + s_numNonInvolved;
	for (o; o < toEndOfNonInv; ++o)
	{
		if (locations[o] == location)
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

void DetailedState::EraseNonInv()
{
	int startNonInv = 1 + s_numEnemies;
	for (int n = 0; n < s_numNonInvolved; ++n)
		EraseObject(startNonInv);
}

void DetailedState::EraseObject(int objectIdx)
{
	m_locations.erase(m_locations.begin() + objectIdx);
	m_directions.erase(m_directions.begin() + objectIdx);
}

std::string DetailedState::text() const
{
	std::string ret = "(";
	for (int i = 0; i < m_locations.size(); ++i)
		ret += std::to_string(m_locations[i]) + "_" + Move_Properties::s_directionNames[m_directions[i]] + ", ";
	
	ret += ")\n";

	PrintGrid(m_locations, ret);
	return ret;
}

void DetailedState::PrintGrid(const locationVec & locations, std::string & buffer)
{
	for (int y = 0; y < s_gridSize; ++y)
	{
		for (int x = 0; x < s_gridSize; ++x)
		{
			int loc = x + y * s_gridSize;
			buffer += ObjIdentity(locations, loc);
		}
		buffer += "\n";
	}
}

bool DetailedState::NoEnemies(int gridSize) const
{
	for (int e = 0; e < s_numEnemies; ++e)
		if (m_locations[e + 1] != gridSize * gridSize)
			return false;
	
	return true;
}

bool DetailedState::IsDead(int objIdx, int gridSize) const
{
	return m_locations[objIdx] == gridSize * gridSize;
}

DetailedObservation::DetailedObservation(OBS_TYPE obs)
: m_locations(InitObservation(obs))
{
}

DetailedObservation::DetailedObservation(const DetailedState & state, const Self_Obj & self, int gridSize, doubleVec & prob)
: m_locations(InitObservation(state.m_locations, self, gridSize, prob))
{
}

locationVec DetailedObservation::InitObservation(const locationVec & realLocation, const Self_Obj & self, int gridSize, doubleVec & prob)
{
	locationVec observation(realLocation.size());
	observation[0] = realLocation[0];

	for (int o = 1; o < realLocation.size(); ++o)
		observation[o] = self.GetObservation()->GetObservationObject(realLocation[0], realLocation[o], gridSize, prob[o - 1]);
	return observation;
}

locationVec DetailedObservation::InitObservation(OBS_TYPE obs)
{
	return DetailedState::InitLocationsFromId((STATE_TYPE)obs);
}

OBS_TYPE DetailedObservation::GetObsType()
{
	return DetailedState::GetLocationsId(m_locations);
}

std::string DetailedObservation::text() const
{
	std::string ret = "(";
	for (int i = 0; i < m_locations.size(); ++i)
		ret += std::to_string(m_locations[i]) + ", ";

	ret += ")\n";

	DetailedState::PrintGrid(m_locations, ret);
	return ret;
}

/* =============================================================================
* nxnGrid Functions
* =============================================================================*/

nxnGrid::nxnGrid(int gridSize, int target, Self_Obj & self, std::vector<locationVec> & objectsInitLoc)
	: m_gridSize(gridSize)
	, m_targetIdx(target)
	, m_self(self)
	, m_enemyVec()
	, m_shelters()
	, m_nonInvolvedVec()
{
	// init size  of state for nxnGridstate
	DetailedState::s_gridSize = gridSize;
	DetailedState::s_targetLoc = target;

	s_objectsInitLocations = objectsInitLoc;
}

bool nxnGrid::InitUDP(int vbsPort, int sendTreePort)
{
	bool stat = true;
	if (vbsPort > 0)
	{
		stat &= s_udpVBS.Init(vbsPort);
		s_isVBSEvaluator = true;
	}
	else
		s_isVBSEvaluator = false;

	if (sendTreePort > 0)
	{
		stat &= s_udpTree.Init(sendTreePort);
		char c;
		s_udpTree.Read(&c);
		s_toSendTree = c;
	}
	else
		s_toSendTree = false;

	if (stat)
		std::cout << "required udp connections initialized\n";
	else
		std::cout << "udp connections not initialized correctly\n";

	return stat;
}

void nxnGrid::InitStaticMembers(bool isOnline, int periodOfDecision, bool resampleFromLastObs)
{
	s_isOnline = isOnline;
	s_periodOfDecision = periodOfDecision;
	s_resampleFromLastObs = resampleFromLastObs;
}

void nxnGrid::InitLUT(lut_t & offlineLut, int offlineGridSize, CALCULATION_TYPE ctype)
{
	s_calculationType = ctype;
	s_lutGridSize = offlineGridSize;
	s_LUT = offlineLut;
}

std::vector<State*> nxnGrid::ResampleLastObs(int num, const std::vector<State*>& belief, const DSPOMDP* model, History history)
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
	std::vector<std::map<locAndDirPair, double>> objLocations(modelCast->CountMovingObjects());
	std::vector<int> stateVec;
	// to get num particles need to take from each observed object nth root of num particles
	double numObjStates = pow(num, 1.0 / (modelCast->CountMovingObjects() - 1));
	// insert self location (is known)
	objLocations[0].emplace_back(modelCast->GetObsLoc(history.LastObservation(), 0), 1.0);
	// run on each observable object and create individualy for each object belief state
	for (int obj = 1; obj < modelCast->CountMovingObjects(); ++obj)
	{
		int startSim = history.Size();
		int loc = -1;

		// running on all history and search for the last time observed the object
		for (; startSim > 0 & loc < 0; --startSim)
			loc = modelCast->GetObsLoc(history.Observation(startSim - 1), obj);

		State* observedLoc = nullptr;

		// if we observe the object remember state as initial condition
		if (loc >= 0)
		{
			observedLoc = model->Allocate(history.Observation(startSim), 1.0);
			++startSim;  // start simulate step after the observation
		}

		int trial = 0;
		int count = 0;

		while (count < numObjLocations && trial < 200 * numObjLocations)
		{
			State* particle;
			OBS_TYPE prevObs;
			// if we didn't observed the object start from initial belief
			if (observedLoc == nullptr)
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
				prevObs = 0;
			}
			else
			{
				particle = model->Copy(observedLoc);
				prevObs = history.Observation(startSim - 1);
			}

			// Step through history
			double wgt = 1.0;

			for (int i = startSim; i < history.Size(); i++)
			{
				model->Step(*particle, Random::RANDOM.NextDouble(), history.Action(i), prevObs, reward, obs);
				double prob = modelCast->ObsProbOneObj(history.Observation(i), *particle, history.Action(i), obj);
				if (prob <= 0)
				{
					model->Free(particle);
					break;
				}
				wgt *= prob;
				prevObs = history.Observation(i);
			}

			// Add to obj available locations if survived
			if (particle->IsAllocated())
			{
				nxnGridState::IdxToState(particle, stateVec);
				count++;
				objLocations[obj].emplace_back(stateVec[obj], wgt);
			}
		}

		if (observedLoc != nullptr)
			model->Free(observedLoc);
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

void nxnGrid::UpdateRealAction(int & action)
{
	static int counter = 0;
	static int prevAction = 0;

	if (counter % s_periodOfDecision == 0)
		prevAction = action;
	else
		action = prevAction;

	++counter;
}

void nxnGrid::SendEndRun()
{
	unsigned int buf[1];

	buf[0] = s_END_RUNNING;
	s_udpTree.Write(reinterpret_cast<char *>(buf), 1 * sizeof (int));
}


void nxnGrid::SendModelDetails(int numActions, int numObj)
{
	// insert to unsigned int for sending tree compatibility
	unsigned int buf[4];

	buf[0] = s_START_NEW_TRIAL;
	buf[1] = numActions;
	buf[2] = numObj;
	buf[3] = s_SIGN_END_TREE;

	s_udpTree.Write(reinterpret_cast<char *>(buf), 4 * sizeof (int));
}

void nxnGrid::SendTree(State * state, VNode *root)
{
	// create tree
	std::vector<unsigned int> buffer;

	buffer.emplace_back(s_START_NEW_STEP);

	// TODO : send state for tree visualizer
	//locationVec stateVec;
	//nxnGridState::IdxToStateWithShelters(state, stateVec);
	//for (auto v : stateVec)
	//	buffer.emplace_back(v);

	SendTreeRec(root, 0, 1, buffer);

	buffer.emplace_back(s_SIGN_END_TREE);

	// send to client
	int sent = 0;
	// to make sure bufsize is divided by sizeof int
	int maxWrite = UDP_Server::s_BUF_LEN - UDP_Server::s_BUF_LEN % sizeof(int);
	while ( sent < (buffer.size()) )
	{
		char *ptr = reinterpret_cast<char *>(&buffer[sent]);
		int leftToWrite = buffer.size() - sent;
		int size = Min(leftToWrite * sizeof(int), maxWrite);
		s_udpTree.Write(ptr, size);
		sent += size / sizeof(int);
	}
}

bool nxnGrid::NoEnemies(State * s)
{
	DetailedState state(*s);
	return state.NoEnemies(m_gridSize);
}
unsigned int nxnGrid::SendTreeRec(VNode *node, unsigned int parentId, unsigned int id, std::vector<unsigned int> & buffer)
{
	if (node == nullptr || node->count() == 0)
		return id - 1;

	float val = node->value();
	unsigned int count = node->count();

	buffer.emplace_back(parentId);
	buffer.emplace_back(id);
	buffer.emplace_back(*reinterpret_cast<int *>(&val));
	buffer.emplace_back(count);

	int numch = node->children().size() + 1;
	int nextId = id + node->children().size();
	int actionId = id + 1;
	for (auto action : node->children())
	{
		nextId = SendTreeRec(action, id, actionId, nextId, buffer);
		++actionId;
	}

	return nextId;
}

unsigned int nxnGrid::SendTreeRec(QNode *node, unsigned int parentId, unsigned int id, unsigned int nextAvailableId, std::vector<unsigned int> & buffer)
{
	if (node == nullptr) 
		return id; // for action node need to remember id for completion of non initialized nodes

	float val = node->value();
	unsigned int count = node->count();

	buffer.emplace_back(parentId);
	buffer.emplace_back(id);
	buffer.emplace_back(*reinterpret_cast<int *>(&val));
	buffer.emplace_back(count);

	int nextId = nextAvailableId;
	for (auto obs : node->children())
	{
		++nextId;
		nextId = SendTreeRec(obs.second, id, nextId, buffer);
	}

	return nextId;
}

void nxnGrid::AddObj(Attack_Obj&& obj)
{
	m_enemyVec.emplace_back(std::forward<Attack_Obj>(obj));
	DetailedState::s_numEnemies = m_enemyVec.size();

	AddActionsToEnemy();
}

void nxnGrid::AddObj(Movable_Obj&& obj)
{
	m_nonInvolvedVec.emplace_back(std::forward<Movable_Obj>(obj));
	DetailedState::s_numNonInvolved = m_nonInvolvedVec.size();
}

void nxnGrid::AddObj(ObjInGrid&& obj)
{
	m_shelters.emplace_back(std::forward<ObjInGrid>(obj));
	DetailedState::s_shelters = GetSheltersVec();

	AddActionsToShelter();
}



int nxnGrid::GetObsLoc(OBS_TYPE observation, int objIdx) const
{
	DetailedObservation obs(observation);
	bool isRealLoc = obs.m_locations[objIdx] <= m_gridSize * m_gridSize;
	//
	//return obsState[objIdx] * (isRealLoc) -1 * (!isRealLoc);
	return obs.m_locations[objIdx] * isRealLoc - 1 * !isRealLoc;
}

double nxnGrid::ObsProb(OBS_TYPE obs, const State & s, int action) const
{
	DetailedState state(s);
	DetailedObservation observation(obs);

	// if observation is not including the location of the robot return 0
	if (state.m_locations[0] != observation.m_locations[0])
		return 0.0;

	double pObs = 1.0;
	// run on all non-self objects location
	for (int i = 1; i < CountMovingObjects(); ++i)
	{
		// create possible observation of obj location
		locationVec observableLocations;
		m_self.GetObservation()->InitObsAvailableLocations(state.m_locations[0], state.m_locations[1], state.m_locations, m_gridSize, observableLocations);
		
		// run on possible observable location
		bool isObserved = false;
		for (auto obsLoc : observableLocations)
		{
			if (obsLoc == observation.m_locations[i])
			{
				pObs *= m_self.GetObservation()->GetProbObservation(state.m_locations[0], state.m_locations[i], m_gridSize, obsLoc);
				isObserved = true;
			}
		}
		
		// if the object is not in observable locations return 0
		if (!isObserved)
			return 0.0;
	}
	
	return pObs;
}

double nxnGrid::ObsProbOneObj(OBS_TYPE obs, const State & s, int action, int objIdx) const
{
	DetailedState state(s);
	DetailedObservation observation(obs);

	if (objIdx == 0)
		return state.m_locations[0] == observation.m_locations[0];

	double pObs = 0.0;
	
	// create possible observation of obj location
	locationVec observableLocations;
	// send location of self of observed state for non observable location(only important location of object not self)
	m_self.GetObservation()->InitObsAvailableLocations(state.m_locations[0], state.m_locations[objIdx], state.m_locations, m_gridSize, observableLocations);

	// run on possible observable location
	for (auto obsLoc : observableLocations)
	{
		if (obsLoc == observation.m_locations[objIdx])
			return m_self.GetObservation()->GetProbObservation(state.m_locations[0], state.m_locations[objIdx], m_gridSize, obsLoc);
	}

	return 0.0;
}

void nxnGrid::CreateParticleVec(std::vector<std::vector<std::pair<int, double> > > & objLocations, std::vector<State*> & particles) const
{
	//DetailedState state(CountMovingObjects(), true);
	//CreateParticleVecRec(state, objLocations, particles, 1.0, 0);
}

void nxnGrid::ChoosePreferredActionIMP(const DetailedState & beliefState, doubleVec & expectedReward) const
{
	lut_t::iterator itr, itr2;

	switch (s_calculationType)
	{
	case ALL:
	{
		DetailedState scaledState(beliefState.size(), true);

		ScaleState(beliefState, scaledState, DetailedState::s_shelters);
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
		DetailedState woNInv(beliefState);
		woNInv.EraseNonInv();
		
		DetailedState scaledState(woNInv.size(), true);
		ScaleState(woNInv, scaledState, DetailedState::s_shelters);
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
		DetailedState woNInv(beliefState);
		woNInv.EraseNonInv();
		DetailedState scaledState(woNInv.size(), true);

		ScaleState(woNInv, scaledState, DetailedState::s_shelters);
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
		DetailedState firstEnemy(beliefState);
		firstEnemy.EraseNonInv();
		DetailedState secondEnemy(firstEnemy);

		firstEnemy.EraseObject(1);
		secondEnemy.EraseObject(2);
		DetailedState scaledState(firstEnemy.size(), true);
		
		ScaleState(firstEnemy, scaledState, DetailedState::s_shelters);
		itr = s_LUT.find(scaledState.GetStateId());

		// calculate reward with second enemy
		ScaleState(secondEnemy, scaledState, DetailedState::s_shelters);
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

		//// TODO : move members 1 slot right
		//if (itr != s_LUT.end())
		//	expectedReward = itr->second;
		//else
		//	expectedReward = doubleVec(NumActions(), REWARD_LOSS);
		//break;
	}
	default: // calc type = WITHOUT
		expectedReward = doubleVec(NumActions(), REWARD_LOSS);
		break;
	}
}

int nxnGrid::FindMaxReward(const doubleVec & rewards, double & expectedReward)
{
	int maxAction = -1;
	double maxReward = REWARD_LOSS - 1;

	for (int a = 0; a < rewards.size(); ++a)
	{
		if (rewards[a] > maxReward)
		{
			maxAction = a;
			maxReward = rewards[a];
		}
	}

	expectedReward = maxReward;
	return maxAction;
}

void nxnGrid::CreateParticleVecRec(DetailedState & state, std::vector<std::vector<std::pair<int, double> > > & objLocations, std::vector<State*> & particles, double weight, int currIdx) const
{
	//if (currIdx == state.size())
	//{
	//	auto beliefState = static_cast<nxnGridState*>(Allocate(state.GetStateId(), weight));
	//	particles.push_back(beliefState);
	//}
	//else
	//{
	//	for (auto loc : objLocations[currIdx])
	//	{
	//		state.m_locations[currIdx] = loc.first;
	//		CreateParticleVecRec(state, objLocations, particles, weight * loc.second, currIdx + 1);
	//	}
	//}
}

State * nxnGrid::CreateStartState(std::string type) const
{
	DetailedState state(CountMovingObjects(), true);

	state.m_directions[0] = NO_DIRECTION;
	state.m_locations[0] = m_self.GetLocation().GetIdx(m_gridSize);
	
	int obj = 1;
	for (int i = 0; i < m_enemyVec.size(); ++i, ++obj)
	{
		state.m_directions[obj] = NO_DIRECTION;
		state.m_locations[obj] = m_enemyVec[i].GetLocation().GetIdx(m_gridSize);
	}
	
	for (int i = 0; i < m_nonInvolvedVec.size(); ++i, ++obj)
	{
		state.m_directions[obj] = NO_DIRECTION;
		state.m_locations[obj] = m_nonInvolvedVec[i].GetLocation().GetIdx(m_gridSize);
	}

	return new nxnGridState(state.GetStateId());
}

Belief * nxnGrid::InitialBelief(const State * start, std::string type) const
{
	std::vector<State*> particles;
	DetailedState state(CountMovingObjects(), true);

	// create belief states given self location
	state.m_locations[0] = m_self.GetLocation().GetIdx(m_gridSize);
	int numStates = 1;
	// assumption : init states cannot be in same locations for different object
	for (int i = 1; i < CountMovingObjects(); ++i)
		numStates *= s_objectsInitLocations[i].size();

	double stateProb = 1.0 / numStates;
	
	InitialBeliefStateRec(state, 1, stateProb, particles);

	return new ParticleBelief(particles, this);
}

State * nxnGrid::Allocate(STATE_TYPE state_id, double weight) const
{
	nxnGridState* particle = memory_pool_.Allocate();
	particle->state_id = state_id;
	particle->weight = weight;
	return particle;
}

State * nxnGrid::Copy(const State * particle) const
{
	nxnGridState* new_particle = memory_pool_.Allocate();
	*new_particle = *static_cast<const nxnGridState*>(particle);
	new_particle->SetAllocated();
	return new_particle;
}

void nxnGrid::Free(State * particle) const
{
	memory_pool_.Free(static_cast<nxnGridState*>(particle));
}

int nxnGrid::NumActiveParticles() const
{
	return memory_pool_.num_allocated();
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
	DetailedState state(s.state_id);
	out << state.text() << "\n";
}

void nxnGrid::PrintBelief(const Belief & belief, std::ostream & out) const
{
}

void nxnGrid::PrintObs(const State & state, OBS_TYPE obs, std::ostream & out) const
{
	DetailedObservation obsState(obs);
	out << obsState.text() << "\n";
}

bool nxnGrid::InRange(int locationSelf, int locationObj, double range, int gridSize)
{
	Coordinate self(locationSelf % gridSize, locationSelf / gridSize);
	Coordinate enemy(locationObj % gridSize, locationObj / gridSize);

	// return true when distance is <= from range
	return self.RealDistance(enemy) <= range;
}

bool nxnGrid::CalcIfDead(int enemyIdx, DetailedState & state, double & randomNum) const
{
	locationVec shelters(GetSheltersVec());

	m_enemyVec[enemyIdx].AttackOnline(state.m_locations, enemyIdx + 1, 0, shelters, m_gridSize, randomNum);

	return state.IsDead(0, m_gridSize);
}

void nxnGrid::MoveNonProtectedShelters(const locationVec & beliefState, locationVec & scaledState, int oldGridSize, int newGridSize) const
{
	int startShelter = 1 + NumEnemiesInCalc() + NumNonInvInCalc();
	for (int s = 0; s < m_shelters.size(); ++s)
	{
		int shelterIdx = s + startShelter;
		
		for (int o = 0; o < startShelter; ++o)
		{
			if ((scaledState[o] == scaledState[shelterIdx]) & (beliefState[o] != beliefState[shelterIdx] & scaledState[o] != newGridSize * newGridSize))
			{
				MoveObjectLocation(beliefState, scaledState, o, oldGridSize, newGridSize);
			}
		}
	}
}

void nxnGrid::MoveObjectLocation(const locationVec & beliefState, locationVec & scaledState, int objIdx, int oldGridSize, int newGridSize) const
{
	// TODO: adjust to model change

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
	
	// TODO: see whats happenning to shelter after the moving of the object
}

void nxnGrid::ShiftSelfFromTarget(const locationVec & beliefState, locationVec & scaledState, int oldGridSize, int newGridSize) const
{
	Coordinate realSelf(beliefState[0] % oldGridSize, beliefState[0] / oldGridSize);
	
	// if x > y shift on y axis else shift object on x axis
	if (realSelf.X() > realSelf.Y())
	{
		for (int i = 0; i < scaledState.size(); ++i)
		{
			if (scaledState[i] == newGridSize * newGridSize)
				continue;

			int yLoc = scaledState[i] / newGridSize - 1;

			// if the object is out of grid drop him from calculation
			if (yLoc < 0)
			{
				scaledState[i] = newGridSize * newGridSize;
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
			if (scaledState[i] == newGridSize * newGridSize)
				continue;

			int xLoc = scaledState[i] % newGridSize - 1;

			// if the object is out of grid drop him from calculation
			if (xLoc < 0)
			{
				scaledState[i] = newGridSize * newGridSize;
			}
			else
			{
				--scaledState[i];
			}
		}
	}
}

//void nxnGrid::DropUnProtectedShelter(locationVec & state, int gridSize) const
//{
//	bool isProtected = false;
//	int shelterIdx = 1 + m_enemyVec.size() + m_nonInvolvedVec.size();
//	foWr (int i = 0; i < shelterIdx; ++i)
//	{
//		isProtected |= state[i] == state[shelterIdx];
//	}
//
//	if (!isProtected)
//		state[shelterIdx] = gridSize * gridSize;
//}

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


inline bool nxnGrid::InBoundary(int location, int xChange, int yChange) const
{
	int x = location % m_gridSize + xChange;
	int y = location / m_gridSize + yChange;
	// return true if the location is in boundary
	return x >= 0 & x < m_gridSize & y >= 0 & y < m_gridSize;
}


enum nxnGrid::OBJECT nxnGrid::WhoAmI(int objIdx) const
{
	return objIdx == 0 ? SELF :
		objIdx < m_enemyVec.size() + 1 ? ENEMY :
		objIdx < m_nonInvolvedVec.size() + m_enemyVec.size() + 1 ? NON_INV :
		objIdx < m_shelters.size() + m_nonInvolvedVec.size() + m_enemyVec.size() + 1 ? SHELTER : TARGET;
}

void nxnGrid::InitialBeliefStateRec(DetailedState & state, int currObj, double stateProb, std::vector<State*> & particles) const
{
	if (currObj == CountMovingObjects())
	{
		auto beliefState = static_cast<nxnGridState*>(Allocate(state.GetStateId(), stateProb));
		particles.push_back(beliefState);
	}
	else
	{
		for (auto obj : s_objectsInitLocations[currObj])
		{
			state.m_directions[currObj] = NO_DIRECTION;
			state.m_locations[currObj] = obj;
			InitialBeliefStateRec(state, currObj + 1, stateProb, particles);
		}
	}
}
OBS_TYPE nxnGrid::FindObservation(DetailedState & state, doubleVec obsRandNums) const
{
	// calculate observed state
	DetailedObservation observation(state, m_self, m_gridSize, obsRandNums);

	// return the observed state
	return observation.GetObsType();
}

void nxnGrid::SetNextPosition(DetailedState & state, doubleVec & randomNum) const
{
	// run on all enemies
	for (int i = 0; i < m_enemyVec.size(); ++i)
	{
		// if the object is not dead calc movement
		if (!state.IsDead(i + 1, m_gridSize))
			CalcMovement(state, &m_enemyVec[i], i + 1, randomNum[i], state.m_locations[0]);
	}
	// run on all non-involved non-involved death is the end of game so don't need to check for death
	for (int i = 0; i < m_nonInvolvedVec.size(); ++i)
	{
		// if the object is not dead calc movement
		int nonInvIdx = i + 1 + m_enemyVec.size();
		if (!state.IsDead(nonInvIdx, m_gridSize))
			CalcMovement(state, &m_nonInvolvedVec[i], nonInvIdx, randomNum[nonInvIdx - 1]);
	}
}

void nxnGrid::CalcMovement(DetailedState & state, const Movable_Obj *object, int objIdx, double rand, int targetLocation) const
{
	std::map<DIRECTIONS, double> possibleDirections;
	object->GetMovement()->GetPossibleDirections(state.m_locations[objIdx], state.m_directions[objIdx], targetLocation, m_gridSize, possibleDirections);

	for (auto dir : possibleDirections)
	{
		std::map<int, double> possibleLocations;
		m_self.GetMovement()->GetPossibleLocations(state.m_locations[objIdx], dir.first, m_gridSize, possibleLocations);
		for (auto loc : possibleLocations)
		{
			rand -= dir.second * loc.second;
			if (rand <= 0.0)
			{
				if (objIdx > 0 && loc.first != state.m_locations[objIdx])
					int a = 9;
				state.m_directions[objIdx] = dir.first;;
				state.m_locations[objIdx] = loc.first;;
				return;
			}
		}
	}
}

bool nxnGrid::ValidLocation(locationVec & state, int location)
{
	for (auto v : state)
	{
		if (location == v)
		{
			return false;
		}
	}

	return true;
}

bool nxnGrid::ValidLegalLocation(locationVec & state, Coordinate location, int end, int gridSize)
{
	if ((location.X() >= gridSize) | (location.X() < 0) | (location.Y() >= gridSize) | (location.Y() < 0))
		return false;

	int locationIdx = location.X() + location.Y() * gridSize;
	for (size_t i = 0; i < end; ++i)
	{
		if (locationIdx == state[i])
		{
			return false;
		}
	}

	return true;
}

void nxnGrid::ScaleState(const DetailedState & beliefState, DetailedState & scaledState, locationVec & sheltersLocation) const
{
	ScaleState(beliefState, scaledState, sheltersLocation, s_lutGridSize, m_gridSize);
}

void nxnGrid::ScaleState(const DetailedState & beliefState, DetailedState & scaledState, locationVec & sheltersLocation, int newGridSize, int oldGridSize) const
{
	// TODO adjust rescailing to movement

	//double scale = static_cast<double>(oldGridSize) / newGridSize;

	//for (size_t i = 0; i < beliefState.size(); ++i)
	//{
	//	Coordinate location(beliefState[i] % oldGridSize, beliefState[i] / oldGridSize);
	//	location /= scale;
	//	scaledState[i] = location.X() + location.Y() * newGridSize;
	//}

	//// if target is in self location shift map to the left or upper so self won't be in target location 
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

void nxnGrid::Combine2EnemiesRewards(const locationVec & beliefStateLocations, const doubleVec & rewards1E, const doubleVec & rewards2E, doubleVec & rewards) const
{
	static int bitE1 = 1;
	static int bitE2 = 2;
	int deadEnemies = 0;

	rewards.resize(NumActions());
	// insert first enemy related actions rewards (if dead insert reward loss)
	if (beliefStateLocations[1] != m_gridSize * m_gridSize)
		for (int a = s_numBasicActions; a < s_numBasicActions + s_numEnemyRelatedActions; ++a)
			rewards[a] = rewards1E[a];
	else
	{
		deadEnemies |= bitE1;
		for (int a = s_numBasicActions; a < s_numBasicActions + s_numEnemyRelatedActions; ++a)
			rewards[a] = REWARD_LOSS;
	}
	// insert second enemy related actions rewards (if dead insert reward loss)
	if (beliefStateLocations[2] != m_gridSize * m_gridSize)
		for (int a = s_numBasicActions; a < s_numBasicActions + s_numEnemyRelatedActions; ++a)
			rewards[a + s_numEnemyRelatedActions] = rewards2E[a];
	else
	{
		deadEnemies |= bitE2;
		for (int a = s_numBasicActions; a < s_numBasicActions + s_numEnemyRelatedActions; ++a)
			rewards[a + s_numEnemyRelatedActions] = REWARD_LOSS;
	}

	// for non enemy related action do average of rewards
	for (int a = 0; a < s_numBasicActions; ++a)
	{
		// when only 1 enemy is dead insert to rewards calculation only reward of the live enemy
		double rE1 = rewards1E[a] * (deadEnemies != bitE1) + rewards2E[a] * (deadEnemies == bitE1);
		double rE2 = rewards2E[a] * (deadEnemies != bitE2) + rewards1E[a] * (deadEnemies == bitE2);
		rewards[a] = (rE1 + rE2) / 2;
	}
}


bool nxnGrid::NoRepetitions(locationVec & state, int currIdx, int gridSize)
{
	for (int i = 0; i < currIdx; ++i)
	{
		if (state[i] == state[currIdx] && state[i] != gridSize * gridSize)
		{
			return false;
		}
	}
	return true;
}


void nxnGrid::DecreasePObsRec(locationVec & currState, const locationVec & originalState, int currIdx, double pToDecrease, double &pLeft) const
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
		locationVec observableLocations;
		m_self.GetObservation()->InitObsAvailableLocations(selfLoc, objLoc, originalState, m_gridSize, observableLocations);
		
		for (int obs = 0; obs < observableLocations.size() & pLeft >= 0.0; ++obs)
		{
			currState[currIdx] = observableLocations[obs];
			double pObs = m_self.GetObservation()->GetProbObservation(selfLoc, objLoc, m_gridSize, observableLocations[obs]);
			DecreasePObsRec(currState, originalState, currIdx + 1, pToDecrease * pObs, pLeft);
		}
	}
}

int nxnGrid::CountAllObjects() const
{
	return CountMovingObjects() + m_shelters.size();
}

int nxnGrid::CountMovingObjects() const
{
	return 1 + m_enemyVec.size() + m_nonInvolvedVec.size();
}

void nxnGrid::CreateRandomVec(doubleVec & randomVec, int size)
{
	// insert to a vector numbers between 0-1
	for (int i = 0; i < size; ++i)
	{
		randomVec.emplace_back(Random::RANDOM.NextDouble());
	}
}

void nxnGrid::GetCloser(locationVec & state, int objIdx, int gridSize) const
{
	int xSelf = state[0] % gridSize;
	int ySelf = state[0] / gridSize;
	int xObj = state[objIdx] % gridSize;
	int yObj = state[objIdx] / gridSize;

	int objLocation = state[objIdx];
	int move = objLocation;

	int xDiff = xSelf - xObj;
	int yDiff = ySelf - yObj;

	int changeToInsertX = xDiff != 0 ? xDiff / Abs(xDiff) : 0;
	int changeToInsertY = yDiff != 0 ? (yDiff / Abs(yDiff)) * m_gridSize : 0;

	// insert to move the best valid option
	if (ValidLocation(state, move + changeToInsertX))
		move += changeToInsertX;
	if (ValidLocation(state, move + changeToInsertY))
		move += changeToInsertY;

	state[objIdx] = move;
}

int nxnGrid::FindObjMove(int currLocation, double random, int gridSize) const
{
	int x = currLocation % gridSize;
	int y = currLocation / gridSize;
	// change x and y according to random probability
	if (random > 0.5)
	{
		random = (random - 0.5) * 2;
		if (random > 0.5)
		{
			++x;
			if (random > 0.75)
				++y;
			else
				--y;
		}
		else
		{
			--x;
			if (random > 0.25)
				++y;
			else
				--y;
		}
	}
	else
	{
		random *= 2;
		if (random > 0.5)
		{
			if (random > 0.75)
				++y;
			else
				--y;
		}
		else
		{
			if (random > 0.25)
				++x;
			else
				--x;
		}
	}

	if ((x >= 0) & (x < gridSize) & (y >= 0) & (y < gridSize))
		return x + y * gridSize;

	return currLocation;
}

bool nxnGrid::InSquare(int location, int location2, int squareSize,int gridSize)
{
	int xReal = location % gridSize;
	int yReal = location / gridSize;

	int xObserved = location2 % gridSize;
	int yObserved = location2 / gridSize;

	int xDiff = Abs(xReal - xObserved);
	int yDiff = Abs(yReal - yObserved);

	return (xDiff <= squareSize) & (yDiff <= squareSize);
}

void nxnGrid::SendAction(int action)
{
	s_udpVBS.Write(reinterpret_cast<char *>(&action), sizeof(int));
}

bool nxnGrid::RcvState(State * s, int action, OBS_TYPE lastObs, double & reward, OBS_TYPE & obs)
{
	// rcv state from simulator

	//DetailedState state(CountMovingObjects(), true);
	//DetailedObservation observation;
	//InitStateIMP(state, observation);

	//if (state[0] == m_targetIdx)
	//{
	//	SendAction(NumActions());
	//	return true;
	//}
	//
	//s->state_id = nxnGridState::StateToIdx(state);
	//obs = nxnGridState::StateToIdx(observation);

	//reward = 0.0;
	//locationVec lastObsState;
	//nxnGridState::IdxToState(lastObs, lastObsState);

	//if (state[0] == m_gridSize * m_gridSize)
	//	reward = REWARD_LOSS;
	//else if (state[0] == m_targetIdx)
	//	reward = REWARD_WIN;
	//else if (EnemyRelatedAction(action) & (lastObsState[1] == m_gridSize * m_gridSize))
	//	reward = REWARD_ILLEGAL_MOVE;

	return false;
}

void nxnGrid::InitState()
{
	if (ToSendTree())
		SendModelDetails(NumActions(), CountAllObjects());

	if (s_isVBSEvaluator)
		InitStateVBS();
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
		DetailedState::s_shelters[i] = loc;
	}

}
/// init state from vbs simulator
void nxnGrid::InitStateVBS()
{
	locationVec state(CountMovingObjects());
	locationVec observation(CountMovingObjects());
	InitStateIMP(state, observation);

	int currObj = 0;
	Coordinate self(state[currObj] % m_gridSize, state[currObj] / m_gridSize);
	m_self.SetLocation(self);

	for (int i = 0; i < m_enemyVec.size(); ++i)
	{
		++currObj;
		Coordinate enemy;
		if (state[currObj] == m_gridSize * m_gridSize)
			enemy = Coordinate(m_gridSize, m_gridSize);
		else
			enemy = Coordinate(state[currObj] % m_gridSize, state[currObj] / m_gridSize);

		m_enemyVec[i].SetLocation(enemy);
	}

	for (int i = 0; i < m_nonInvolvedVec.size(); ++i)
	{
		++currObj;
		Coordinate ninv;
		if (state[currObj] == m_gridSize * m_gridSize)
			ninv = Coordinate(m_gridSize, m_gridSize);
		else
			ninv = Coordinate(state[currObj] % m_gridSize, state[currObj] / m_gridSize);

		m_nonInvolvedVec[i].SetLocation(ninv);
	}
}

void nxnGrid::InitStateIMP(locationVec & state, locationVec & observation)
{
	// TODO : read state from udp with movement
	
	//int buffer[40];
	//int length = s_udpVBS.Read(reinterpret_cast<char *>(buffer));

	//int vbsGridSize = buffer[0];

	//locationVec identity;
	//locationVec readState;
	//// read objects
	//for (int obj = 2; obj < length / sizeof(int); obj += 2)
	//{
	//	identity.emplace_back(buffer[obj - 1]);
	//	readState.emplace_back(buffer[obj]);
	//}

	//// organize objects according to identity
	//locationVec organizedState(readState.size() - 1);
	//int obj = 0;
	//organizedState[obj] = FindObject(readState, identity, SELF, 0);

	//++obj;
	//std::vector<bool> isObserved(organizedState.size());
	//isObserved[0] = true;

	//for (int i = 0; i < m_enemyVec.size(); ++i, ++obj)
	//{
	//	bool isObs;
	//	organizedState[obj] = FindObject(readState, identity, ENEMY_VBS, OBSERVED_ENEMY_VBS, isObs, i);
	//	isObserved[obj] = isObs;
	//}

	//for (int i = 0; i < m_nonInvolvedVec.size(); ++i, ++obj)
	//{
	//	bool isObs;
	//	organizedState[obj] = FindObject(readState, identity, NON_INVOLVED_VBS, OBSERVED_NON_INVOLVED_VBS, isObs, i);
	//	isObserved[obj] = isObs;
	//}

	//// read shelters
	//for (int i = 0; i < m_shelters.size(); ++i, ++obj)
	//	organizedState[obj] = FindObject(readState, identity, SHELTER, i);


	//// read and scale target
	//int vbsTarget = FindObject(readState, identity, TARGET, 0);
	//Coordinate targetC(vbsTarget % vbsGridSize, vbsTarget / vbsGridSize);
	//targetC *= static_cast<double>(m_gridSize) / vbsGridSize;
	//m_targetIdx = targetC.GetIdx(m_gridSize);
	//DetailedState::s_targetLoc = m_targetIdx;

	//locationVec scaledState(organizedState.size());
	//
	////ScaleState(organizedState, scaledState, m_gridSize, vbsGridSize);

	//// insert to observation and state moving objects
	//obj = 0;
	//for (; obj < CountMovingObjects(); ++obj)
	//{
	//	state[obj] = scaledState[obj];
	//	observation[obj] = state[obj] * isObserved[obj] + state[0] * (!isObserved[obj]);
	//}

	////insert scaled shelters location
	//for (int i = 0; i < m_shelters.size(); ++i, ++obj)
	//{
	//	DetailedState::s_shelters[i] = scaledState[obj];
	//	m_shelters[i].SetLocation(Coordinate(scaledState[obj] % m_gridSize, scaledState[obj] / m_gridSize));
	//}
}

int nxnGrid::FindObject(locationVec & state, locationVec & identity, int object, int idx)
{
	for (int i = 0; i < state.size(); ++i)
	{
		if (identity[i] == object)
		{
			if (idx == 0)
				return state[i];

			--idx;
		}
	}

	return m_gridSize * m_gridSize;
}

int nxnGrid::FindObject(locationVec & state, locationVec & identity, int object, int observedObj, bool & isObserved, int idx)
{
	for (int i = 0; i < state.size(); ++i)
	{
		if (identity[i] == object | identity[i] == observedObj)
		{
			if (idx == 0)
			{
				isObserved = identity[i] == observedObj;
				return state[i];
			}
			--idx;
		}
	}

	return m_gridSize * m_gridSize;
}

void nxnGrid::ChoosePreferredAction(const State * state, POMCPPrior * prior, const DSPOMDP* m, doubleVec & expectedRewards)
{
	const nxnGrid * model = static_cast<const nxnGrid *>(m);
	if (prior->history().Size() > 0)
	{
		DetailedState beliefState(model->CountMovingObjects(), true);
		// init self location and diretion
		DetailedState::GetSelfStatus(state->state_id, beliefState.m_locations[0], beliefState.m_directions[0]);

		model->InitBeliefState(beliefState, prior->history());
		model->ChoosePreferredActionIMP(beliefState, expectedRewards);
	}
	else
	{
		expectedRewards = doubleVec(model->NumActions(), REWARD_LOSS);
	}
}

int nxnGrid::ChoosePreferredAction(const State * state, POMCPPrior * prior, const DSPOMDP* m, double & expectedReward)
{	
	doubleVec rewards;
	ChoosePreferredAction(state, prior, m, rewards);
	// if calc type != without return lut result else return random decision
	if (s_calculationType != WITHOUT)
		return FindMaxReward(rewards, expectedReward);
	else
	{
		expectedReward = REWARD_LOSS;
		return rand() % m->NumActions();
	}
}

void nxnGrid::InitBeliefState(DetailedState & beliefState, const History & h) const
{
	DetailedObservation observedState(h.LastObservation());

	int nonValLoc = m_gridSize * m_gridSize + 1;
	int obj = 1;
	for (; obj < CountMovingObjects() ; ++obj)
	{
		// if the object is non observed treat it as dead
		int loc = nonValLoc;
		// run on all history and initialize state with last location observed on object
		for (int o = h.Size() - 1; o >= 0 & loc == nonValLoc; --o)
		{
			OBS_TYPE obs = h.Observation(o);
			loc = GetObsLoc(obs, obj);
		}

		beliefState.m_locations[obj] = loc * (loc != nonValLoc) + m_gridSize * m_gridSize * (loc == nonValLoc);
		// TODO : initialize direction in more correct way
		beliefState.m_directions[obj] = NO_DIRECTION;
	}
}

void nxnGrid::AddSheltersLocations(locationVec & state) const
{
	for (auto v : m_shelters)
		state.emplace_back(v.GetLocation().GetIdx(m_gridSize));
}

locationVec nxnGrid::GetSheltersVec() const
{
	locationVec shelters;
	for (auto v : m_shelters)
		shelters.emplace_back(v.GetLocation().GetIdx(m_gridSize));

	return shelters;
}
} //end ns despot