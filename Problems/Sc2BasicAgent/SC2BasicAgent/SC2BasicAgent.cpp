#include "SC2BasicAgent.h"

#include "Move_Properties.h"
#include "Coordinate.h"

#include <string>
#include <math.h>

namespace despot 
{

static OBS_TYPE s_ObsWin = 0;
static OBS_TYPE s_ObsLoss = 0;

static int SumVector(const intVec & vec)
{
	int sum = 0;
	for (auto v : vec)
		sum += v;

	return sum;
}
// init static members

intVec SC2DetailedState::s_NUM_BITS_PER_COUNT(InitNumBitsPerCount());
int SC2DetailedState::s_NUM_BITS_COUNT_PART(SumVector(s_NUM_BITS_PER_COUNT));

// state params
int SC2DetailedState::s_gridSize = 0;
int SC2DetailedState::s_baseLocation = 0;
int SC2DetailedState::s_enemyBaseLocation = 0;

std::vector<std::string> SC2DetailedState::s_countNames;
std::vector<char> SC2DetailedState::s_pixelStatusNames;

// model params
std::vector<std::string> SC2BasicAgent::s_ACTION_STR(NUM_ACTIONS);

const int OnlineSolverModel::REWARD_WIN = 50;;
const int OnlineSolverModel::REWARD_LOSS = -100.0;
const int SC2BasicAgent::REWARD_ILLEGAL_MOVE = -200.0;
const int SC2BasicAgent::REWARD_STEP = 0;

const int SC2BasicAgent::REWARD_MAX = REWARD_WIN;
const int SC2BasicAgent::REWARD_MIN = min(REWARD_LOSS, REWARD_ILLEGAL_MOVE);

/* =============================================================================
* State Functions
* =============================================================================*/

double DetailedState::ObsProb(STATE_TYPE state_id, OBS_TYPE obs)
{
	STATE_TYPE bitsFullyObsVars = (s_ONE << SC2DetailedState::s_NUM_BITS_COUNT_PART) - 1;

	STATE_TYPE s = state_id & bitsFullyObsVars;
	STATE_TYPE o = obs & bitsFullyObsVars;
	// if fully observed vars are not the obs prob is 0
	if (s != o)
		return 0.0;

	return SC2DetailedState::ObsProbPartialObsVars(state_id, obs);
}

SC2DetailedState::SC2DetailedState()
: m_countVec(NUM_COUNTS)
, m_miniMap(s_gridSize * s_gridSize, FREE)
{
	m_countVec[COMMAND_CENTER] = 1;
	m_miniMap[s_baseLocation] = BLUE;
	m_miniMap[s_enemyBaseLocation] = RED;
}

SC2DetailedState::SC2DetailedState(const State & state)
: m_countVec(NUM_COUNTS)
, m_miniMap(s_gridSize * s_gridSize)
{
	STATE_TYPE id = state.state_id;
	InitCountsFromId(id, m_countVec);
	InitMapFromId(id, m_miniMap);
}

SC2DetailedState::SC2DetailedState(STATE_TYPE stateId)
: m_countVec(NUM_COUNTS)
, m_miniMap(s_gridSize * s_gridSize)
{
	InitCountsFromId(stateId, m_countVec);
	InitMapFromId(stateId, m_miniMap);
}

SC2DetailedState::SC2DetailedState(int * arr)
: m_countVec(NUM_COUNTS)
, m_miniMap(s_gridSize * s_gridSize)
{
	int idx = 0;
	for (int i = 0; i < NUM_COUNTS; ++i, ++idx)
		m_countVec[i] = arr[idx];

	for (int i = 0; i < s_gridSize * s_gridSize; ++i, ++idx)
		m_miniMap[i] = (PIXEL)arr[idx];
}

intVec SC2DetailedState::InitNumBitsPerCount()
{
	intVec numBits(NUM_COUNTS);
	
	numBits[COMMAND_CENTER] = 1;
	numBits[SUPPLY_DEPOTS] = 3;
	numBits[BARRACKS] = 4;
	numBits[ARMY] = 8;
	numBits[MINERALS] = 16;

	return numBits;
}

void SC2DetailedState::InitCountsFromId(STATE_TYPE & stateId, intVec & countVec)
{
	// retrieve each location according to s_NUM_BITS_LOCATION
	for (int count = 0; count < NUM_COUNTS; ++count)
	{
		int bitsForSingleCount = (s_ONE << s_NUM_BITS_PER_COUNT[count]) - 1;
		countVec[count] = stateId & bitsForSingleCount;
		stateId >>= s_NUM_BITS_PER_COUNT[count];
	}
}

void SC2DetailedState::InitMapFromId(STATE_TYPE & stateId, miniMap_t & map)
{
	// init location of self
	int bitsForSinglePixel = (s_ONE << s_NUM_BITS_PIXEL) - 1;
	for (int singleCount = 0; singleCount < map.size(); ++singleCount)
	{
		map[singleCount] = static_cast<PIXEL>(stateId & bitsForSinglePixel);
		stateId >>= s_NUM_BITS_PIXEL;
	}
	
}

STATE_TYPE SC2DetailedState::GetStateId() const
{
	STATE_TYPE stateId;
	stateId = GetIDMiniMap(m_miniMap);
	stateId <<= s_NUM_BITS_COUNT_PART;
	stateId |= GetIDCounts(m_countVec);
	
	return stateId;
}

OBS_TYPE SC2DetailedState::GetObsId() const
{
	return GetStateId();
}

OBS_TYPE SC2DetailedState::GetObsId(double rand) const
{
	OBS_TYPE obsId = GetIDMiniMap(m_miniMap, rand);
	obsId <<= s_NUM_BITS_COUNT_PART;
	obsId |= GetIDCounts(m_countVec);

	return obsId;
}

void SC2DetailedState::CopyFullyObservedFromObservation(const OBS_TYPE obs, State * state)
{
	// count part is fully observable
	STATE_TYPE bits2Copy = (s_ONE << s_NUM_BITS_COUNT_PART) - 1;
	
	state->state_id &= ~bits2Copy;
	state->state_id |= (bits2Copy & obs);
}

STATE_TYPE SC2DetailedState::GetIDCounts(const intVec &countVec)
{
	STATE_TYPE id = 0;

	for (int count = NUM_COUNTS - 1; count >= 0; --count)
	{
		id <<= s_NUM_BITS_PER_COUNT[count];
		id |= countVec[count];
	}

	return id;
}

STATE_TYPE SC2DetailedState::GetIDMiniMap(const miniMap_t & miniMap)
{
	STATE_TYPE id = 0;
	for (int obj = miniMap.size() - 1; obj >= 0; --obj)
	{
		id <<= s_NUM_BITS_PIXEL;
		id |= miniMap[obj];
	}

	return id;
}

STATE_TYPE SC2DetailedState::GetIDMiniMap(const miniMap_t & miniMap, double random)
{
	// assumption base in corner
	int startX = s_baseLocation % s_gridSize;
	int startY = s_baseLocation / s_gridSize;

	int changeX = startX == 0 ? 1 : -1;
	int changeY = startY == 0 ? 1 : -1;

	
	std::vector<std::pair<int, double>> probMat;
	InitProbMat(miniMap, probMat);

	miniMap_t obsMap(miniMap.size());
	obsMap[s_baseLocation] = BLUE;
	GetIDMiniMapRec(obsMap, probMat, 1.0, random, 0);


	return GetIDMiniMap(obsMap);
}

void SC2DetailedState::InitProbMat(const miniMap_t & map, std::vector<std::pair<int, double>> &probMat)
{
	Coordinate base(s_baseLocation % s_gridSize, s_baseLocation / s_gridSize);
	for (int i = 0; i < s_gridSize * s_gridSize; ++i)
	{
		if (map[i] == RED)
		{
			Coordinate pnt(i % s_gridSize, i / s_gridSize);
			probMat.emplace_back(i, 1.0 / (pnt.Distance(base)));
		}
	}
}
void SC2DetailedState::GetIDMiniMapRec(miniMap_t & map, const std::vector<std::pair<int, double>> &probMat, double prob, double & random, int currIdx)
{
	if (currIdx == probMat.size())
	{
		random -= prob;
	}
	else
	{
		const std::pair<int, double> & locAndProb = probMat[currIdx];
		map[locAndProb.first] = FREE;
		GetIDMiniMapRec(map, probMat, prob * (1 - locAndProb.second), random, currIdx + 1);
		
		if (random <= 0)
			return;

		map[locAndProb.first] = RED;
		GetIDMiniMapRec(map, probMat, prob * locAndProb.second, random, currIdx + 1);
	}
}

double SC2DetailedState::ObsProbPartialObsVars(STATE_TYPE state_id, OBS_TYPE obs)
{
	SC2DetailedState realState(state_id);
	miniMap_t & stateMap(realState.GetMiniMap());

	// skip fully obs vars
	obs >>= s_NUM_BITS_COUNT_PART;
	miniMap_t observationMap(s_gridSize * s_gridSize);
	InitMapFromId(obs, observationMap);

	// run on obs map if there is red spot where cannot be return 0
	for (int i = 0; i < s_gridSize * s_gridSize; ++i)
	{
		if (observationMap[i] == RED && stateMap[i] != RED)
			return 0.0;
	}

	// don't treat blue spots for now
	std::vector<std::pair<int, double>> probMat;
	InitProbMat(stateMap, probMat);

	double p = 1.0;
	for (auto locAndProb : probMat)
	{
		p *= observationMap[locAndProb.first] == RED ? locAndProb.second : 1 - locAndProb.second;
	}

	return p;
}

void SC2DetailedState::StateVector(uintVec & vec) const
{
	int idx = 0;
	for (; idx < m_countVec.size(); ++idx)
		vec[idx] = m_countVec[idx];

	for (int pix = 0; pix < m_miniMap.size(); ++pix, ++idx)
	{
		vec[idx] = m_miniMap[pix];
	}

}

void SC2DetailedState::InitStatic()
{
	s_gridSize = -1;
	s_baseLocation = -1;
	s_enemyBaseLocation = -1;
	s_countNames.resize(NUM_COUNTS);
	s_countNames[COMMAND_CENTER] = "Command Center Count";
	s_countNames[SUPPLY_DEPOTS] = "Supply Depot Count";
	s_countNames[BARRACKS] = "Barracks Count";
	s_countNames[ARMY] = "Army Count";
	s_countNames[MINERALS] = "Minerals";

	s_pixelStatusNames.resize(NUM_PIXEL_OPTIONS);
	s_pixelStatusNames[FREE] = '_';
	s_pixelStatusNames[RED] = 'R';
	s_pixelStatusNames[BLUE] = 'B';
}

std::string SC2DetailedState::text() const
{
	std::string ret = "(";
	for (int i = 0; i < m_countVec.size(); ++i)
	{
		ret += s_countNames[i] + " = " + std::to_string(m_countVec[i]) + ", ";
	}
	ret += ")\n";

	ret += MiniMapStr();

	return ret;
}

std::string SC2DetailedState::MiniMapStr() const
{
	std::string ret;
	for (int x = 0; x < s_gridSize; ++x)
	{
		for (int y = 0; y < s_gridSize; ++y)
		{
			int idx = x + y * s_gridSize;
			if (idx == s_baseLocation)
				ret += s_pixelStatusNames[BLUE];
			else
				ret += s_pixelStatusNames[m_miniMap[idx]];
		}
		ret += "\n";
	}
	ret += "\n";

	return ret;
}

bool SC2DetailedState::MaxCount(int idxCount) const
{
	int maxCount = (1 << s_NUM_BITS_PER_COUNT[idxCount]) - 1;
	return m_countVec[idxCount] == maxCount;
}

int SC2DetailedState::EnemyPower() const
{
	int power = 0;
	for (auto pixel : m_miniMap)
	if (pixel == RED)
		++power;

	return power;
}

bool SC2DetailedState::NoEnemy() const
{
	for (auto pixel : m_miniMap)
	if (pixel == RED)
		return false;

	return true;
}

bool SC2DetailedState::RedNearBase() const
{
	// assumption : base is in corner

	int baseX = s_baseLocation % s_gridSize;
	int baseY = s_baseLocation / s_gridSize;

	int changeX = baseX == 0 ? 1 : -1;
	int changeY = baseY == 0 ? s_gridSize : -s_gridSize;

	bool redNear = m_miniMap[s_baseLocation + changeX] == RED;
	redNear |= m_miniMap[s_baseLocation + changeY] == RED;
	redNear |= m_miniMap[s_baseLocation + changeX + changeY] == RED;

	return redNear;
}


/* =============================================================================
* SC2BasicAgent Functions
* =============================================================================*/

void OnlineSolverModel::ChoosePreferredAction(POMCPPrior * prior, const DSPOMDP* m, doubleVec & expectedRewards)
{
	// TODO : update reward vec for all actions
	expectedRewards = doubleVec(m->NumActions(), SC2BasicAgent::REWARD_LOSS);
}

int OnlineSolverModel::ChoosePreferredAction(POMCPPrior * prior, const DSPOMDP* m, double & expectedReward)
{
	// TODO : return preffered action and update reward 
	return rand() % m->NumActions();
}

void OnlineSolverModel::InsertState2Buffer(uintVec & buffer, State * state)
{
	SC2DetailedState s(state->state_id);
	s.StateVector(buffer);
}

SC2BasicAgent::SC2BasicAgent(int gridSize, int baseLocation, int enemyBaseLoc)
	: m_gridSize(gridSize)
	, m_baseLocation(baseLocation)
	, m_enemyBaseLocation(enemyBaseLoc)
{
	// init size  of state for nxnGridstate
	SC2DetailedState::s_gridSize = gridSize;
	SC2DetailedState::s_baseLocation = baseLocation;
	SC2DetailedState::s_enemyBaseLocation = enemyBaseLoc;

	s_ACTION_STR[NOTHING] = "Nothing";
	s_ACTION_STR[BUILD_SUPPLY_DEPOT] = "Build Supply Depot";
	s_ACTION_STR[BUILD_BARRACKS] = "Build Barracks";
	s_ACTION_STR[TRAIN_ARMY] = "Train Army";
	s_ACTION_STR[ATTACK] = "Attack";
}


State * SC2BasicAgent::CreateStartState(std::string type) const
{
	SC2DetailedState state;
	return new OnlineSolverState(state.GetStateId());
}

Belief * SC2BasicAgent::InitialBelief(const State * start, std::string type) const
{
	std::vector<State*> particles;
	
	SC2DetailedState initState;
	particles.emplace_back(new State(initState.GetStateId(), 1.0));

	if (type == "SC2AgentBelief")
		return new SC2AgentBelief(particles, this);
	else
		return new ParticleBelief(particles, this);
}

bool SC2BasicAgent::Step(State& s, double randomObservation, int action, double& reward, OBS_TYPE& obs) const
{
	// termination if self army location is in enemy army base location and power and health of it is 0
	SC2DetailedState state(s);

	if (state.NoEnemy())
	{
		reward = REWARD_WIN;
		obs = s_ObsWin;
		return true;
	}
	else if (state[SC2DetailedState::COMMAND_CENTER] <= 0)
	{
		reward = REWARD_LOSS;
		obs = s_ObsLoss;
		return true;
	}

	// run on actions
	if (action == BUILD_SUPPLY_DEPOT)
		BuildSupplyDepot(state, reward);
	else if (action == BUILD_BARRACKS)
		BuildBarracks(state, reward);
	else if (action == TRAIN_ARMY)
		TrainArmy(state, reward);
	else if (action == ATTACK)
		Attack(state, RandomNum(), reward);

	// set next position of the objects on grid
	SetNextState(state, RandomNum(), RandomNum());

	reward = REWARD_STEP;

	//update state
	s.state_id = state.GetStateId();
	// update observation
	obs = state.GetObsId(randomObservation);
	return false;
}


void SC2BasicAgent::BuildSupplyDepot(SC2DetailedState & state, double & reward) const
{
	if (state[SC2DetailedState::MINERALS] < SUPPLY_DEPOT_PRICE)
		reward = REWARD_ILLEGAL_MOVE;
	else
	{
		++state[SC2DetailedState::SUPPLY_DEPOTS];
		state[SC2DetailedState::MINERALS] -= SUPPLY_DEPOT_PRICE;
	}

}

void SC2BasicAgent::BuildBarracks(SC2DetailedState & state, double & reward) const
{
	if (state[SC2DetailedState::MINERALS] < BARRACKS_PRICE || state[SC2DetailedState::SUPPLY_DEPOTS] <= state[SC2DetailedState::BARRACKS])
		reward = REWARD_ILLEGAL_MOVE;
	else
	{
		++state[SC2DetailedState::BARRACKS];
		state[SC2DetailedState::MINERALS] -= BARRACKS_PRICE;
	}
}

void SC2BasicAgent::TrainArmy(SC2DetailedState & state, double & reward) const
{
	if (state[SC2DetailedState::MINERALS] < TRAINING_PRICE || state[SC2DetailedState::BARRACKS] < 1)
		reward = REWARD_ILLEGAL_MOVE;
	else
	{
		++state[SC2DetailedState::ARMY];
		state[SC2DetailedState::MINERALS] -= TRAINING_PRICE;
	}
}


void SC2BasicAgent::Attack(SC2DetailedState & state, double random, double & reward) const
{
	if (state[SC2DetailedState::ARMY] > 10)
	{
		if (random > 0.5)
		{
			miniMap_t & map = state.GetMiniMap();

			for (int i = 0; i < map.size(); ++i)
			{
				if (map[i] == RED)
				{
					map[i] = FREE;
					break;
				}
			}

			--state[SC2DetailedState::ARMY];
		}
	}
	else if (state[SC2DetailedState::ARMY] > 0)
		--state[SC2DetailedState::ARMY];
}

bool SC2BasicAgent::LegalAction(OBS_TYPE observation, int action) const
{
	bool legalAction = true;
	SC2DetailedState observedState(observation);

	int minerals = observedState[SC2DetailedState::MINERALS];

	switch (action)
	{
	case BUILD_SUPPLY_DEPOT:
		legalAction = minerals > SUPPLY_DEPOT_PRICE || observedState.MaxCount(SC2DetailedState::SUPPLY_DEPOTS);
		break;
	
	case BUILD_BARRACKS:
		legalAction = minerals > BARRACKS_PRICE || observedState.MaxCount(SC2DetailedState::BARRACKS);
		break;
	case TRAIN_ARMY:
		legalAction = minerals > TRAINING_PRICE || observedState.MaxCount(SC2DetailedState::ARMY);
		break;
	case ATTACK:
	{
		int soldiers = observedState[SC2DetailedState::ARMY];
		legalAction = soldiers > 0;
		break;
	}
	default: break;
	}

	return legalAction;
}

void SC2BasicAgent::SetNextState(SC2DetailedState & state, double spreadingEnemy, double enemyAttack) const
{
	if (spreadingEnemy > .90)
	{
		miniMap_t & map = state.GetMiniMap();

		bool spread = false;
		for (int idx = 0; idx < map.size() && !spread; ++idx)
		{
			if (map[idx] == RED)
			{
				Coordinate pnt(idx % m_gridSize, idx / m_gridSize);

				for (int d = 0; d < SOUTH_EAST && !spread; ++d)
				{
					Coordinate dir(pnt + Move_Properties::s_directionsLUT[d]);
					int loc = dir.GetIdx(m_gridSize);

					if (dir.ValidLocation(m_gridSize) && map[loc] == FREE)
					{
						map[dir.GetIdx(m_gridSize)] = RED;
						spread = true;
					}

				}
			}
		}
	}

	if (enemyAttack > 0.75)
	{
		int attackPower = state.EnemyPower();

		if (state[SC2DetailedState::ARMY] > 0)
		{
			int powerNeed = min(state[SC2DetailedState::ARMY], attackPower);
			attackPower -= powerNeed;
			state[SC2DetailedState::ARMY] -= powerNeed;
		}

		if (attackPower > 0 && state.RedNearBase())
		{

			if (attackPower > 0 && state[SC2DetailedState::BARRACKS] > 0)
			{
				int powerNeed = min(state[SC2DetailedState::BARRACKS], attackPower / 3);
				attackPower -= powerNeed * 3;
				state[SC2DetailedState::BARRACKS] -= powerNeed;
			}

			if (attackPower > 0 && state[SC2DetailedState::SUPPLY_DEPOTS] > 0)
			{
				int powerNeed = min(state[SC2DetailedState::SUPPLY_DEPOTS], attackPower / 3);
				attackPower -= powerNeed * 3;
				state[SC2DetailedState::SUPPLY_DEPOTS] -= powerNeed;
			}
			else if (attackPower > 0 && state[SC2DetailedState::COMMAND_CENTER] > 0)
			{
				attackPower = (attackPower > 5);
				state[SC2DetailedState::COMMAND_CENTER] -= attackPower;
			}
		}
	}

	state[SC2DetailedState::MINERALS] += MINNING_RATE;
}

void SC2BasicAgent::DisplayParameters(std::ofstream & out) const
{
	out << "\n\nSC2Agent Model Details:\n";
	out << "grid size: " + std::to_string(m_gridSize) + "  base location: " + std::to_string(m_baseLocation);
}

void SC2BasicAgent::PrintState(const State & s, std::ostream & out) const
{
	SC2DetailedState state(s.state_id);
	out << state.text();
}

void SC2BasicAgent::PrintBelief(const Belief & belief, std::ostream & out) const
{
}

void SC2BasicAgent::PrintObs(const State & state, OBS_TYPE obs, std::ostream & out) const
{
	SC2DetailedState obsState(obs);
	out << obsState.text();
}

void SC2BasicAgent::PrintAction(int action, std::ostream & out) const
{
	out << s_ACTION_STR[action];
}

void SC2BasicAgent::InitState() 
{
	// TODO : there is not init state form simulator yet
}

bool SC2BasicAgent::RcvStateIMP(intVec & data, State * s, double & reward, OBS_TYPE & obs) const
{
	int buffer[40];
	int length = s_udpSimulator.Read(reinterpret_cast<char *>(buffer));

	SC2DetailedState observation(buffer);

	obs = observation.GetObsId();

	SC2DetailedState::CopyFullyObservedFromObservation(obs, s);
	return false;
}


void SC2AgentBelief::Update(int action, OBS_TYPE obs)
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
		SC2DetailedState::CopyFullyObservedFromObservation(obs, particle);
		double prob = model_->ObsProb(obs, *particle, action);

		if (!terminal && prob) 
		{ // Terminal state is not required to be explicitly represented and may not have any observation
			particle->weight *= prob;
			total_weight += particle->weight;
			updated.push_back(particle);
		}
		else {
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
		for (int i = 0; i < particles_.size(); i++) {
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

std::vector<State*> SC2AgentBelief::Resample(int num, const std::vector<State*>& belief, const DSPOMDP* model, History history)
{
	const SC2BasicAgent * modelCast = static_cast<const SC2BasicAgent *>(model);

	double unit = 1.0 / num;
	double mass = Random::RANDOM.NextDouble(0, unit);
	int pos = 0;
	double cur = belief[0]->weight;

	double reward;
	OBS_TYPE obs;

	std::vector<State*> sample;
	int count = 0;
	double max_wgt = Globals::NEG_INFTY;
	int trial = 0;
	while (count < num && trial < 200 * num) {
		// Pick next particle
		while (mass > cur) {
			pos++;
			if (pos == belief.size())
				pos = 0;

			cur += belief[pos]->weight;
		}
		trial++;

		mass += unit;

		State* particle = model->Copy(belief[pos]);

		// Step through history
		double log_wgt = 0;
		
		for (int i = 0; i < history.Size(); i++)
		{
			model->Step(*particle, Random::RANDOM.NextDouble(), history.Action(i), reward, obs);

			double prob = SC2DetailedState::ObsProbPartialObsVars(history.Observation(i), particle->state_id);
			//double prob = model->ObsProb(history.Observation(i), *particle, history.Action(i));
			if (prob > 0) {
				log_wgt += log(prob);
			}
			else {
				model->Free(particle);
				break;
			}
		}

		// Add to sample if survived
		if (particle->IsAllocated()) 
		{
			SC2DetailedState::CopyFullyObservedFromObservation(history.LastObservation(), particle);

			count++;

			particle->weight = log_wgt;
			sample.push_back(particle);

			max_wgt = max(log_wgt, max_wgt);
		}

		// Remove particles with very small weights
		if (count == num) 
		{
			for (int i = sample.size() - 1; i >= 0; i--)
			if (sample[i]->weight - max_wgt < log(1.0 / num)) {
				model->Free(sample[i]);
				sample.erase(sample.begin() + i);
				count--;
			}
		}
	}

	double total_weight = 0;
	for (int i = 0; i < sample.size(); i++) {
		sample[i]->weight = exp(sample[i]->weight - max_wgt);
		total_weight += sample[i]->weight;
	}
	for (int i = 0; i < sample.size(); i++) {
		sample[i]->weight = sample[i]->weight / total_weight;
	}

	logd << "[Belief::Resample] Resampled " << sample.size() << " particles\n";
	for (int i = 0; i < sample.size(); i++) {
		logv << " " << i << " = " << *sample[i] << std::endl;
	}

	return sample;
}

} //end ns despot