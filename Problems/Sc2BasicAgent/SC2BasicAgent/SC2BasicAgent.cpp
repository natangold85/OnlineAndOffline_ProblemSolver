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

intVec SC2DetailedState::s_NUM_BITS_PER_ENEMY_STATUS(InitNumBitsPerEnemyStatus());
int SC2DetailedState::s_NUM_BITS_ENEMY_STATUS_PART(SumVector(s_NUM_BITS_PER_COUNT));

// state params
int SC2DetailedState::s_gridSize = 0;
int SC2DetailedState::s_baseLocation = 0;
int SC2DetailedState::s_enemyBaseLocation = 0;

std::vector<std::string> SC2DetailedState::s_countNames;
std::vector<std::string> SC2DetailedState::s_enemyStatusNames;

// model params
SC2BasicAgent::OnlineSolverLUT SC2BasicAgent::s_Q_TABLE;
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
, m_enemyStatus(NUM_ENEMY_STATUS)
{
	m_countVec[COMMAND_CENTER] = 1;
	m_enemyStatus[BASE_POWER] = SC2BasicAgent::NUM_START_SCV + SC2BasicAgent::COMMAND_CENTER_2_POWER;
	m_enemyStatus[ARMY_LOCATION] = s_baseLocation;
	m_enemyStatus[ENEMY_OBSERVED_LOCATION] = s_NON_VALID_LOCATION;
}

SC2DetailedState::SC2DetailedState(const State & state)
: SC2DetailedState(state.state_id)
{
}

SC2DetailedState::SC2DetailedState(STATE_TYPE stateId)
: m_countVec(NUM_COUNTS)
, m_enemyStatus(NUM_ENEMY_STATUS)
{
	InitCountsFromId(stateId, m_countVec);
	InitEnemyStatusFromId(stateId, m_enemyStatus);
}

SC2DetailedState::SC2DetailedState(int * arr)
: m_countVec(NUM_COUNTS)
, m_enemyStatus(NUM_ENEMY_STATUS)
{
	int idx = 0;
	for (int i = 0; i < NUM_COUNTS; ++i, ++idx)
		m_countVec[i] = arr[idx];

	int idx = 0;
	for (int i = 0; i < NUM_COUNTS; ++i, ++idx)
		m_enemyStatus[i] = arr[idx];
}

void SC2DetailedState::InitStatic(int baseLocation, int enemyBaseLoc, int gridSize)
{
	s_gridSize = gridSize;
	s_baseLocation = baseLocation;
	s_enemyBaseLocation = enemyBaseLoc;
	s_countNames.resize(NUM_COUNTS);
	s_countNames[COMMAND_CENTER] = "Command Center Count";
	s_countNames[SUPPLY_DEPOTS] = "Supply Depot Count";
	s_countNames[BARRACKS] = "Barracks Count";
	s_countNames[ARMY] = "Army Count";
	s_countNames[MINERALS] = "Minerals";

	s_enemyStatusNames.resize(NUM_ENEMY_STATUS);
	s_enemyStatusNames[BASE_POWER] = "Enemy Base Power";
	s_enemyStatusNames[ARMY_POWER] = "Enemy Army Power";
	s_enemyStatusNames[ARMY_LOCATION] = "Enemy Army Location";
	s_enemyStatusNames[ENEMY_OBSERVED_LOCATION] = "Enemy Observed Location";
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

intVec SC2DetailedState::InitNumBitsPerEnemyStatus()
{
	intVec numBits(NUM_ENEMY_STATUS);

	numBits[BASE_POWER] = 8;
	numBits[ARMY_POWER] = 8;
	numBits[ARMY_LOCATION] = 4;
	numBits[ENEMY_OBSERVED_LOCATION] = 4;

	return numBits;
}
void SC2DetailedState::InitCountsFromId(STATE_TYPE & stateId, intVec & countVec)
{
	// retrieve each location according to num bits vec
	for (int count = 0; count < NUM_COUNTS; ++count)
	{
		int bitsForSingleCount = (s_ONE << s_NUM_BITS_PER_COUNT[count]) - 1;
		countVec[count] = stateId & bitsForSingleCount;
		stateId >>= s_NUM_BITS_PER_COUNT[count];
	}
}

void SC2DetailedState::InitEnemyStatusFromId(STATE_TYPE & stateId, intVec & enemyStatus)
{
	// retrieve each location according to num bits vec
	for (int count = 0; count < NUM_COUNTS; ++count)
	{
		int bitsForSingleCount = (s_ONE << s_NUM_BITS_PER_ENEMY_STATUS[count]) - 1;
		enemyStatus[count] = stateId & bitsForSingleCount;
		stateId >>= s_NUM_BITS_PER_ENEMY_STATUS[count];
	}
}

void SC2DetailedState::DeleteEnemyFromId(STATE_TYPE & state_id)
{
	STATE_TYPE onlyCountsBits = (s_ONE << s_NUM_BITS_COUNT_PART) - 1;
	state_id = state_id & onlyCountsBits;
}

STATE_TYPE SC2DetailedState::GetStateId() const
{
	STATE_TYPE stateId = GetIDEnemyStatus(m_enemyStatus);
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
	OBS_TYPE obsId = GetIDEnemyDetails(rand);
	obsId <<= s_NUM_BITS_COUNT_PART;
	obsId |= GetIDCounts(m_countVec);

	return obsId;
}

STATE_TYPE SC2DetailedState::GetIDEnemyDetails(double random) const
{
	// TODO : based on enemy transition lut and random number return id of enemy details (right now treat it as mdp)

	STATE_TYPE stateId;
	stateId = GetIDEnemyStatus(m_enemyStatus);

	return stateId;
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

STATE_TYPE SC2DetailedState::GetIDEnemyStatus(const intVec &enemyStatusVec)
{
	STATE_TYPE id = 0;

	for (int status = NUM_ENEMY_STATUS - 1; status >= 0; --status)
	{
		id <<= s_NUM_BITS_PER_ENEMY_STATUS[status];
		id |= enemyStatusVec[status];
	}

	return id;
}


double SC2DetailedState::ObsProbPartialObsVars(STATE_TYPE state_id, OBS_TYPE obs)
{
	SC2DetailedState realState(state_id);
	// TODO use transition from qTable
	return -1;
}

void SC2DetailedState::StateVector(uintVec & vec) const
{
	vec.resize(NUM_COUNTS + NUM_ENEMY_STATUS);
	int idx = 0;
	for (; idx < m_countVec.size(); ++idx)
		vec[idx] = m_countVec[idx];

	for (int i = 0; i < m_enemyStatus.size(); ++i, ++idx)
		vec[idx] = m_enemyStatus[i];

}

std::string SC2DetailedState::text() const
{
	std::string ret = "(";
	for (int i = 0; i < m_countVec.size(); ++i)
	{
		ret += s_countNames[i] + " = " + std::to_string(m_countVec[i]) + ", ";
	}
	ret += ")\n";

	return ret;
}


bool SC2DetailedState::MaxCount(int idxCount) const
{
	int maxCount = (1 << s_NUM_BITS_PER_COUNT[idxCount]) - 1;
	return m_countVec[idxCount] == maxCount;
}



bool SC2DetailedState::NoEnemy() const
{
	if (m_enemyStatus[BASE_POWER] == 0 && m_enemyStatus[ARMY_POWER] == 0)
		return true;
	return false;
}

bool SC2DetailedState::IsObservedEnemy() const
{
	return m_enemyStatus[ENEMY_OBSERVED_LOCATION] != s_NON_VALID_LOCATION;
}

bool SC2DetailedState::EnemyLocation(int & enemyPower) const
{
	if (m_enemyStatus[ENEMY_OBSERVED_LOCATION] != s_enemyBaseLocation && m_enemyStatus[ENEMY_OBSERVED_LOCATION] != s_NON_VALID_LOCATION)
	{
		enemyPower = m_enemyStatus[ARMY_POWER];
		return m_enemyStatus[ENEMY_OBSERVED_LOCATION];
	}
	else
	{
		enemyPower = m_enemyStatus[BASE_POWER];
		return s_enemyBaseLocation;
	}
}

bool SC2DetailedState::UpdatePower(int attackLocation, int power)
{
	bool valid = true;
	if (m_enemyStatus[ARMY_LOCATION] == attackLocation)
		m_enemyStatus[ARMY_POWER] = power;
	else if (s_enemyBaseLocation == attackLocation)
		m_enemyStatus[ARMY_POWER] = power;
	else
		valid = false;

	return valid;
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

SC2BasicAgent::SC2BasicAgent(int gridSize, int baseLocation)
	: m_gridSize(gridSize)
	, m_baseLocation(baseLocation)
	, m_enemyBaseLocation(0)
{
	if (m_baseLocation == 0)
		m_enemyBaseLocation = gridSize * gridSize - 1;

	// init size  of state for nxnGridstate
	SC2DetailedState::s_gridSize = gridSize;
	SC2DetailedState::s_baseLocation = baseLocation;
	SC2DetailedState::s_enemyBaseLocation = m_enemyBaseLocation;

	s_ACTION_STR[NOTHING] = "Nothing";
	s_ACTION_STR[BUILD_SUPPLY_DEPOT] = "Build Supply Depot";
	s_ACTION_STR[BUILD_BARRACKS] = "Build Barracks";
	s_ACTION_STR[TRAIN_ARMY] = "Train Army";
	s_ACTION_STR[ATTACK] = "Attack";
}

void SC2BasicAgent::InitQTable(OnlineSolverLUT & qTable, int gridSize, int baseLocation)
{
	auto model = new SC2BasicAgent(gridSize, baseLocation);
	s_Q_TABLE = qTable;

	// assessing self power transition to derive enemy transition by it
	int numStepsForState = 100;
	std::map<int, std::vector<int>> power2PowerTransition;
	for (auto itr : qTable)
	{
		intVec counts;
		State s(itr.first, 1);
		double reward;
		int action = OnlineSolverModel::FindMaxReward(itr.second, reward);

		SC2DetailedState::InitCountsFromId(s.state_id, counts);
		SC2DetailedState::DeleteEnemyFromId(s.state_id);

		int initialPower = NUM_START_SCV + SumVector(counts);
		
		OBS_TYPE obs = 0;
		int sumTransitionPower = 0;

		for (int i = 0; i < numStepsForState; ++i)
		{
			State s2(s);
			model->Step(s, static_cast<float>(i) / numStepsForState, action, reward, obs);
			SC2DetailedState::InitCountsFromId(s.state_id, counts);
			sumTransitionPower += NUM_START_SCV + SumVector(counts);
		}

		int endPower = sumTransitionPower / numStepsForState;
		power2PowerTransition[initialPower].emplace_back(endPower);
	}

	for (auto itr : power2PowerTransition)
	{
		s_ENEMY_TRANSITION_LUT[itr.first] = SumVector(itr.second) / itr.second.size();
	}

	delete model;
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
		buildSupplyDepot(state, reward);
	else if (action == BUILD_BARRACKS)
		buildBarracks(state, reward);
	else if (action == TRAIN_ARMY)
		trainArmy(state, reward);
	else if (action == ATTACK)
		attack(state, RandomNum(), reward);

	// set next position of the objects on grid
	setNextState(state, RandomNum(), RandomNum());

	reward = REWARD_STEP;

	//update state
	s.state_id = state.GetStateId();
	// update observation
	obs = state.GetObsId(randomObservation);
	return false;
}


void SC2BasicAgent::buildSupplyDepot(SC2DetailedState & state, double & reward) const
{
	if (state[SC2DetailedState::MINERALS] < SUPPLY_DEPOT_PRICE)
		reward = REWARD_ILLEGAL_MOVE;
	else
	{
		++state[SC2DetailedState::SUPPLY_DEPOTS];
		state[SC2DetailedState::MINERALS] -= SUPPLY_DEPOT_PRICE;
	}

}

void SC2BasicAgent::buildBarracks(SC2DetailedState & state, double & reward) const
{
	if (state[SC2DetailedState::MINERALS] < BARRACKS_PRICE || state[SC2DetailedState::SUPPLY_DEPOTS] <= state[SC2DetailedState::BARRACKS])
		reward = REWARD_ILLEGAL_MOVE;
	else
	{
		++state[SC2DetailedState::BARRACKS];
		state[SC2DetailedState::MINERALS] -= BARRACKS_PRICE;
	}
}

void SC2BasicAgent::trainArmy(SC2DetailedState & state, double & reward) const
{
	if (state[SC2DetailedState::MINERALS] < TRAINING_PRICE || state[SC2DetailedState::BARRACKS] < 1)
		reward = REWARD_ILLEGAL_MOVE;
	else
	{
		++state[SC2DetailedState::ARMY];
		state[SC2DetailedState::MINERALS] -= TRAINING_PRICE;
	}
}


void SC2BasicAgent::attack(SC2DetailedState & state, double random, double & reward) const
{
	int attackLocation;
	int enemyPower;
	attackLocation = state.EnemyLocation(enemyPower);
	pairInt src(state[SC2DetailedState::ARMY], enemyPower);
	auto result = s_ATTACKS_LUT.find(src);
	if (result != s_ATTACKS_LUT.end())
	{
		
		// TODO : use attack lut to understand attack results
		state[SC2DetailedState::ARMY] = result->second.first;
		state.UpdatePower(attackLocation, result->second.second);
	}
	else
	{
		// naive modelling of attack
		enemyPower -= src.first;
		state[SC2DetailedState::ARMY] -= src.second / 2;
		state.UpdatePower(attackLocation, enemyPower);
	}


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

void SC2BasicAgent::setNextState(SC2DetailedState & state, double spreadingEnemy, double enemyAttack) const
{
	// TODO : enemy development simulation

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