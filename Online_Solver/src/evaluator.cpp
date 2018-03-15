#include "../include/despot/evaluator.h"

#include "../include/despot/solver/pomcp.h"

#include "../include/despot/OnlineSolverModel.h"

using namespace std;

namespace despot {

inline int Max(int a, int b)
{
	return a * (a >= b) + b * (a < b);
}


/* =============================================================================
 * EvalLog class
 * =============================================================================*/

time_t EvalLog::start_time = 0;
double EvalLog::curr_inst_start_time = 0;
double EvalLog::curr_inst_target_time = 0;
double EvalLog::curr_inst_budget = 0;
double EvalLog::curr_inst_remaining_budget = 0;
int EvalLog::curr_inst_steps = 0;
int EvalLog::curr_inst_remaining_steps = 0;
double EvalLog::allocated_time = 1.0;
double EvalLog::plan_time_ratio = 1.0;

EvalLog::EvalLog(string log_file) :
	log_file_(log_file) {
	ifstream fin(log_file_.c_str(), ifstream::in);
	if (!fin.good() || fin.peek() == ifstream::traits_type::eof()) {
		time(&start_time);
	} else {
		fin >> start_time;

		int num_instances;
		fin >> num_instances;
		for (int i = 0; i < num_instances; i++) {
			string name;
			int num_runs;
			fin >> name >> num_runs;
			runned_instances.push_back(name);
			num_of_completed_runs.push_back(num_runs);
		}
	}
	fin.close();
}

void EvalLog::Save() {
	ofstream fout(log_file_.c_str(), ofstream::out);
	fout << start_time << endl;
	fout << runned_instances.size() << endl;
	for (int i = 0; i < runned_instances.size(); i++)
		fout << runned_instances[i] << " " << num_of_completed_runs[i] << endl;
	fout.close();
}

void EvalLog::IncNumOfCompletedRuns(string problem) {
	bool seen = false;
	for (int i = 0; i < runned_instances.size(); i++) {
		if (runned_instances[i] == problem) {
			num_of_completed_runs[i]++;
			seen = true;
		}
	}

	if (!seen) {
		runned_instances.push_back(problem);
		num_of_completed_runs.push_back(1);
	}
}

int EvalLog::GetNumCompletedRuns() const {
	int num = 0;
	for (int i = 0; i < num_of_completed_runs.size(); i++)
		num += num_of_completed_runs[i];
	return num;
}

int EvalLog::GetNumRemainingRuns() const {
	return 80 * 30 - GetNumCompletedRuns();
}

int EvalLog::GetNumCompletedRuns(string instance) const {
	for (int i = 0; i < runned_instances.size(); i++) {
		if (runned_instances[i] == instance)
			return num_of_completed_runs[i];
	}
	return 0;
}

int EvalLog::GetNumRemainingRuns(string instance) const {
	return 30 - GetNumCompletedRuns(instance);
}

double EvalLog::GetUsedTimeInSeconds() const {
	time_t curr;
	time(&curr);
	return (double) (curr - start_time);
}

double EvalLog::GetRemainingTimeInSeconds() const {
	return 24 * 3600 - GetUsedTimeInSeconds();
}

// Pre-condition: curr_inst_start_time is initialized
void EvalLog::SetInitialBudget(string instance) {
	curr_inst_budget = 0;
	if (GetNumRemainingRuns() != 0 && GetNumRemainingRuns(instance) != 0) {
		cout << "Num of remaining runs: curr / total = "
			<< GetNumRemainingRuns(instance) << " / " << GetNumRemainingRuns()
			<< endl;
		curr_inst_budget = (24 * 3600 - (curr_inst_start_time - start_time))
			/ GetNumRemainingRuns() * GetNumRemainingRuns(instance);
		if (curr_inst_budget < 0)
			curr_inst_budget = 0;
		if (curr_inst_budget > 18 * 60)
			curr_inst_budget = 18 * 60;
	}
}

double EvalLog::GetRemainingBudget(string instance) const {
	return curr_inst_budget
		- (get_time_second() - EvalLog::curr_inst_start_time);
}

/* =============================================================================
 * Evaluator class
 * =============================================================================*/

Evaluator::Evaluator(DSPOMDP* model, string belief_type, SolverBase * solver,
	clock_t start_clockt, ostream* out) :
	model_(model),
	belief_type_(belief_type),
	solver_(solver),
	start_clockt_(start_clockt),
	target_finish_time_(-1),
	out_(out)
{
}

Evaluator::~Evaluator()
{
}

void Evaluator::RunRoundForParallel(int round)
{
	ParallelSolver *ptr = reinterpret_cast<ParallelSolver *>(solver_);
	std::thread treeMngr([ptr] { ptr->ThreadsMngrFunction(); });

	TreeMngrThread & treeMngrData = ptr->GetTreeMngrData();
	Tree_Properties treeProp(model_->NumActions());
	bool terminal = false;

	for (int step = 0; step < Globals::config.sim_len && !terminal; ++step)
	{
		Sleep(Globals::config.time_per_move * 1000);
		// inform to builder that action is needed
		{
			std::lock_guard<std::mutex> lock(treeMngrData.m_flagsMutex);
			treeMngrData.m_actionNeeded = true;
		}
		// wait for action is ready
		bool actionRecieved = false;
		while (!actionRecieved)
		{
			std::lock_guard<std::mutex> lock(treeMngrData.m_flagsMutex);
			actionRecieved = treeMngrData.m_actionRecieved;
		}

		int action = -1;
		double reward;
		OBS_TYPE obs;
		State currState;

		{ // take action and update observation
			std::lock_guard<std::mutex> mainLock(treeMngrData.m_mainMutex);
			action = treeMngrData.m_action;

			terminal = ExecuteAction(action, reward, obs);
			currState.state_id = state_->state_id;

			GetTreeProperties(treeProp);

			treeMngrData.m_lastObservation = obs;
			std::lock_guard<std::mutex> flagsLock(treeMngrData.m_flagsMutex);
			treeMngrData.m_observationRecieved = true;
			treeMngrData.m_actionRecieved = false;
		}

		// print data
		std::cout << "- Tree Properties: (size,count,value)\n" << treeProp.text() << "\n";

		std::cout << "- Action = ";
		model_->PrintAction(action);
		std::cout << "- Current State: : ";
		model_->PrintState(currState);
		std::cout << "- Observation: : ";
		model_->PrintObs(currState, obs);
		std::cout << "- ObsProb = " << model_->ObsProb(obs, currState, action) << "\n";

		std::cout << "\n- StepReward = " << reward << "\n\n\n";
	}

	{ // inform to builder that round is over
		std::lock_guard<std::mutex> lock(treeMngrData.m_flagsMutex);
		treeMngrData.m_terminal = true;
	}

	treeMngr.join();
}

bool Evaluator::RunStep(int step, int round) {
	if (target_finish_time_ != -1 && get_time_second() > target_finish_time_) {
		if (!Globals::config.silence && out_)
			*out_ << "Exit. (Total time "
				<< (get_time_second() - EvalLog::curr_inst_start_time)
				<< "s exceeded time limit of "
				<< (target_finish_time_ - EvalLog::curr_inst_start_time) << "s)"
				<< endl
				<< "Total time: Real / CPU = "
				<< (get_time_second() - EvalLog::curr_inst_start_time) << " / "
				<< (double(clock() - start_clockt_) / CLOCKS_PER_SEC) << "s"
				<< endl;
		exit(1);
	}

	double step_start_t = get_time_second();

	 double start_t = get_time_second();

	
	int action;
	double startStep = get_time_second();
	if (OnlineSolverModel::IsOnline())
		action = solver_->Search().action;
	else
	{
		double reward;
		action = static_cast<OnlineSolverModel *>(model_)->ChoosePreferredAction(static_cast<POMCP *>(solver_)->GetPrior(), model_, reward);
	}

	double endSearch = get_time_second();
	
	
	double end_t = get_time_second();
	logi << "[RunStep] Time spent in " << typeid(*solver_).name()
		<< "::Search(): " << (end_t - start_t) << endl;

	start_t = get_time_second();
	*out_ << "-----------------------------------Round " << round
		<< " Step " << step << "-----------------------------------"
		<< endl;

	
	double reward;
	OBS_TYPE obs;
	bool terminal;


	if (OnlineSolverModel::ToSendTree())
		static_cast<POMCP *>(solver_)->SendTree(state_);

	terminal = ExecuteAction(action, reward, obs);
	
	end_t = get_time_second();
	logi << "[RunStep] Time spent in ExecuteAction(): " << (end_t - start_t)
		<< endl;

	start_t = get_time_second();

	// gathering tree properties
	Tree_Properties actionsTreeData(model_->NumActions());
	GetTreeProperties(actionsTreeData);
		

	std::cout << "- Tree Properties: (size,count,value)\n" << actionsTreeData.text() << "\n";

	if (!Globals::config.silence && out_) {
		*out_ << "\n- Action = ";
		model_->PrintAction(action, *out_);
	}

	if (state_ != NULL) {
		if (!Globals::config.silence && out_) {
			*out_ << "\n- Current State: : ";
			model_->PrintState(*state_, *out_);
		}
	}

	if (!Globals::config.silence && out_) {
		*out_ << "\n- Observation = ";
		model_->PrintObs(*state_, obs, *out_);
	}

	if (state_ != NULL) {
		if (!Globals::config.silence && out_)
			*out_ << "\n- ObsProb = " << model_->ObsProb(obs, *state_, action)
				<< endl;
	}

	if (!Globals::config.silence && out_) {
	/*	*out_ << "- Belief = ";
		model_->PrintBelief(*(solver_->belief()), *out_);*/
	}

	ReportStepReward();
	end_t = get_time_second();

	double step_end_t;
	if (terminal) {
		model_->PrintState(*state_, *out_);
		step_end_t = get_time_second();
		logi << "[RunStep] Time for step: actual / allocated = "
			<< (step_end_t - step_start_t) << " / " << EvalLog::allocated_time
			<< endl;
		if (!Globals::config.silence && out_)
			*out_ << endl;
		step_++;

		// win = 1, loss = 2, tie = 0
		int result = reward == OnlineSolverModel::REWARD_WIN ? 1 : OnlineSolverModel::REWARD_LOSS ? 2 : 0;
		winsVec.push_back(result);

		return true;
	}

	*out_<<endl;

	start_t = get_time_second();

	// update action and observation in history and belief state(not exist in offline)
	if (OnlineSolverModel::IsOnline())
		solver_->Update(action, obs);
	else
		solver_->UpdateHistory(action, obs);

	end_t = get_time_second();
	logi << "[RunStep] Time spent in Update(): " << (end_t - start_t) << endl;
	std::cout << "\nsearch time = " << endSearch - startStep << " step time = " << end_t - startStep << "\n\n";
	step_++;
	return false;
}

void Evaluator::UserPlayRound()
{
	bool terminal = false;
	while (!terminal)
	{
		int action = -1;
		double reward;
		OBS_TYPE obs;

		std::cout << "Enter Action(";

		for (int a = 0; a < model_->NumActions(); ++a)
		{
			std::cout << a << " - ";
			model_->PrintAction(a);
			std::cout << ", ";
		}
		std::cout << ")\n";

		std::cin >> action;
		terminal = ExecuteAction(action, reward, obs);

		std::cout << "\n- Action = ";
		model_->PrintAction(action);
		std::cout << "\n- Current State: : ";
		model_->PrintState(*state_);
		std::cout << "\n- Observation: : ";
		model_->PrintObs(*state_, obs);
		std::cout << "\n- ObsProb = " << model_->ObsProb(obs, *state_, action) << "\n";

		std::cout << "\n- StepReward = " << reward << "\n\n\n";
	}
}
double Evaluator::AverageUndiscountedRoundReward() const {
	double sum = 0;
	for (int i = 0; i < undiscounted_round_rewards_.size(); i++) {
		double reward = undiscounted_round_rewards_[i];
		sum += reward;
	}
	return undiscounted_round_rewards_.size() > 0 ? (sum / undiscounted_round_rewards_.size()) : 0.0;
}

double Evaluator::StderrUndiscountedRoundReward() const {
	double sum = 0, sum2 = 0;
	for (int i = 0; i < undiscounted_round_rewards_.size(); i++) {
		double reward = undiscounted_round_rewards_[i];
		sum += reward;
		sum2 += reward * reward;
	}
	int n = undiscounted_round_rewards_.size();
	return n > 0 ? sqrt(sum2 / n / n - sum * sum / n / n / n) : 0.0;
}


double Evaluator::AverageDiscountedRoundReward() const {
	double sum = 0;
	for (int i = 0; i < discounted_round_rewards_.size(); i++) {
		double reward = discounted_round_rewards_[i];
		sum += reward;
	}
	return discounted_round_rewards_.size() > 0 ? (sum / discounted_round_rewards_.size()) : 0.0;
}

double Evaluator::StderrDiscountedRoundReward() const {
	double sum = 0, sum2 = 0;
	for (int i = 0; i < discounted_round_rewards_.size(); i++) {
		double reward = discounted_round_rewards_[i];
		sum += reward;
		sum2 += reward * reward;
	}
	int n = discounted_round_rewards_.size();
	return n > 0 ? sqrt(sum2 / n / n - sum * sum / n / n / n) : 0.0;
}

void Evaluator::AddDiscounted2String(std::string & buffer) const // NATAN CHANGES
{
	buffer += "discounted reward:\n";
	for (auto v : discounted_round_rewards_) 
		buffer += std::to_string(v) + ", ";
	buffer += "\n";
}

void Evaluator::AddUnDiscounted2String(std::string & buffer) const // NATAN CHANGES
{
	buffer += "undiscounted reward:\n";
	for (auto v : undiscounted_round_rewards_)
		buffer += std::to_string(v) + ", ";
}

void Evaluator::PrintFinalResults(std::string & buffer) const
{
	buffer += "results:\n";
	for (auto v : winsVec)
		buffer += std::to_string(v) + ", ";
	buffer += "\n";
}


void Evaluator::PrintTreeProp(std::string & buffer) const
{

	//buffer += "\n\nHeight:\n";
	//double maxHeight = 0;
	//for (auto v : tree_properties_)
	//{
	//	if (isinf(v.m_height))
	//		buffer += "0, ";
	//	else
	//	{
	//		maxHeight = Max(maxHeight, v.m_height);
	//		buffer += std::to_string(v.m_height) + ", ";
	//	}
	//}

	//buffer += "\n\Size:\n";
	//for (auto v : tree_properties_)
	//{
	//	if (isinf(v.m_size))
	//		buffer += "0, ";
	//	else
	//		buffer += std::to_string(v.m_size) + ", ";
	//}

	//buffer += "\n\level sizes:\n";
	//for (int i = 0; i < maxHeight - 1; ++i)
	//{
	//	for (auto v : tree_properties_)
	//	{
	//		if (i < v.m_levelSize.size())
	//			buffer += std::to_string(v.m_levelSize[i]) + ", ";
	//		else
	//			buffer += "0 , ";
	//	}
	//	buffer += "\n";
	//}


	//buffer += "\nlevel portion:\n";
	//for (int i = 0; i < maxHeight - 1; ++i)
	//{
	//	for (auto v : tree_properties_)
	//	{
	//		if (i < v.m_levelSize.size())
	//			buffer += std::to_string(v.m_preferredActionPortion[i]) + ", ";
	//		else
	//			buffer += "0 , ";
	//	}
	//	buffer += "\n";
	//}

}

void Evaluator::ReportStepReward() {
	// NOTE: The rewards are incorrectly computed at last step in IPPC
	if (!Globals::config.silence && out_)
		*out_ << "- Reward = " << reward_ << endl
			<< "- Current rewards:" << endl
			<< "  discounted / undiscounted = " << total_discounted_reward_
			<< " / " << total_undiscounted_reward_ << endl;
}

/* =============================================================================
 * IPPCEvaluator class
 * =============================================================================*/

IPPCEvaluator::IPPCEvaluator(DSPOMDP* model, string belief_type, SolverBase * solver,
	clock_t start_clockt, string hostname, string port, string log,
	ostream* out) :
	Evaluator(model, belief_type, solver, start_clockt, out),
	pomdpx_(static_cast<POMDPX*>(model)),
	log_(log),
	hostname_(hostname),
	port_(port) {
}

IPPCEvaluator::~IPPCEvaluator() {
}

int IPPCEvaluator::Handshake(string instance) {
	int num_remaining_runs = log_.GetNumRemainingRuns(instance);
	if (num_remaining_runs == 0) {
		return 0;
	}

	double start_t = get_time_second();
	instance_ = instance;

	client_ = new Client();
	client_->setHostName(hostname_);
	client_->setPort(port_);

	client_->initializeSocket();
	client_->connectToServer();

	client_->sendMessage(client_->createSessionRequestMes(instance));

	string sessionInitMes = client_->recvMessage();

	if (!Globals::config.silence && out_) {
		*out_ << sessionInitMes << endl;
	}

	client_->processSessionInitMes(sessionInitMes);
	double end_t = get_time_second();

	if (!Globals::config.silence && out_) {
		*out_ << "Time for handsake " << (end_t - start_t) << endl;
	}

	log_.SetInitialBudget(instance);
	EvalLog::curr_inst_steps = num_remaining_runs * Globals::config.sim_len;
	EvalLog::curr_inst_remaining_steps = num_remaining_runs
		* Globals::config.sim_len;
	EvalLog::curr_inst_target_time = EvalLog::curr_inst_budget
		/ EvalLog::curr_inst_steps;
	UpdateTimeInfo(instance);
	EvalLog::plan_time_ratio = 1.0;
	Globals::config.time_per_move = EvalLog::plan_time_ratio
		* EvalLog::allocated_time;

	return num_remaining_runs;
}

void IPPCEvaluator::InitRound(bool isParallelSolver) {
	step_ = 0;
	state_ = NULL;

	double start_t, end_t;

	// Initial belief
	start_t = get_time_second();
	delete solver_->belief();
	end_t = get_time_second();
	logi << "[IPPCEvaluator::InitRound] Deleted initial belief in "
		<< (end_t - start_t) << "s" << endl;

	start_t = get_time_second();
	
	if (isParallelSolver)
	{	// create belief for all solvers
		ParallelSolver * parSolver = dynamic_cast<ParallelSolver *>(solver_);
		for (int sol = 0; sol < parSolver->NumSolvers(); ++sol)
		{
			Belief* belief = model_->InitialBelief(NULL, belief_type_);
			parSolver->belief(belief, sol);
		}
	}
	else
	{

		Belief* belief = model_->InitialBelief(NULL, belief_type_);
		end_t = get_time_second();
		logi << "[IPPCEvaluator::InitRound] Initialized initial belief: "
			<< typeid(*belief).name() << " in " << (end_t - start_t) << "s" << endl;
		solver_->belief(belief);
	}


	// Initiate a round with server
	start_t = get_time_second();
	client_->sendMessage(client_->createRoundRequestMes());
	string roundMes = client_->recvMessageTwice();
	end_t = get_time_second();
	logi << "[IPPCEvaluator::InitRound] Time for startround msg "
		<< (end_t - start_t) << "s" << endl;

	total_discounted_reward_ = 0;
	total_undiscounted_reward_ = 0;
}

double IPPCEvaluator::EndRound() {
	double start_t = get_time_second();

	string roundEndMes = client_->recvMessage();
	double round_reward = client_->processRoundEndMes(roundEndMes);

	if (!Globals::config.silence && out_) {
		*out_ << "Total undiscounted reward = " << round_reward << endl;
	}

	log_.IncNumOfCompletedRuns(instance_);
	log_.Save();

	double end_t = get_time_second();

	if (!Globals::config.silence && out_) {
		*out_ << "Time for endround msg (save log) " << (end_t - start_t)
			<< endl;
	}

	discounted_round_rewards_.push_back(total_discounted_reward_);
	undiscounted_round_rewards_.push_back(round_reward);

	return round_reward;
}

bool IPPCEvaluator::ExecuteAction(int action, double& reward, OBS_TYPE& obs) {
	double start_t = get_time_second();

	client_->sendMessage(
		client_->createActionMes(pomdpx_->GetActionName(),
			pomdpx_->GetEnumedAction(action)));

	if (step_ == Globals::config.sim_len - 1) {
		return true;
	}

	string turnMes = client_->recvMessage();

	//get step reward from turn message: added by wkg
	reward = client_->getStepReward(turnMes);
	reward_ = reward;
	total_discounted_reward_ += Globals::Discount(step_) * reward;
	total_undiscounted_reward_ += reward;

	map<string, string> observs = client_->processTurnMes(turnMes);
	obs = pomdpx_->GetPOMDPXObservation(observs);

	double end_t = get_time_second();

	if (!Globals::config.silence && out_) {
		*out_ << "Time for executing action " << (end_t - start_t) << endl;
	}

	return false;
}

double IPPCEvaluator::End() {
	double start_t = get_time_second();

	string sessionEndMes = client_->recvMessage();
	double total_reward = client_->processSessionEndMes(sessionEndMes);
	client_->closeConnection();
	delete client_;

	double end_t = get_time_second();

	if (!Globals::config.silence && out_) {
		*out_ << "Time for endsession " << (end_t - start_t) << endl
			<< "Total reward for all runs = " << total_reward << endl
			<< "Total time: Real / CPU = "
			<< (get_time_second() - EvalLog::curr_inst_start_time) << " / "
			<< (double(clock() - start_clockt_) / CLOCKS_PER_SEC) << "s"
			<< endl;
	}

	return total_reward;
}

void IPPCEvaluator::UpdateTimeInfo(string instance) {
	EvalLog::curr_inst_remaining_budget = log_.GetRemainingBudget(instance);
	if (EvalLog::curr_inst_remaining_budget <= 0) {
		EvalLog::curr_inst_remaining_budget = 0;
	}

	if (EvalLog::curr_inst_remaining_steps <= 0) {
		EvalLog::allocated_time = 0;
	} else {
		EvalLog::allocated_time = (EvalLog::curr_inst_remaining_budget - 2.0)
			/ EvalLog::curr_inst_remaining_steps;

		if (EvalLog::allocated_time > 5.0)
			EvalLog::allocated_time = 5.0;
	}
}

void IPPCEvaluator::UpdateTimePerMove(double step_time) {
	if (step_time < 0.99 * EvalLog::allocated_time) {
		if (EvalLog::plan_time_ratio < 1.0)
			EvalLog::plan_time_ratio += 0.01;
		if (EvalLog::plan_time_ratio > 1.0)
			EvalLog::plan_time_ratio = 1.0;
	} else if (step_time > EvalLog::allocated_time) {
		double delta = (step_time - EvalLog::allocated_time)
			/ (EvalLog::allocated_time + 1E-6);
		if (delta < 0.02)
			delta = 0.02; // Minimum reduction per step
		if (delta > 0.05)
			delta = 0.05; // Maximum reduction per step
		EvalLog::plan_time_ratio -= delta;
		// if (EvalLog::plan_time_ratio < 0)
		// EvalLog::plan_time_ratio = 0;
	}

	EvalLog::curr_inst_remaining_budget = log_.GetRemainingBudget(instance_);
	EvalLog::curr_inst_remaining_steps--;

	UpdateTimeInfo(instance_);
	Globals::config.time_per_move = EvalLog::plan_time_ratio
		* EvalLog::allocated_time;

	if (!Globals::config.silence && out_) {
		*out_
			<< "Total time: curr_inst / inst_target / remaining / since_start = "
			<< (get_time_second() - EvalLog::curr_inst_start_time) << " / "
			<< (EvalLog::curr_inst_target_time
				* (EvalLog::curr_inst_steps - EvalLog::curr_inst_remaining_steps))
			<< " / " << EvalLog::curr_inst_remaining_budget << " / "
			<< (get_time_second() - EvalLog::start_time) << endl;
	}
}

/* =============================================================================
 * POMDPEvaluator class
 * =============================================================================*/

POMDPEvaluator::POMDPEvaluator(DSPOMDP* model, string belief_type,
	SolverBase * solver, clock_t start_clockt, ostream* out,
	double target_finish_time, int num_steps) :
	Evaluator(model, belief_type, solver, start_clockt, out),
	random_((unsigned) 0) {
	target_finish_time_ = target_finish_time;

	if (target_finish_time_ != -1) {
		EvalLog::allocated_time = (target_finish_time_ - get_time_second())
			/ num_steps;
		Globals::config.time_per_move = EvalLog::allocated_time;
		EvalLog::curr_inst_remaining_steps = num_steps;
	}
}

POMDPEvaluator::~POMDPEvaluator() {
}

int POMDPEvaluator::Handshake(string instance) {
	return -1; // Not to be used
}

void POMDPEvaluator::InitRound(bool isParallelSolver) {
	step_ = 0;

	double start_t, end_t;
	// Initial state
	state_ = model_->CreateStartState();
	logi << "[POMDPEvaluator::InitRound] Created start state." << endl;
	if (!Globals::config.silence && out_) {
		*out_ << "Initial state: " << endl;
		model_->PrintState(*state_, *out_);
		*out_ << endl;
	}

	// Initial belief
	start_t = get_time_second();

	solver_->DeleteBelief();

	end_t = get_time_second();
	logi << "[POMDPEvaluator::InitRound] Deleted old belief in "
		<< (end_t - start_t) << "s" << endl;

	start_t = get_time_second();
	if (isParallelSolver)
	{	// create belief for all solvers
		ParallelSolver * parSolver = dynamic_cast<ParallelSolver *>(solver_);
		for (int sol = 0; sol < parSolver->NumSolvers(); ++sol)
		{
			Belief* belief = model_->InitialBelief(NULL, belief_type_);
			parSolver->belief(belief, sol);
		}
	}
	else
	{
		Belief* belief = model_->InitialBelief(state_, belief_type_);

		end_t = get_time_second();
		logi << "[POMDPEvaluator::InitRound] Created intial belief "
			<< typeid(*belief).name() << " in " << (end_t - start_t) << "s" << endl;

		solver_->belief(belief);
	}

	total_discounted_reward_ = 0;
	total_undiscounted_reward_ = 0;
}

double POMDPEvaluator::EndRound() {
	if (!Globals::config.silence && out_) {
		*out_ << "Total discounted reward = " << total_discounted_reward_ << endl
			<< "Total undiscounted reward = " << total_undiscounted_reward_ << endl;
	}

	discounted_round_rewards_.push_back(total_discounted_reward_);
	undiscounted_round_rewards_.push_back(total_undiscounted_reward_);

	return total_undiscounted_reward_;
}

bool POMDPEvaluator::ExecuteAction(int action, double& reward, OBS_TYPE& obs) 
{
	double random_num = random_.NextDouble();

	// if this is the first action treat current state as observation
	bool terminal;
	if (OnlineSolverModel::IsExternalSimulator())
	{
		// send action to simulator
		static_cast<OnlineSolverModel *>(model_)->SendAction(action);
		// read state and update reward and observation
		terminal = static_cast<OnlineSolverModel *>(model_)->RcvState(state_, reward, obs);
	}
	else
		terminal = model_->Step(*state_, random_num, action, reward, obs);

	reward_ = reward;
	total_discounted_reward_ += Globals::Discount(step_) * reward;
	total_undiscounted_reward_ += reward;

	return terminal;
}

double POMDPEvaluator::End() {
	return 0; // Not to be used
}

void POMDPEvaluator::UpdateTimePerMove(double step_time) {
	if (target_finish_time_ != -1) {
		if (step_time < 0.99 * EvalLog::allocated_time) {
			if (EvalLog::plan_time_ratio < 1.0)
				EvalLog::plan_time_ratio += 0.01;
			if (EvalLog::plan_time_ratio > 1.0)
				EvalLog::plan_time_ratio = 1.0;
		} else if (step_time > EvalLog::allocated_time) {
			double delta = (step_time - EvalLog::allocated_time)
				/ (EvalLog::allocated_time + 1E-6);
			if (delta < 0.02)
				delta = 0.02; // Minimum reduction per step
			if (delta > 0.05)
				delta = 0.05; // Maximum reduction per step
			EvalLog::plan_time_ratio -= delta;
			// if (EvalLog::plan_time_ratio < 0)
			// EvalLog::plan_time_ratio = 0;
		}

		EvalLog::curr_inst_remaining_budget = target_finish_time_
			- get_time_second();
		EvalLog::curr_inst_remaining_steps--;

		if (EvalLog::curr_inst_remaining_steps <= 0) {
			EvalLog::allocated_time = 0;
		} else {
			EvalLog::allocated_time =
				(EvalLog::curr_inst_remaining_budget - 2.0)
					/ EvalLog::curr_inst_remaining_steps;

			if (EvalLog::allocated_time > 5.0)
				EvalLog::allocated_time = 5.0;
		}

		Globals::config.time_per_move = EvalLog::plan_time_ratio
			* EvalLog::allocated_time;
	}
}


} // namespace despot
