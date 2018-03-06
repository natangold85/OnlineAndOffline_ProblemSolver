#include <fstream>      // std::ofstream NATAN CHANGES

#include "../include/despot/simple_tui.h"

#include "nxnGrid.h"

using namespace std;


namespace despot {

void disableBufferedIO(void) {
	
 //### setbuf(stdout, NULL);
 //### setbuf(stdin, NULL);
 //### setbuf(stderr, NULL);
  setvbuf(stdout, nullptr, _IONBF, 0);
  setvbuf(stdin, nullptr, _IONBF, 0);
  setvbuf(stderr, nullptr, _IONBF, 0);
}

void TreeBuilderThread(ThreadDataStruct * data)
{
	Evaluator * evaluator = reinterpret_cast<Evaluator *>(data->m_evaluator);

	 for (int i = 0; i < INT_MAX; i++) 
	 {
		evaluator->RunStep(data, i);
		{ // inform to builder that round is over
			std::lock_guard<std::mutex> lock(data->m_flagsMutex);
			if (data->m_terminal)
				break;
		}
	 }
}

SimpleTUI::SimpleTUI() {}

SimpleTUI::~SimpleTUI() {}

Solver *SimpleTUI::InitializeSolver(DSPOMDP *model, string solver_type,
                                    option::Option *options) {
  Solver *solver = NULL;
  // DESPOT or its default policy
  if (solver_type == "DESPOT" ||
      solver_type == "PLB") // PLB: particle lower bound
  {
    string blbtype = options[E_BLBTYPE] ? options[E_BLBTYPE].arg : "DEFAULT";
    string lbtype = options[E_LBTYPE] ? options[E_LBTYPE].arg : "DEFAULT";
    ScenarioLowerBound *lower_bound =
        model->CreateScenarioLowerBound(lbtype, blbtype);

    logi << "Created lower bound " << typeid(*lower_bound).name() << endl;

    if (solver_type == "DESPOT") {
      string bubtype = options[E_BUBTYPE] ? options[E_BUBTYPE].arg : "DEFAULT";
      string ubtype = options[E_UBTYPE] ? options[E_UBTYPE].arg : "DEFAULT";
      ScenarioUpperBound *upper_bound =
          model->CreateScenarioUpperBound(ubtype, bubtype);

      logi << "Created upper bound " << typeid(*upper_bound).name() << endl;

      solver = new DESPOT(model, lower_bound, upper_bound);
    } else
      solver = lower_bound;
  } // AEMS or its default policy
  else if (solver_type == "AEMS" || solver_type == "BLB") {
    string lbtype = options[E_LBTYPE] ? options[E_LBTYPE].arg : "DEFAULT";
    BeliefLowerBound *lower_bound =
        static_cast<BeliefMDP *>(model)->CreateBeliefLowerBound(lbtype);

    logi << "Created lower bound " << typeid(*lower_bound).name() << endl;

    if (solver_type == "AEMS") {
      string ubtype = options[E_UBTYPE] ? options[E_UBTYPE].arg : "DEFAULT";
      BeliefUpperBound *upper_bound =
          static_cast<BeliefMDP *>(model)->CreateBeliefUpperBound(ubtype);

      logi << "Created upper bound " << typeid(*upper_bound).name() << endl;

      solver = new AEMS(model, lower_bound, upper_bound);
    } else
      solver = lower_bound;
  } // POMCP or DPOMCP
  else if (solver_type == "POMCP" || solver_type == "DPOMCP") {
    string ptype = options[E_PRIOR] ? options[E_PRIOR].arg : "DEFAULT";
    POMCPPrior *prior = model->CreatePOMCPPrior(ptype);

    logi << "Created POMCP prior " << typeid(*prior).name() << endl;

    if (options[E_PRUNE]) {
      prior->exploration_constant(Globals::config.pruning_constant);
    }

	if (solver_type == "POMCP")
	{
		solver = new POMCP(model, prior);
		((POMCP *)solver)->reuse(true);
	}
    else
      solver = new DPOMCP(model, prior);
  } else { // Unsupported solver
    cerr << "ERROR: Unsupported solver type: " << solver_type << endl;
    exit(1);
  }
  return solver;
}

void SimpleTUI::OptionParse(option::Option *options, int &num_runs,
                            string &simulator_type, string &belief_type,
                            int &time_limit, string &solver_type,
                            bool &search_solver) {
  if (options[E_SILENCE])
    Globals::config.silence = true;

  if (options[E_DEPTH])
    Globals::config.search_depth = atoi(options[E_DEPTH].arg);

  if (options[E_DISCOUNT])
    Globals::config.discount = atof(options[E_DISCOUNT].arg);

  if (options[E_SEED])
    Globals::config.root_seed = atoi(options[E_SEED].arg);
  else { // last 9 digits of current time in milli second
    long millis = (long)get_time_second() * 1000;
    long range = (long)pow((double)10, (int)9);
    Globals::config.root_seed =
        (unsigned int)(millis - (millis / range) * range);
  }

  if (options[E_TIMEOUT])
    Globals::config.time_per_move = atof(options[E_TIMEOUT].arg);

  if (options[E_NUMPARTICLES])
    Globals::config.num_scenarios = atoi(options[E_NUMPARTICLES].arg);

  if (options[E_PRUNE])
    Globals::config.pruning_constant = atof(options[E_PRUNE].arg);

  if (options[E_GAP])
    Globals::config.xi = atof(options[E_GAP].arg);

  if (options[E_SIM_LEN])
    Globals::config.sim_len = atoi(options[E_SIM_LEN].arg);

  if (options[E_EVALUATOR])
    simulator_type = options[E_EVALUATOR].arg;

  if (options[E_MAX_POLICY_SIM_LEN])
    Globals::config.max_policy_sim_len =
        atoi(options[E_MAX_POLICY_SIM_LEN].arg);

  if (options[E_DEFAULT_ACTION])
    Globals::config.default_action = options[E_DEFAULT_ACTION].arg;

  if (options[E_RUNS])
    num_runs = atoi(options[E_RUNS].arg);

  if (options[E_BELIEF])
    belief_type = options[E_BELIEF].arg;

  if (options[E_TIME_LIMIT])
    time_limit = atoi(options[E_TIME_LIMIT].arg);

  if (options[E_NOISE])
    Globals::config.noise = atof(options[E_NOISE].arg);

  search_solver = options[E_SEARCH_SOLVER];

  if (options[E_SOLVER])
    solver_type = options[E_SOLVER].arg;

  int verbosity = 0;
  if (options[E_VERBOSITY])
    verbosity = atoi(options[E_VERBOSITY].arg);
  logging::level(verbosity);
}

void SimpleTUI::InitializeEvaluator(Evaluator *&simulator,
                                    option::Option *options, DSPOMDP *model,
                                    Solver *solver, int num_runs,
                                    clock_t main_clock_start,
                                    string simulator_type, string belief_type,
                                    int time_limit, string solver_type) {

  if (time_limit != -1) {
    simulator =
        new POMDPEvaluator(model, belief_type, solver, main_clock_start, &cout,
                           EvalLog::curr_inst_start_time + time_limit,
                           num_runs * Globals::config.sim_len);
  } else {
    simulator =
        new POMDPEvaluator(model, belief_type, solver, main_clock_start, &cout);
  }
}

void SimpleTUI::DisplayParameters(option::Option *options, DSPOMDP *model, std::ofstream & out) 
{
	static int counter = 0;
	if (counter > 0)
		return;
	counter = 1;
	string lbtype = options[E_LBTYPE] ? options[E_LBTYPE].arg : "DEFAULT";
	string ubtype = options[E_UBTYPE] ? options[E_UBTYPE].arg : "DEFAULT";
	out << "Model = " << typeid(*model).name() << endl
				<< "Random root seed = " << Globals::config.root_seed << endl
				<< "Search depth = " << Globals::config.search_depth << endl
				<< "Discount = " << Globals::config.discount << endl
				<< "Simulation steps = " << Globals::config.sim_len << endl
				<< "Number of scenarios = " << Globals::config.num_scenarios
				<< endl
				<< "Search time per step = " << Globals::config.time_per_move
				<< endl
				<< "Regularization constant = "
				<< Globals::config.pruning_constant << endl
				<< "Lower bound = " << lbtype << endl
				<< "Upper bound = " << ubtype << endl
				<< "Policy simulation depth = "
				<< Globals::config.max_policy_sim_len << endl
				<< "Target gap ratio = " << Globals::config.xi << endl;

	model->DisplayParameters(out);
	// << "Solver = " << typeid(*solver).name() << endl << endl;
}

void SimpleTUI::RunEvaluator(DSPOMDP *model, Evaluator *simulator,
                             option::Option *options, int num_runs,
                             bool search_solver, Solver *&solver,
                             string simulator_type, clock_t main_clock_start,
                             int start_run) {
  // Run num_runs simulations
  vector<double> round_rewards(num_runs);
  for (int round = start_run; round < start_run + num_runs; round++) 
  {
	  nxnGrid *m = static_cast<nxnGrid *>(model); //NATAN CHANGES (random initial condition)
	  m->InitState();
    default_out << endl
                << "####################################### Round " << round
                << " #######################################" << endl;

    if (search_solver) {
      if (round == 0) {
        solver = InitializeSolver(model, "DESPOT", options);
        default_out << "Solver: " << typeid(*solver).name() << endl;

        simulator->solver(solver);
      } else if (round == 5) {
        solver = InitializeSolver(model, "POMCP", options);
        default_out << "Solver: " << typeid(*solver).name() << endl;

        simulator->solver(solver);
      } else if (round == 10) {
        double sum1 = 0, sum2 = 0;
        for (int i = 0; i < 5; i++)
          sum1 += round_rewards[i];
        for (int i = 5; i < 10; i++)
          sum2 += round_rewards[i];
        if (sum1 < sum2)
          solver = InitializeSolver(model, "POMCP", options);
        else
          solver = InitializeSolver(model, "DESPOT", options);
        default_out << "Solver: " << typeid(*solver).name()
                    << " DESPOT:" << sum1 << " POMCP:" << sum2 << endl;
      }

      simulator->solver(solver);
    }

    simulator->InitRound();
	int i = 0;

	if (nxnGrid::ParallelRun())
		RunRound(simulator, model, round);
	else
	{	// original implementation
		for (; i < Globals::config.sim_len; i++) {
			/*
			default_out << "-----------------------------------Round " << round
			<< " Step " << i << "-----------------------------------"
			<< endl;*/
			double step_start_t = get_time_second();



			bool terminal = simulator->RunStep(i, round);

			if (terminal)
				break;

			double step_end_t = get_time_second();
			logi << "[main] Time for step: actual / allocated = "
				<< (step_end_t - step_start_t) << " / " << EvalLog::allocated_time
				<< endl;
			simulator->UpdateTimePerMove(step_end_t - step_start_t);
			logi << "[main] Time per move set to " << Globals::config.time_per_move
				<< endl;
			logi << "[main] Plan time ratio set to " << EvalLog::plan_time_ratio
				<< endl;
			//  default_out << endl;
		}
	}

    default_out << "Simulation terminated in " << simulator->step() << " steps"
                << endl;

    double round_reward = simulator->EndRound();

	number_steps_termination_.emplace_back(i);
	
	// print avg reward
	default_out << "Average Reward = " << simulator->AverageUndiscountedRoundReward() << "\n";
  }

  if (simulator_type == "ippc" && num_runs != 30) {
    cout << "Exit without receiving reward." << endl
         << "Total time: Real / CPU = "
         << (get_time_second() - EvalLog::curr_inst_start_time) << " / "
         << (double(clock() - main_clock_start) / CLOCKS_PER_SEC) << "s"
         << endl;
    exit(0);
  }
}

void SimpleTUI::RunRound(Evaluator * evaluator, DSPOMDP * model, int round)
{
	ThreadDataStruct dataForThread;
	
	dataForThread.m_evaluator = evaluator;
	dataForThread.m_action = -1;
	dataForThread.m_lastObservation = -1;
	dataForThread.m_actionNeeded = false;
	dataForThread.m_terminal = false;
	dataForThread.m_observationRecieved = false;

	std::thread treeBuilder(TreeBuilderThread, &dataForThread);
	
	std::vector<Tree_Properties> actionsTreeData(model->NumActions());
	bool terminal = false;
	
	for (int step = 0; step < Globals::config.sim_len && !terminal; ++step)
	{
		double stepStart = get_time_second();
		Sleep(Globals::config.time_per_move * 1000);

		double actionWanted = get_time_second();

		// inform to builder that action is needed
		{ 
			std::lock_guard<std::mutex> lock(dataForThread.m_flagsMutex);
			dataForThread.m_actionNeeded = true;
		}

		// wait for action is ready
		bool actionRecieved = false;
		while (!actionRecieved)
		{
			std::lock_guard<std::mutex> lock(dataForThread.m_flagsMutex);
			actionRecieved = dataForThread.m_actionRecieved;
		}

		double actionWantedMsgDelievered = get_time_second();
		
		int action = -1;
		double reward;
		OBS_TYPE obs;
		State currState;

		{ // take action and update observation
			std::lock_guard<std::mutex> mainLock(dataForThread.m_mainMutex);
			action = dataForThread.m_action;
			
			nxnGrid::UpdateRealAction(action);
			
			terminal = evaluator->ExecuteAction(action, reward, obs);
			currState.state_id = evaluator->currStateId();

			evaluator->GetTreeProperties(actionsTreeData);

			dataForThread.m_lastObservation = obs;
			std::lock_guard<std::mutex> flagsLock(dataForThread.m_flagsMutex);
			dataForThread.m_observationRecieved = true;
			dataForThread.m_actionRecieved = false;
		}
		
		double actionCommited = get_time_second();

		// print data
		std::cout << "- Tree Properties: (size,count,value)\n";
		for (auto actionProp : actionsTreeData)
		{
			std::cout << actionProp.m_size << ", " << actionProp.m_nodeCount << ", " << actionProp.m_nodeValue << "\n";
		}

		std::cout << "- Action = ";
		model->PrintAction(action);
		std::cout << "- Current State: : ";
		model->PrintState(currState);
		std::cout << "- Observation: : ";
		model->PrintObs(currState, obs);
		std::cout << "- ObsProb = " << model->ObsProb(obs, currState, action) << "\n";

		std::cout << "\n- StepReward = " << reward << "\n\n\n";
	}

	{ // inform to builder that round is over
		std::lock_guard<std::mutex> lock(dataForThread.m_flagsMutex);
		dataForThread.m_terminal = true;
	}

	treeBuilder.join();
}


void SimpleTUI::PrintResult(int num_runs, Evaluator *simulator,
                            clock_t main_clock_start, std::string & result) {

	result += "\nCompleted " + std::to_string(num_runs) + " run(s).\n";
	
	// insert final results
	simulator->PrintFinalResults(result);

	// insert num steps to termination to buffer
	result += "num steps for each run:\n";
	for (auto v : number_steps_termination_)
		result += std::to_string(v) + ", ";
	result += "\n\n";
	
	// insert undiscounted reward to to buffer
	result += "\nAverage total undiscounted reward (stderr) = "
		+ std::to_string(simulator->AverageUndiscountedRoundReward()) + " ("
		+ std::to_string(simulator->StderrUndiscountedRoundReward()) + ")" + "\n\n";
	simulator->AddUnDiscounted2String(result);

	// insert discounted reward to buffer
	result += "\nAverage total discounted reward (stderr) = "
		+ std::to_string(simulator->AverageDiscountedRoundReward()) + " ("
		+ std::to_string(simulator->StderrDiscountedRoundReward()) + ")" + "\n\n";
	simulator->AddDiscounted2String(result);

	// insert time and tree properties to buffer
	result += "Total time: Real / CPU = "
		+ std::to_string(get_time_second() - EvalLog::curr_inst_start_time) + " / "
		+ std::to_string(double(clock() - main_clock_start) / CLOCKS_PER_SEC) + "s\n\n";

	simulator->PrintTreeProp(result);
}

int SimpleTUI::run(int argc, char *argv[], std::ofstream & result) {

	clock_t main_clock_start = clock();
	EvalLog::curr_inst_start_time = get_time_second();

	const char *program = (argc > 0) ? argv[0] : "despot";

	argc -= (argc > 0);
	argv += (argc > 0); // skip program name argv[0] if present

	option::Stats stats(usage, argc, argv);
	option::Option *options = new option::Option[stats.options_max];
	option::Option *buffer = new option::Option[stats.buffer_max];
	option::Parser parse(usage, argc, argv, options, buffer);

	string solver_type = "POMCP";
	bool search_solver;

	/* =========================
	* Parse required parameters
	* =========================*/
	int num_runs = 100; // NATAN CHANGES SOLVER
	string simulator_type = "pomdp";
	string belief_type = "DEFAULT";
	int time_limit = -1;

	/* =========================================
	* Problem specific default parameter values
	*=========================================*/
	InitializeDefaultParameters();

	/* =========================
	* Parse optional parameters
	* =========================*/
	if (options[E_HELP]) {
	cout << "Usage: " << program << " [options]" << endl;
	option::printUsage(std::cout, usage);
	return 0;
	}
	OptionParse(options, num_runs, simulator_type, belief_type, time_limit,
				solver_type, search_solver);

	/* =========================
	* Global random generator
	* =========================*/
	Seeds::root_seed(Globals::config.root_seed);
	unsigned world_seed = Seeds::Next();
	unsigned seed = Seeds::Next();
	Random::RANDOM = Random(seed);

	/* =========================
	* initialize model
	* =========================*/
	DSPOMDP *model = InitializeModel(options);


	/* =========================
	* initialize solver
	* =========================*/
	Solver *solver = InitializeSolver(model, solver_type, options);

	/* =========================
	* Print parameters
	* =========================*/
	DisplayParameters(options, model, result);
	assert(solver != NULL);

	/* =========================
	* initialize simulator
	* =========================*/
	Evaluator *simulator = NULL;
	InitializeEvaluator(simulator, options, model, solver, num_runs,
						main_clock_start, simulator_type, belief_type, time_limit,
						solver_type);

	simulator->world_seed(world_seed);

	int start_run = 0;


	/* =========================
	* run simulator
	* =========================*/
	RunEvaluator(model, simulator, options, num_runs, search_solver, solver,
				simulator_type, main_clock_start, start_run);

	simulator->End();
	if (nxnGrid::ToSendTree())
		nxnGrid::SendEndRun();

	std::string resultString;
	PrintResult(num_runs, simulator, main_clock_start, resultString);
  
	// NATAN CHANGES
	result << resultString;
	if (result.bad())
	{
		std::cerr << "failed write result to file\nresults:\n";
		std::cerr << resultString;
		exit(1);
	}
  return 0;
}

} // namespace despot
