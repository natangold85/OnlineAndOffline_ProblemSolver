#ifndef SOLVER_H
#define SOLVER_H

#include "../core/globals.h"
#include "../core/history.h"

#include "..\..\..\src\ThreadDataClass.h"
#include "..\..\..\src\Tree_Properties.h"

namespace despot {

class DSPOMDP;
class Belief;
struct ValuedAction;


/* =============================================================================
 * SearchStatistics class
 * =============================================================================*/

struct SearchStatistics {
	double initial_lb, initial_ub, final_lb, final_ub;
	double time_search;
	double time_path;
	double time_backup;
	double time_node_expansion;
	int num_policy_nodes;
	int num_tree_nodes;
	int num_expanded_nodes;
	int num_tree_particles;
	int num_particles_before_search;
	int num_particles_after_search;
	int num_trials;
	int longest_trial_length;

	SearchStatistics();

	friend std::ostream& operator<<(std::ostream& os, const SearchStatistics& statitics);
};

class SolverBase
{
public:
	virtual void Search(TreeDevelopThread * threadData, int action)  = 0;
	virtual ValuedAction Search() = 0;

	virtual void Update(int action, OBS_TYPE obs) = 0;
	virtual void UpdateHistory(int action, OBS_TYPE obs) = 0;

	virtual void GetTreeProperties(Tree_Properties & treeProp) const = 0;

	virtual void belief(Belief* b) = 0;
	virtual Belief* belief() = 0;
	virtual void DeleteBelief() = 0;
};
/* =============================================================================
 * Solver class
 * =============================================================================*/

class Solver: public SolverBase 
{
protected:
	const DSPOMDP* model_;
	Belief* belief_;
	History history_;

public:
	Solver(const DSPOMDP* model, Belief* belief);
	virtual ~Solver();

	/**
	 * Find the optimal action for current belief, and optionally return the
	 * found value for the action. Return the value Globals::NEG_INFTY if the
	 * value is not to be used.
	 */
	virtual ValuedAction Search() = 0;
	
	virtual void Search(TreeDevelopThread * threadData, int action) {};
	virtual void GetTreeProperties(Tree_Properties & treeProp) const {};
	virtual void GetSingleActionTreeProp(SingleNodeTreeProp & treeProp, int action) const {};

	/**
	 * Update current belief, history, and any other internal states that is
	 * needed for Search() to function correctly.
	 */
	virtual void Update(int action, OBS_TYPE obs);
	virtual void UpdateHistory(int action, OBS_TYPE obs);
	/**
	 * Set initial belief for planning. Make sure internal states associated with
	 * initial belief are reset. In particular, history need to be cleaned, and
	 * allocated memory from previous searches need to be cleaned if not.
	 */
	virtual void belief(Belief* b);
	Belief* belief();

	virtual void DeleteBelief();

};

// in order for solver to run parallel it has to have copy ctor
class ParallelSolver : public SolverBase
{
public:
	ParallelSolver(std::vector<Solver *> solver, int numActions);
	
	ParallelSolver(const ParallelSolver & solv) = delete;
	ParallelSolver & operator=(const ParallelSolver & solv) = delete;

	void ThreadsMngrFunction();
	void TreeThreadsMainFunction(int action);

	virtual void GetTreeProperties(Tree_Properties & treeProp) const override;

	virtual ValuedAction Search() override;
	
	virtual void Update(int action, OBS_TYPE obs) override;
	virtual void UpdateHistory(int action, OBS_TYPE obs);

	virtual void belief(Belief* b) override;
	virtual void belief(Belief* b, int idxSolver);
	virtual Belief* belief() override;
	virtual void DeleteBelief() override;

	virtual void Search(TreeDevelopThread * threadData, int action) {};

	TreeMngrThread & GetTreeMngrData() { return mngrData_; };
	

	int NumSolvers() const {return solvers_.size(); };

private:
	void StartRoundMngr();
	void EndRoundMngr();


	ValuedAction FindPrefferedAction();

	TreeMngrThread mngrData_;

	std::vector<Solver *> solvers_;
	std::vector<TreeDevelopThread> threadsData_;
	std::vector<std::thread> threads_;

	std::mutex barrierMutex_;
	int barrierCounter_;
};

} // namespace despot

#endif
