#ifndef PARALLEL_SOLVER_H
#define PARALLEL_SOLVER_H

#include "../core/solver.h"

namespace despot 
{


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

#endif	//PARALLEL_SOLVER_H
