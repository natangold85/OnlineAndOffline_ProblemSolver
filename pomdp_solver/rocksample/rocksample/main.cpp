#include "RockSampleSarsop.h"

#include <fstream>
#include <iostream>

static std::string SOLVER_PATH = "C:\\Users\\moshe\\Documents\\GitHub\\pomdp_solver\\src\\Release\\";
static std::string POMDP_FNAME = "rsTest.pomdp";
static std::string POLICY_FNAME = "rsTest.policy";
int main()
{
	RockSampleSarsop::rockVec rock;
	rock.emplace_back(2, std::vector<bool>{ true});
	rock.emplace_back(5, std::vector<bool>{ true});
	rock.emplace_back(7, std::vector<bool>{ false});
	rock.emplace_back(18, std::vector<bool>{ true});
	rock.emplace_back(42, std::vector<bool>{ false});
	rock.emplace_back(35, std::vector<bool>{ true});
	rock.emplace_back(31, std::vector<bool>{ true});

	//rock.emplace_back(1, std::vector<bool>{ true });
	RockSampleSarsop test(7, rock, 1);
	
	std::ofstream out(POMDP_FNAME, std::ios::out);
	test.SaveInFormat(out);
	out.flush();

	std::string solver = SOLVER_PATH + "pomdpsol.exe " + POMDP_FNAME + " --timeout 30000 --precision 0.005 -o " + POLICY_FNAME;
	system(solver.c_str());

	std::cout << "solver finished\n";
	std::cout << "press any key to continue\n";
	char c;
	std::cin >> c;

	std::string simulator = SOLVER_PATH + "pomdpsim.exe --simLen 50 --simNum 50 ";
	simulator += "--policy-file " + POLICY_FNAME + " " + POMDP_FNAME;
	system(simulator.c_str());

	std::cout << "simulator finished\n";
	std::cout << "press any key to exit\n";
	std::cin >> c;

	return 0;
}