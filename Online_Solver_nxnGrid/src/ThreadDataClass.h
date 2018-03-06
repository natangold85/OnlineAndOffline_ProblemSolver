#ifndef THREAD_DATA_CLASS_H
#define THREAD_DATA_CLASS_H

#include <thread>
#include <mutex>

namespace despot
{


struct ThreadDataStruct
{
	// is void because of inner dependencies in algorithm of solvers
	void * m_evaluator;

	std::mutex m_mainMutex;
	std::mutex m_flagsMutex;

	bool m_actionNeeded;
	bool m_observationRecieved;
	bool m_actionRecieved;
	bool m_terminal;

	OBS_TYPE m_lastObservation;
	int m_action;
};

struct TreeThreadDataStruct
{
	// is void because of inner dependencies in algorithm of solvers
	void * actionChild;
	void * solver;

	std::mutex m_flagsMutex;
	bool m_terminal;
};

} // ns despot

# endif //THREAD_DATA_CLASS_H