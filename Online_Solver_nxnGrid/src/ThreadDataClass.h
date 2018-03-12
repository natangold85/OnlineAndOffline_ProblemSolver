#ifndef THREAD_DATA_CLASS_H
#define THREAD_DATA_CLASS_H

#include <thread>
#include <mutex>

namespace despot
{


struct TreeMngrThread
{
	TreeMngrThread()
	: m_action(-1), m_lastObservation(-1), m_actionNeeded(false), m_terminal(false), m_observationRecieved(false), m_actionRecieved(false)
	{}

	TreeMngrThread(const TreeMngrThread &) = delete;
	TreeMngrThread &operator=(const TreeMngrThread &) = delete;

	std::mutex m_mainMutex;
	std::mutex m_flagsMutex;

	bool m_actionNeeded;
	bool m_observationRecieved;
	bool m_actionRecieved;
	bool m_terminal;

	OBS_TYPE m_lastObservation;
	int m_action;
};

struct TreeDevelopThread
{
	TreeDevelopThread()
	: m_toDevelop(false)
	, m_toUpdate(false)
	, m_terminal(false)
	{};

	TreeDevelopThread(const TreeDevelopThread &) = delete;
	TreeDevelopThread &operator=(const TreeDevelopThread &) = delete;

	std::mutex m_treeFlagsMutex;
	
	int m_actionToUpdate;
	OBS_TYPE m_obsToUpdate;
	float m_value;

	bool m_toDevelop;
	bool m_toUpdate;
	bool m_terminal;
};

} // ns despot

# endif //THREAD_DATA_CLASS_H