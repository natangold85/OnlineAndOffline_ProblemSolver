#ifndef GLOBALS_H
#define GLOBALS_H

#include <typeinfo>
#include <memory>
#include <set>
#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <list>
#include <algorithm>
#include <ctime>
#include <vector>
#include <queue>
#include <cmath>
#include <cassert>
#include <limits>
#include <sstream>
#include <map>
#include <inttypes.h>
#include "../config.h"

#include "../util/exec_tracker.h"
#include "../util/logging.h"

namespace despot {

//struct stateType
//{
//	int32_t m_counts;
//	int32_t m_armies;
//	int64_t m_queues;
//};
//
//static bool operator>(const stateType &a, const stateType &b)
//{
//	if (a.m_counts != b.m_counts)
//		return a.m_counts > b.m_counts;
//	else if (a.m_armies != b.m_armies)
//		return a.m_armies > b.m_armies;
//	else if (a.m_queues != b.m_queues)
//		return a.m_queues > b.m_queues;
//
//	return false;
//}
//
//static bool operator>=(const stateType &a, const stateType &b)
//{
//	if (a.m_counts != b.m_counts)
//		return a.m_counts > b.m_counts;
//	else if (a.m_armies != b.m_armies)
//		return a.m_armies > b.m_armies;
//	else if (a.m_queues != b.m_queues)
//		return a.m_queues > b.m_queues;
//
//	return true;
//}
//
//static bool operator<(const stateType &a, const stateType &b)
//{
//	return !(b >= a);
//}
//
//static bool operator<=(const stateType &a, const stateType &b)
//{
//	return !(b > a);
//}

typedef int64_t OBS_TYPE;
typedef int64_t STATE_TYPE;

static STATE_TYPE s_DEFAULT_CTOR_STATE;
static OBS_TYPE s_DEFAULT_CTOR_OBS;

namespace Globals {
extern const double NEG_INFTY;
extern const double POS_INFTY;
extern const double INF;
extern const double TINY;

extern Config config;
extern ExecTracker tracker;

inline bool Fequals(double a, double b) {
	return std::fabs(a - b) < TINY;
}

inline double Discount() {
	return config.discount;
}

inline double Discount(int d) {
	return std::pow(config.discount, d);
}

inline void Track(std::string addr, std::string loc) {
	tracker.Track(addr, loc);
}

inline void Untrack(std::string addr) {
	tracker.Untrack(addr);
}

inline void PrintLocs() {
	tracker.PrintLocs();
}
} // namespace

} // namespace despot

#endif
