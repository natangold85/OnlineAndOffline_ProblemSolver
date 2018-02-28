#include "Tree_Properties.h"

std::vector<int> Tree_Properties::s_levelCounter;

static std::vector<double> &operator+=(std::vector<double> &vec, const std::vector<double> &toAdd)
{
	if (vec.size() < toAdd.size())
		vec.resize(toAdd.size(), 0);

	for (int i = 0; i < toAdd.size(); ++i)
		vec[i] += toAdd[i];

	return vec;
}

static std::vector<double> &operator/=(std::vector<double> &vec, int toDivide)
{
	for (int i = 0; i < vec.size(); ++i)
		vec[i] /= toDivide;

	return vec;
}

static std::vector<std::vector<double>> &operator+=(std::vector<std::vector<double>> &vec, const std::vector<std::vector<double>> &toAdd)
{
	if (vec.size() < toAdd.size())
		vec.resize(toAdd.size());

	for (int i = 0; i < toAdd.size(); ++i)
		vec[i] += toAdd[i];

	return vec;
}

static std::vector<std::vector<double>> &operator/=(std::vector<std::vector<double>> &vec, int toDivide)
{
	for (int i = 0; i < vec.size(); ++i)
		vec[i] /= toDivide;

	return vec;
}

void Tree_Properties::UpdateCount()
{
	if (s_levelCounter.size() < m_levelSize.size())
		s_levelCounter.resize(m_levelSize.size(), 0);

	for (int i = 0; i < m_levelSize.size(); ++i)
		++s_levelCounter[i];
}

void Tree_Properties::ZeroCount()
{
	s_levelCounter.resize(0);
}

void Tree_Properties::Avg()
{
	if (s_levelCounter.size() == 0)
		return;

	int numTrees = s_levelCounter[0];
	m_height /= numTrees;
	m_size /= numTrees;
	m_levelSize /= numTrees;
	m_levelActionSize /= numTrees;

	for (int i = 0; i < m_preferredActionPortion.size(); ++i)
		m_preferredActionPortion[i] /= s_levelCounter[i];
}

Tree_Properties & Tree_Properties::operator+= (const Tree_Properties &other)
{
	m_height += other.m_height;
	m_size += other.m_size;
	m_levelSize += other.m_levelSize;
	m_levelActionSize += other.m_levelActionSize;
	m_preferredActionPortion += other.m_preferredActionPortion;

	return *this;
}

Tree_Properties &operator/=(Tree_Properties &p1, int toDivide) // NATAN CHANGES
{
	p1.m_height /= toDivide;
	p1.m_size /= toDivide;
	p1.m_levelSize /= toDivide;
	p1.m_levelActionSize /= toDivide;
	p1.m_preferredActionPortion /= toDivide;

	return p1;
}