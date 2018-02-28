#ifndef TREE_PROPERTIES_H
#define TREE_PROPERTIES_H

#include <vector>

class Tree_Properties // NATAN CHANGES
{
public:
	Tree_Properties() : m_height(0), m_size(0), m_levelSize(), m_levelActionSize(), m_preferredActionPortion() {};

	void UpdateCount();
	static void ZeroCount();
	void Avg();

	Tree_Properties & operator += (const Tree_Properties &p2);

	double m_height;
	double m_size;
	std::vector<double> m_levelSize;
	std::vector<std::vector<double>> m_levelActionSize;
	std::vector<double> m_preferredActionPortion;

	static std::vector<int> s_levelCounter;
};

# endif //TREE_PROPERTIES_H