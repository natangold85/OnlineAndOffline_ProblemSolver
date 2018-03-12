#ifndef TREE_PROPERTIES_H
#define TREE_PROPERTIES_H

#include <vector>
#include <string>

struct SingleNodeTreeProp
{
	SingleNodeTreeProp() : m_nodeCount(0), m_nodeValue(0), m_size(0) {};

	unsigned int m_nodeCount;
	float m_nodeValue;
	unsigned int m_size;
};


class Tree_Properties // NATAN CHANGES
{
public:
	Tree_Properties() : m_actionsChildren() {};
	Tree_Properties(int numChildren) : m_actionsChildren(numChildren) {};

	std::string text() const
	{
		std::string ret = "";

		ret += "whole tree : " + std::to_string(m_rootTreeProp.m_size) + ", " + std::to_string(m_rootTreeProp.m_nodeCount) + ", " + std::to_string(m_rootTreeProp.m_nodeValue) + "\n\n";
		for (int a = 0; a < m_actionsChildren.size(); ++a)
		{
			ret += "action " + std::to_string(a) + " : " + std::to_string(m_actionsChildren[a].m_size) + ", " + 
				std::to_string(m_actionsChildren[a].m_nodeCount) + ", " + std::to_string(m_actionsChildren[a].m_nodeValue) + "\n";
		}

		return ret;
	};

	/*MEMBERS*/
	SingleNodeTreeProp m_rootTreeProp;
	std::vector<SingleNodeTreeProp> m_actionsChildren;


	//void UpdateCount();
	//static void ZeroCount();
	//void Avg();
	//Tree_Properties & operator += (const Tree_Properties &p2);

	//std::vector<double> m_levelSize;
	//std::vector<std::vector<double>> m_levelActionSize;
	//std::vector<double> m_preferredActionPortion;

	//static std::vector<int> s_levelCounter;
};

# endif //TREE_PROPERTIES_H