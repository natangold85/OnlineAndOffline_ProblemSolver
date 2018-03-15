#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <string>
#include <memory>

#include <UDP_Prot.h>

static const int DESPOT_PORT = 5678;

static const bool saveEachTrialInSingleFile = true;
static const char *TREE_FILE_NAME = "tree";

// for tree sending
static int s_END_OF_SEND_TREE = -1;
static int s_START_NEW_TRIAL = 1;
static int s_START_NEW_STEP = 2;
static int s_END_RUNNING = 3;
static int s_ERROR_READING = 4;

static int s_numActions = -1;
static const int s_numValsInNode = 4;

static const int s_parentIdx = 0;
static const int s_idIdx = 1;
static const int s_valueIdx = 2;
static const int s_countIdx = 3;

class TreeNode
{
public:
	using TreeNodePtr = std::shared_ptr<TreeNode>;
	TreeNode(TreeNodePtr parent, float value, unsigned int count, unsigned int id)
	: m_parent(parent), m_value(value), m_count(count), m_id(id), m_children()
	{}

	~TreeNode() = default;


	void AddChild(TreeNodePtr child) { m_children.emplace_back(child); };
	/// preform function on root only
	void SortActionsChilds() 
	{
		// get curr node shared ptr
		TreeNodePtr currNodePtr(GetNodePtr());
		if (currNodePtr == nullptr)
			return;

		std::vector<int> childActionsId(s_numActions);
		for (int i = 0; i < s_numActions; ++i)
			childActionsId[i] = i + m_id + 1;

		std::vector<TreeNodePtr> sorted;
		for (int a = 0; a < s_numActions; ++a)
		{
			TreeNodePtr node = nullptr;
			for (int n = 0; n < m_children.size(); ++n)
			{
				if (childActionsId[a] == m_children[n]->m_id)
				{
					node = m_children[n];
					m_children.erase(m_children.begin() + n);
					break;
				}
			}
			if (node == nullptr)
			{
				TreeNodePtr newNode(new TreeNode(currNodePtr, 0, 0, childActionsId[a]));
				node = newNode;
			}

			sorted.emplace_back(node);
		}

		m_children = sorted;
		// sort garndsons
		for (auto child : m_children)
		{
			for (auto grandsons : child->m_children)
				grandsons->SortActionsChilds();
		}
	}

	int Depth() const
	{
		if (m_parent == nullptr)
			return 0;

		return m_parent->Depth() + 1;
	}

	TreeNodePtr GetNodePtr()
	{
		if (m_children.size() == 0)
			return nullptr;

		return m_children[0]->m_parent;
	}
 
	TreeNodePtr m_parent;
	float m_value;
	unsigned int m_count;
	unsigned int m_id;
	std::vector<TreeNodePtr> m_children;
};

int ReadMSg(UDP_Client * sock, std::vector<unsigned int> * treeVec, std::vector<unsigned int> * state);
TreeNode::TreeNodePtr BuildTree(std::vector<unsigned int> & treeVec);
TreeNode * SearchId(TreeNode *root, unsigned int id);
void SaveTree(std::vector<unsigned int> & state, TreeNode::TreeNodePtr root, int step, std::ofstream & out);
std::ofstream & operator<<(std::ofstream & out, TreeNode::TreeNodePtr node);
void PrintTree(TreeNode::TreeNodePtr root);

int main()
{
	UDP_Client sock;
	if (!sock.Init(DESPOT_PORT))
	{
		std::cout << "error in udp init";
		Sleep(1000);
		exit(0);
	}

	sock.TimeOut(500);
	std::ofstream out;
	char c = 1;
	sock.Write(&c, 1);
	Sleep(1000);

	if (saveEachTrialInSingleFile)
	{
		std::string fName = TREE_FILE_NAME;
		fName += ".txt";
		out.open(fName, std::ios::out);
	}
	std::vector<unsigned int> firstRead(2);
	
	unsigned int Readstatus = ReadMSg(&sock, nullptr, &firstRead);
	s_numActions = firstRead[0];
	int numObjects = firstRead[1];

	bool isRunning = Readstatus == s_START_NEW_TRIAL;
	int trial = 0;
	// run on all trials
	std::cout << "reading msg...\n";
	while (isRunning)
	{
		if (saveEachTrialInSingleFile)
		{
			out.close();
			std::string treeFName(TREE_FILE_NAME);
			treeFName += "_trial" + std::to_string(trial) + ".txt";
			out.open(treeFName, std::ios::out);
		}
		out << "\n\n\ntrial num " << trial << ":\n\n\n";
		// run on all steps
		int step = 0;
		while (1)
		{
			std::vector<unsigned int> treeVec;
			std::vector<unsigned int> state(numObjects);

			int status = ReadMSg(&sock, &treeVec, &state);
			if (status == s_END_RUNNING)
			{
				isRunning = false;
				break;
			}
			else if (status == s_START_NEW_TRIAL)		
			{
				s_numActions = state[0];
				numObjects = state[1];
				break;
			}
			else if (status == s_ERROR_READING)
				continue;

			std::cout << "finished reading msg\n";
			TreeNode::TreeNodePtr root = BuildTree(treeVec);
			std::cout << "finished parsing tree\n";
			SaveTree(state, root, step, out);
			std::cout << "save tree in file\n";
			out.flush();
			++step;
			std::cout << "reading msg...\n";
		}
		++trial;
	}

	return 0;
}

int ReadMSg(UDP_Client * sock, std::vector<unsigned int> * treeVec, std::vector<unsigned int> * state)
{
	// TODO : make this function more compatible if we watn to keep saving trees in file
	static char buf[UDP_Server::s_BUF_LEN];

	unsigned int * ptr;
	int maxToRead = UDP_Server::s_BUF_LEN + UDP_Server::s_BUF_LEN % sizeof (int);
	int size = maxToRead;
	
	int startOfMsg;
	int status;
	bool readStatus = false;
	bool readState = false;

	while (size == maxToRead)
	{
		size = sock->Read(buf);
		if (size == -1)
			return s_ERROR_READING;

		int toInit = size / sizeof(int);
		ptr = reinterpret_cast<unsigned int *>(buf);

		// search for end of msg in middle message if fount return error in read
		int *currPtr = reinterpret_cast<int *>(buf);
		for (int i = 0; i < toInit - 1; ++i)
		{
			if (currPtr[i] == s_END_OF_SEND_TREE)
				return s_ERROR_READING;
		}

		if (!readStatus)
		{
			status = *reinterpret_cast<int *>(ptr);
			
			if (status != s_START_NEW_STEP)
			{
				if (status != s_END_RUNNING)
				{
					int endOfTrialMsg = reinterpret_cast<int *>(ptr)[3];
					if (endOfTrialMsg != s_END_OF_SEND_TREE)
						return s_ERROR_READING;

					(*state)[0] = ptr[1];
					(*state)[1] = ptr[2];
				}
				return status;
			}
			readStatus = true;
			++ptr;
			--toInit;
		}

		if (!readState)
		{
			for (int i = 0; i < state->size(); ++i)
			{
				(*state)[i] = ptr[i];
			}
			readState = true;
			ptr += state->size();
			toInit -= state->size();
		}

		for (int i = 0; i < toInit; ++i)
			treeVec->emplace_back(ptr[i]);
	}

	int endMsg = (*treeVec)[treeVec->size() - 1];
	treeVec->pop_back();
	if (endMsg != s_END_OF_SEND_TREE)
		status = s_ERROR_READING;

	return status;
}


TreeNode::TreeNodePtr BuildTree(std::vector<unsigned int> & treeVec)
{
	
	// read nodes
	std::vector<std::pair<TreeNode::TreeNodePtr, int>> allNodes;

	int toRun = treeVec.size() - treeVec.size() % sizeof(int);
	for (int i = 0; i < toRun; i += s_numValsInNode)
	{
		unsigned int parentId = treeVec[i + s_parentIdx];
		unsigned int count = treeVec[i + s_countIdx];
		unsigned int id = treeVec[i + s_idIdx];
		float value = *reinterpret_cast<float *>(&treeVec[i + s_valueIdx]);

		TreeNode::TreeNodePtr node(new TreeNode(nullptr, value, count, id));
		allNodes.emplace_back(node, parentId);
	}

	TreeNode::TreeNodePtr root = nullptr;

	// create heirarchi
	for (int parent = 0; parent < allNodes.size(); ++parent)
	{
		for (int child = 0; child < allNodes.size(); ++child)
		{
			// init parents & children
			if (allNodes[child].second == allNodes[parent].first->m_id)
			{
				allNodes[child].first->m_parent = allNodes[parent].first;
				allNodes[parent].first->AddChild(allNodes[child].first);
			}
		}

		if (allNodes[parent].second == 0)
			root = allNodes[parent].first;
	}

	root->SortActionsChilds();

	return root;
}

void SaveTree(std::vector<unsigned int> & state, TreeNode::TreeNodePtr root, int step, std::ofstream & out)
{
	out << "decision for step = " << step << " state = ";
	for (auto loc : state)
		out << loc << ", ";
	
	out << ":\n\n";
	out << root;
	out << "\n\n\n";
}

std::ofstream & operator<<(std::ofstream & out, TreeNode::TreeNodePtr node)
{
	std::string prefix("");
	int depth = node->Depth();
	for (int i = 0; i < depth; ++i)
		prefix += "\t";

	out << prefix << "id = " << node->m_id << " value = " << node->m_value << " count = " << node->m_count << "\n";

	for (auto node : node->m_children)
		out << node;

	return out;
}
