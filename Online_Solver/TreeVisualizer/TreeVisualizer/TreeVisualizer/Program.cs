using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Net;
using System.Net.Sockets;

namespace TreeVisualizer
{
    class Program
    {
        // parameters:
        static int DESPOT_PORT = 5555;

        static int s_numActions = 3;
        static int s_numValsInNode = 4;

        static int s_parentIdx = 0;
        static int s_idIdx = 1;
        static int s_valueIdx = 2;
        static int s_countIdx = 3;

        static void Main(string[] args)
        {
            var client = new UdpClient();
            IPEndPoint ep = new IPEndPoint(IPAddress.Parse("127.0.0.1"), DESPOT_PORT); 
            client.Connect(ep);

            // send ready signal
            client.Send(new byte[] { 1}, 1);
            
            // read num actions   
            byte []numActions = new byte[1];
            numActions = client.Receive(ref ep);
            TreeNode.NumActions(numActions[0]);

            while (true)
	        {
                // read data
                byte[] receivedData = new byte[10000];
                byte[] buffer = null;
                int size = 0;
                do
                {
                    buffer = client.Receive(ref ep);
                    buffer.CopyTo(receivedData, size);
                    size += buffer.Length;
                }
                while (buffer.Length == 2048);

                // parse tree
                int numNodes = size / (sizeof(uint) * s_numValsInNode);
                TreeNode[] treeNodes = new TreeNode[numNodes];
                uint[] parentIds = new uint[numNodes];

                ParseData(receivedData, numNodes, treeNodes, parentIds);
                TreeNode root = BuildTree(treeNodes, parentIds);
                // visualize tree
                int a = 0;
	        }
        }

        static void ParseData(byte[] data, int numNodes, TreeNode[] treeNodes, uint[] parentId)
        {
            int currVar = 0;
            for (int i = 0; i < numNodes; ++i)
            {
                parentId[i] = BitConverter.ToUInt32(data, currVar);
                currVar += sizeof(uint);
                uint id = BitConverter.ToUInt32(data, currVar);
                currVar += sizeof(uint);
                float value = BitConverter.ToSingle(data, currVar);
                currVar += sizeof(uint);
                uint count = BitConverter.ToUInt32(data, currVar);
                currVar += sizeof(uint);
                treeNodes[i] = new TreeNode(null, id, value, count);
            }
        }

        static TreeNode BuildTree(TreeNode[] treeNodes, uint[] parentId)
        {
            TreeNode root = null;
            // initialize tree
            for (int child = 0; child < treeNodes.Length; ++child)
            {
                for (int parent = 0; parent < treeNodes.Length; ++parent)
                {
                    if (treeNodes[parent].m_id == parentId[child])
                    {
                        treeNodes[child].m_parent = treeNodes[parent];
                        treeNodes[parent].AddChild(treeNodes[child]);
                    }
                }

                if (treeNodes[child].m_parent == null)
                    root = treeNodes[child];
            }

            // sort childs according to id (order matters for actions)
            root.InitIsActionNode()
            root.FillActionChilds();
            root.SortActionChilds();
         
            return root;
        }
    }

    class TreeNode
    {
        public TreeNode m_parent;
        bool m_isActionNode;
        public uint m_id;
        public float m_value;
        public uint m_count;
        public List<TreeNode> m_children;

        static byte s_numActions;

        public TreeNode(TreeNode parent, uint id, float value, uint count)
        {
            m_children = new List<TreeNode>();
            m_parent = parent;
            m_value = value;
            m_count = count;
            m_id = id;
        }

        public static void NumActions(byte numActions)
        {
            s_numActions = numActions;
        }

        public void AddChild(TreeNode child) 
        {
            m_children.Add(child);
        }
	    
        public void InitIsActionNode()
        {
            m_children.ForEach(delegate(TreeNode node)
            {
                node.InitIsActionNode();
            });

            m_isActionNode = (Depth() % 2 == 1);
        }

        public void FillActionChilds()
        {
            // sort grandsons action childs
            m_children.ForEach(delegate(TreeNode node)
            {
                node.FillActionChilds();
            });

            if (m_isActionNode)
            {
                while (m_children.Count != s_numActions)
                {
                    uint nextId = this.m_id + 1;
                    for (int i = 0; i < m_children.Count; ++i, ++nextId)
                    {
                        if (nextId != m_children[i].m_id)
                            break;
                    }
                    m_children.Add(new TreeNode(this, nextId, -100, 0));
                }
            }
        }

        public void SortActionChilds()
        {
            // sort grandsons action childs
            m_children.ForEach(delegate(TreeNode node)
            {
                node.SortActionChilds();
            });

            if (m_isActionNode)
            {
                // sort curr node action children
                m_children.Sort(delegate(TreeNode a, TreeNode b)
                    {
                        return (int)a.m_id - (int)b.m_id;
                    });
            }
        }

        public uint Depth()
        {
            if (m_parent == null)
                return 0;

            return m_parent.Depth() + 1;
        }

        public uint Size()
        {
            if (m_children.Count == 0)
                return 1;

            uint size = 0;

            m_children.ForEach(delegate(TreeNode child)
            {
                size += child.Size();   
            });

            return size + 1;
        }

    }
}
