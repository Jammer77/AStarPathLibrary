using System;
using System.Collections.Generic;

namespace AStarPathLibrary
{
    public sealed partial class AStarPathfinder
    {
        // Heap (simple list but used as a heap, cf. Steve Rabin's game gems article)
        private readonly List<Node> _openList;
        // Closed list is a list.
        private readonly List<Node> _closedList;

        // Successors is a list filled out by the user each type successors to a node are generated
        private readonly List<Node> _successors;
        // State
        private SearchState _searchState = SearchState.NotInitialized;
        // Counts steps
        private int _steps = 0;
        // Start and goal state pointers
        private Node _startNode = null;
        private Node _goalNode = null;
        private Node _currentSolutionNode = null;

        // Memory
        private readonly List<Node> _fixedSizeAllocator;
        private int _allocateNodeCount = 0;
        private bool _cancelRequest = false;
        private int _allocatedMapSearchNodes = 0;
        private readonly List<MapSearchNode> _mapSearchNodePool = null;
        private int _openListHighWaterMark = 0;
        private int _closedListHighWaterMark = 0;
        private int _successorListHighWaterMark = 0;

        // Fixed sizes for collections
        private readonly int _kPreallocatedNodes = 4000;
        private readonly int _kPreallocatedMapSearchNodes = 1000;
        private readonly int _kPreallocatedOpenListSlots = 32;
        private readonly int _kPreallocatedClosedListSlots = 256;
        private readonly int _kPreallocatedSuccessorSlots = 8;


        // constructor just initialises private data
        public AStarPathfinder(Func<int, int, int> getMap)
        {
            // Allocate all lists
            _openList = new List<Node>(_kPreallocatedOpenListSlots);
            _closedList = new List<Node>(_kPreallocatedClosedListSlots);
            _successors = new List<Node>(_kPreallocatedSuccessorSlots);

            _fixedSizeAllocator = new List<Node>(_kPreallocatedNodes);
            for (int i = 0; i < _kPreallocatedNodes; ++i)
            {
                var node = new Node();
                _fixedSizeAllocator.Add(node);
            }

            _mapSearchNodePool = new List<MapSearchNode>(_kPreallocatedMapSearchNodes);
            for (int i = 0; i < _kPreallocatedMapSearchNodes; ++i)
            {
                var mapSearchNode = new MapSearchNode(this, getMap);
                _mapSearchNodePool.Add(mapSearchNode);
            }
        }

        // call at any time to cancel the search and free up all the memory
        public void CancelSearch() => this._cancelRequest = true;

        // Build the open list as sorted to begin with by inserting new elements in the right place
        private void SortedAddToOpenList(Node node)
        {
            bool inserted = false;

            for (int i = 0; i < _openList.Count; ++i)
            {
                if (node.f < _openList[i].f)
                {
                    _openList.Insert(i, node);
                    break;
                }
            }

            if (!inserted)
            {
                _openList.Add(node);
            }

            if (_openList.Count > _openListHighWaterMark)
            {
                _openListHighWaterMark = _openList.Count;
            }
        }

        private Node AllocateNode()
        {
            if (_allocateNodeCount >= _kPreallocatedNodes)
            {
                System.Console.WriteLine("FATAL - Pathfinder ran out of preallocated nodes!");
            }

            return _fixedSizeAllocator[_allocateNodeCount++];
        }

        private MapSearchNode AllocateMapSearchNode(NodePosition nodePosition)
        {
            if (_allocatedMapSearchNodes >= _kPreallocatedMapSearchNodes)
            {
                System.Console.WriteLine("FATAL - HexGrid has run out of preallocated MapSearchNodes!");
            }

            _mapSearchNodePool[_allocatedMapSearchNodes].Position = nodePosition;
            return _mapSearchNodePool[_allocatedMapSearchNodes++];
        }

        private void InitiatePathfind()
        {
            _cancelRequest = false;
            _allocateNodeCount = 0;	// Reset our used node tracking
            _allocatedMapSearchNodes = 0;
        }

        // Set Start and goal states
        private void SetStartAndGoalStates(MapSearchNode startSearchNode, MapSearchNode goalSearchNode)
        {
            _startNode = AllocateNode();
            _goalNode = AllocateNode();

            System.Diagnostics.Debug.Assert((_startNode != null && _goalNode != null));

            _startNode.UserStateMapSearchNode = startSearchNode;
            _goalNode.UserStateMapSearchNode = goalSearchNode;

            _searchState = SearchState.Searching;

            // Initialise the AStar specific parts of the Start Node
            // The user only needs fill out the state information
            _startNode.g = 0;
            _startNode.h = _startNode.UserStateMapSearchNode.GoalDistanceEstimate(_goalNode.UserStateMapSearchNode);
            _startNode.f = _startNode.g + _startNode.h;
            _startNode.Parent = null;

            // Push the start node on the Open list
            _openList.Add(_startNode);

            // Initialise counter for search steps
            _steps = 0;
        }

        // Advances search one step
        private SearchState SearchStep()
        {
            // Firstly break if the user has not initialised the search
            System.Diagnostics.Debug.Assert((_searchState > SearchState.NotInitialized) && (_searchState < SearchState.Invalid));

            // Next I want it to be safe to do a searchstep once the search has succeeded...
            if (_searchState == SearchState.Succeeded || _searchState == SearchState.Failed)
            {
                return _searchState;
            }

            // Failure is defined as emptying the open list as there is nothing left to
            // search...
            // New: Allow user abort
            if (_openList.Count == 0 || _cancelRequest)
            {
                FreeSolutionNodes();
                _searchState = SearchState.Failed;
                return _searchState;
            }

            // Incremement step count
            _steps++;

            // Pop the best node (the one with the lowest f)
            Node node = _openList[0]; // get pointer to the node
            _openList.RemoveAt(0);

            //System.Console.WriteLine("Checking node at " + n.m_UserState.position + ", f: " + n.f);

            // Check for the goal, once we pop that we're done
            if (node.UserStateMapSearchNode.IsGoal(_goalNode.UserStateMapSearchNode))
            {
                // The user is going to use the Goal Node he passed in
                // so copy the parent pointer of n
                _goalNode.Parent = node.Parent;
                _goalNode.g = node.g;

                // A special case is that the goal was passed in as the start state
                // so handle that here
                if (false == node.UserStateMapSearchNode.IsSameState(_startNode.UserStateMapSearchNode))
                {
                    // set the child pointers in each node (except Goal which has no child)
                    Node nodeChild = _goalNode;
                    Node nodeParent = _goalNode.Parent;

                    do
                    {
                        nodeParent.Child = nodeChild;
                        nodeChild = nodeParent;
                        nodeParent = nodeParent.Parent;
                    }
                    while (nodeChild != _startNode); // Start is always the first node by definition
                }

                // delete nodes that aren't needed for the solution
                //FreeUnusedNodes();

                _searchState = SearchState.Succeeded;
                return _searchState;
            }
            else // not goal
            {
                // We now need to generate the successors of this node
                // The user helps us to do this, and we keep the new nodes in m_Successors ...
                _successors.Clear(); // empty vector of successor nodes to n

                // User provides this functions and uses AddSuccessor to add each successor of
                // node 'n' to m_Successors
                bool ret = false;
                if (node.Parent != null)
                {
                    ret = node.UserStateMapSearchNode.GetSuccessors(this, node.Parent.UserStateMapSearchNode);
                }
                else
                {
                    ret = node.UserStateMapSearchNode.GetSuccessors(this, null);
                }

                if (!ret)
                {
                    _successors.Clear(); // empty vector of successor nodes to n

                    // free up everything else we allocated
                    FreeSolutionNodes();

                    _searchState = SearchState.OutOfMemory;
                    return _searchState;
                }

                // Now handle each successor to the current node ...
                Node successor = null;
                int successors_size = _successors.Count;
                for (int i = 0; i < successors_size; ++i)
                {
                    successor = _successors[i];

                    // 	The g value for this successor ...
                    float newg = node.g + node.UserStateMapSearchNode.GetCost(successor.UserStateMapSearchNode);

                    // Now we need to find whether the node is on the open or closed lists
                    // If it is but the node that is already on them is better (lower g)
                    // then we can forget about this successor

                    // First linear search of open list to find node
                    Node openlist_result = null;
                    int openlist_size = _openList.Count;
                    bool foundOpenNode = false;
                    for (int j = 0; j < openlist_size; ++j)
                    {
                        openlist_result = _openList[j];
                        if (openlist_result.UserStateMapSearchNode.IsSameState(successor.UserStateMapSearchNode))
                        {
                            foundOpenNode = true;
                            break;
                        }
                    }

                    if (foundOpenNode)
                    {
                        // we found this state on open
                        if (openlist_result.g <= newg)
                        {
                            // the one on Open is cheaper than this one
                            continue;
                        }
                    }

                    Node closedlist_result = null;
                    int closedlist_size = _closedList.Count;
                    bool foundClosedNode = false;
                    for (int k = 0; k < closedlist_size; ++k)
                    {
                        closedlist_result = _closedList[k];
                        if (closedlist_result.UserStateMapSearchNode.IsSameState(successor.UserStateMapSearchNode))
                        {
                            foundClosedNode = true;
                            break;
                        }
                    }

                    if (foundClosedNode)
                    {
                        // we found this state on closed
                        if (closedlist_result.g <= newg)
                        {
                            // the one on Closed is cheaper than this one
                            continue;
                        }
                    }

                    // This node is the best node so far with this particular state
                    // so lets keep it and set up its AStar specific data ...
                    successor.Parent = node;
                    successor.g = newg;
                    successor.h = successor.UserStateMapSearchNode.GoalDistanceEstimate(_goalNode.UserStateMapSearchNode);
                    successor.f = successor.g + successor.h;

                    // Remove successor from closed if it was on it
                    if (foundClosedNode)
                    {
                        // remove it from Closed
                        _closedList.Remove(closedlist_result);
                    }

                    // Update old version of this node
                    if (foundOpenNode)
                    {
                        _openList.Remove(openlist_result);
                    }

                    SortedAddToOpenList(successor);
                }

                // push n onto Closed, as we have expanded it now
                _closedList.Add(node);

                if (_closedList.Count > _closedListHighWaterMark)
                {
                    _closedListHighWaterMark = _closedList.Count;
                }
            } // end else (not goal so expand)

            return _searchState; // 'Succeeded' bool is false at this point.
        }

        // User calls this to add a successor to a list of successors
        // when expanding the search frontier
        private bool AddSuccessor(MapSearchNode state)
        {
            Node node = AllocateNode();

            if (node != null)
            {
                node.UserStateMapSearchNode = state;
                _successors.Add(node);

                if (_successors.Count > _successorListHighWaterMark)
                {
                    _successorListHighWaterMark = _successors.Count;
                }
                return true;
            }

            return false;
        }

        // Get start node
        private MapSearchNode GetSolutionStart()
        {
            _currentSolutionNode = _startNode;

            if (_startNode != null)
            {
                return _startNode.UserStateMapSearchNode;
            }
            else
            {
                return null;
            }
        }

        // Get next node
        private MapSearchNode GetSolutionNext()
        {
            if (_currentSolutionNode != null)
            {
                if (_currentSolutionNode.Child != null)
                {
                    Node child = _currentSolutionNode.Child;
                    _currentSolutionNode = _currentSolutionNode.Child;
                    return child.UserStateMapSearchNode;
                }
            }

            return null;
        }

        // Free the solution nodes
        // This is done to clean up all used Node memory when you are done with the
        // search
        private void FreeSolutionNodes()
        {
            _openList.Clear();
            _closedList.Clear();
            _successors.Clear();

            for (int i = 0; i < _kPreallocatedNodes; ++i)
            {
                _fixedSizeAllocator[i].ReInitialize();
            }
        }
        public List<NodePosition> Calculate((int, int) startPoint, (int, int) goalPoint)
        {
            this.InitiatePathfind();

            // Create a start state
            MapSearchNode nodeStart = this.AllocateMapSearchNode(new NodePosition(startPoint));

            // Define the goal state
            MapSearchNode nodeEnd = this.AllocateMapSearchNode(new NodePosition(goalPoint));

            // Set Start and goal states
            this.SetStartAndGoalStates(nodeStart, nodeEnd);

            // Set state to Searching and perform the search
            AStarPathfinder.SearchState searchState = AStarPathfinder.SearchState.Searching;
            uint searchSteps = 0;

            do
            {
                searchState = this.SearchStep();
                searchSteps++;
            }
            while (searchState == AStarPathfinder.SearchState.Searching);

            // Search complete
            bool pathfindSucceeded = (searchState == AStarPathfinder.SearchState.Succeeded);

            var newPath = new List<NodePosition>();

            if (pathfindSucceeded)
            {
                // Success
                int numSolutionNodes = 0;	// Don't count the starting cell in the path length

                // Get the start node
                MapSearchNode node = this.GetSolutionStart();
                newPath.Add(node.Position);
                ++numSolutionNodes;

                // Get all remaining solution nodes
                for (; ; )
                {
                    node = this.GetSolutionNext();

                    if (node == null)
                    {
                        break;
                    }

                    ++numSolutionNodes;
                    newPath.Add(node.Position);
                };

                // Once you're done with the solution we can free the nodes up
                this.FreeSolutionNodes();
            }
            return newPath;
        }
    }
}

