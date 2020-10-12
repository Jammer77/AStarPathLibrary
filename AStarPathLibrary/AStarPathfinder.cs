using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;

namespace AStarPathLibrary
{
    public sealed partial class AStarPathfinder
    {
        private const int NON_WALKABLE = 9;
        private readonly List<Node> _openList;
        private readonly List<Node> _closedList;
        //private readonly List<Node> _fixedSizeAllocatorNodeList;
        //private readonly List<Node> _mapSearchNodePoolList;

        private readonly Func<int, int, int> _getMapDelegate;
        private SearchState _searchState = SearchState.NotInitialized;
        
        private Node _startNode;
        private Node _goalNode;
        private Node _currentSolutionNode;
        private int _openListHighWaterMark;
        private int _closedListHighWaterMark;

        private readonly int _openListSize = 32;
        private readonly int _closeListSize = 256;

        public AStarPathfinder(Func<int, int, int> getMapDelegate)
        {
            _openList = new List<Node>(_openListSize);
            _closedList = new List<Node>(_closeListSize);
            _getMapDelegate = getMapDelegate;
        }

        private void SortedAddToOpenList(Node node)
        {
            bool inserted = false;

            for (int i = 0; i < _openList.Count; ++i)
            {
                if (node.DistanceCostSum < _openList[i].DistanceCostSum)
                {
                    _openList.Insert(i, node);
                    inserted = true;
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

        private void SetStartAndGoalStates(Node startSearchNode, Node goalSearchNode)
        {
            _startNode = startSearchNode;
            _goalNode = goalSearchNode;

            _searchState = SearchState.Searching;

            _startNode.Cost = 0;
            _startNode.Distance = _startNode.CalculateDistance(_goalNode);
            _startNode.DistanceCostSum = _startNode.Cost + _startNode.Distance;
            _openList.Add(_startNode);
        }

        // Advances search one step
        private SearchState SearchStep()
        {
            Debug.Assert((_searchState > SearchState.NotInitialized) && (_searchState < SearchState.Invalid));

            // Next I want it to be safe to do a searchstep once the search has succeeded...
            if (_searchState == SearchState.Succeeded || _searchState == SearchState.Failed)
            {
                return _searchState;
            }

            // Failure is defined as emptying the open list as there is nothing left to
            // search...
            // New: Allow user abort
            if (_openList.Count == 0)
            {
                FreeSolutionNodes();
                _searchState = SearchState.Failed;
                return _searchState;
            }

            // Pop the best node (the one with the lowest f)
            Node node = _openList[0]; // get pointer to the node
            _openList.RemoveAt(0);

            //System.Console.WriteLine("Checking node at " + n.m_UserState.position + ", f: " + n.f);

            // Check for the goal, once we pop that we're done
            if (node.IsEqual(_goalNode))
            {
                // The user is going to use the Goal Node he passed in
                // so copy the parent pointer of n
                _goalNode.Parent = node.Parent;
                _goalNode.Cost = node.Cost;

                // A special case is that the goal was passed in as the start state
                // so handle that here
                if (!node.IsEqual(_startNode))
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
                var successors = this.GetSuccessors(node).ToList();
                bool ret = successors.Any();

                if (!ret)
                {
                    FreeSolutionNodes();
                    _searchState = SearchState.OutOfMemory;
                    return _searchState;
                }

                int successors_size = successors.Count;
                
                for (int i = 0; i < successors_size; ++i)
                {
                    // Now handle each successor to the current node ...
                    Node successor = successors[i];

                    // 	The g value for this successor ...
                    float newg = node.Cost + node.GetCost(successor);

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
                        if (openlist_result.IsEqual(successor))
                        {
                            foundOpenNode = true;
                            break;
                        }
                    }

                    if (foundOpenNode)
                    {
                        // we found this state on open
                        if (openlist_result.Cost <= newg)
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
                        if (closedlist_result.IsEqual(successor))
                        {
                            foundClosedNode = true;
                            break;
                        }
                    }

                    if (foundClosedNode)
                    {
                        // we found this state on closed
                        if (closedlist_result.Cost <= newg)
                        {
                            // the one on Closed is cheaper than this one
                            continue;
                        }
                    }

                    // This node is the best node so far with this particular state
                    // so lets keep it and set up its AStar specific data ...
                    successor.Parent = node;
                    successor.Cost = newg;
                    successor.Distance = successor.CalculateDistance(_goalNode);
                    successor.DistanceCostSum = successor.Cost + successor.Distance;

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

        private IEnumerable<Node> GetSuccessors(Node node)
        {
            var result = new List<Node>();
            NodePosition parentPos = node.Parent == null ? new NodePosition(0, 0) : node.Parent.Position;

            result.Add(AddNeighbourNode(-1, 0, parentPos, node));
            result.Add(AddNeighbourNode(0, -1, parentPos, node));
            result.Add(AddNeighbourNode(1, 0, parentPos, node));
            result.Add(AddNeighbourNode(0, 1, parentPos, node));

            return result.Where( o => o != null); //TODO
        }

        public bool IsNeigbourValid(NodePosition node, int xOffset, int yOffset) =>
                this._getMapDelegate(node.X + xOffset, node.Y + yOffset) < NON_WALKABLE;

        private Node AddNeighbourNode(int xOffset, int yOffset, NodePosition parentPos, Node node)
        {
            Node newNode = null;
            bool isNeigbourValid = IsNeigbourValid(node.Position, xOffset, yOffset);
            if (isNeigbourValid &&
                !(parentPos.X == node.Position.X + xOffset && parentPos.Y == node.Position.Y + yOffset))
            {
                (int, int) neighbourPos = (node.Position.X + xOffset, node.Position.Y + yOffset);
                newNode = new Node(neighbourPos, this, _getMapDelegate);
            }
            return newNode;
        }

        // Get start node
        private Node GetSolutionStart()
        {
            _currentSolutionNode = _startNode;
            return _startNode;
        }

        // Get next node
        private Node GetSolutionNext()
        {
            if (_currentSolutionNode != null)
            {
                if (_currentSolutionNode.Child != null)
                {
                    Node child = _currentSolutionNode.Child;
                    _currentSolutionNode = _currentSolutionNode.Child;
                    return child;
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
        }

        public List<NodePosition> Calculate((int, int) startPoint, (int, int) goalPoint)
        {
            var nodeStart = new Node(startPoint, this, _getMapDelegate);
            var nodeEnd = new Node(goalPoint, this, _getMapDelegate);

            // Set Start and goal states
            this.SetStartAndGoalStates(nodeStart, nodeEnd);

            // Set state to Searching and perform the search
            AStarPathfinder.SearchState searchState = SearchState.Searching;
            uint searchSteps = 0;

            do
            {
                searchState = this.SearchStep();
                searchSteps++;
            }
            while (searchState == SearchState.Searching);

            // Search complete
            bool pathfindSucceeded = searchState == SearchState.Succeeded;

            var newPath = new List<NodePosition>();

            if (pathfindSucceeded)
            {
                // Success
                int numSolutionNodes = 0;	// Don't count the starting cell in the path length

                // Get the start node
                Node node = this.GetSolutionStart();
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

