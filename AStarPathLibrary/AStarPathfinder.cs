using System;
using System.Collections.Generic;
using System.Linq;

namespace AStarPathLibrary
{
    public sealed partial class AStarPathfinder
    {
        private const int NonWalkable = 9;
        private readonly List<Node> _openList;
        private readonly List<Node> _closedList;

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

        private void SetStartAndGoalNode(Node startSearchNode, Node goalSearchNode)
        {
            _startNode = startSearchNode;
            _goalNode = goalSearchNode;

            _searchState = SearchState.Searching;

            _startNode.Cost = 0;
            _startNode.Distance = _startNode.CalculateDistance(_goalNode);
            _startNode.DistanceCostSum = _startNode.Cost + _startNode.Distance;
            _openList.Add(_startNode);
        }

        private SearchState SearchStep()
        {
            if (_searchState == SearchState.Succeeded || _searchState == SearchState.Failed)
            {
                return _searchState;
            }

            if (!_openList.Any())
            {
                FreeSolutionNodes();
                _searchState = SearchState.Failed;
                return _searchState;
            }

            Node node = _openList[0];
            _openList.RemoveAt(0);

            if (node.IsEqual(_goalNode))
            {
                _goalNode.Parent = node.Parent;
                _goalNode.Cost = node.Cost;

                if (!node.IsEqual(_startNode))
                {
                    Node nodeChild = _goalNode;
                    Node nodeParent = _goalNode.Parent;

                    do
                    {
                        nodeParent.Child = nodeChild;
                        nodeChild = nodeParent;
                        nodeParent = nodeParent.Parent;
                    }
                    while (nodeChild != _startNode);
                }

                _searchState = SearchState.Succeeded;
                return _searchState;
            }

            var successors = this.GetSuccessors(node).ToList();
            bool ret = successors.Any();

            if (!ret)
            {
                FreeSolutionNodes();
                _searchState = SearchState.OutOfMemory;
                return _searchState;
            }

            int successorsSize = successors.Count;
                
            for (int i = 0; i < successorsSize; ++i)
            {
                Node successor = successors[i];
                float cost = node.Cost + node.GetCost(successor);
                Node openlistResult = null;
                int openlistSize = _openList.Count;
                bool foundOpenNode = false;
                for (int j = 0; j < openlistSize; ++j)
                {
                    openlistResult = _openList[j];
                    if (openlistResult.IsEqual(successor))
                    {
                        foundOpenNode = true;
                        break;
                    }
                }

                if (foundOpenNode)
                {
                    if (openlistResult.Cost <= cost)
                    {
                        continue;
                    }
                }

                Node closedlistResult = null;
                int closedlistSize = _closedList.Count;
                bool foundClosedNode = false;
                for (int k = 0; k < closedlistSize; ++k)
                {
                    closedlistResult = _closedList[k];
                    if (closedlistResult.IsEqual(successor))
                    {
                        foundClosedNode = true;
                        break;
                    }
                }

                if (foundClosedNode)
                {
                    if (closedlistResult.Cost <= cost)
                    {
                        continue;
                    }
                }

                successor.Parent = node;
                successor.Cost = cost;
                successor.Distance = successor.CalculateDistance(_goalNode);
                successor.DistanceCostSum = successor.Cost + successor.Distance;

                if (foundClosedNode)
                {
                    _closedList.Remove(closedlistResult);
                }

                if (foundOpenNode)
                {
                    _openList.Remove(openlistResult);
                }
                SortedAddToOpenList(successor);
            }
            _closedList.Add(node);
            if (_closedList.Count > _closedListHighWaterMark)
            {
                _closedListHighWaterMark = _closedList.Count;
            }
            return _searchState;
        }

        private IEnumerable<Node> GetSuccessors(Node node)
        {
            var result = new List<Node>();
            var parentPos = node.Parent?.Position ?? new NodePosition(0, 0);
            foreach ((int xOffset, int yOffset) in new []{(-1,0), (0,-1), (1,0), (0,1)})
            {
                if (IsNeighbourValid(xOffset, yOffset, parentPos, node))
                {
                   var successors = CreateNeighbourNode(xOffset, yOffset, node);
                   result.Add(successors);  
                }
            }
            return result;
        }

        private bool IsNeighbourWalkable(NodePosition node, int xOffset, int yOffset) =>
                this._getMapDelegate(node.X + xOffset, node.Y + yOffset) < NonWalkable;

        private Node CreateNeighbourNode(int xOffset, int yOffset, Node node)
        {
            (int, int) neighbourPos = (node.Position.X + xOffset, node.Position.Y + yOffset);
            var newNode = new Node(neighbourPos, this, _getMapDelegate);
            return newNode;
        }

        private bool IsNeighbourValid(int xOffset, int yOffset, NodePosition parentPos, Node node)
        {
            bool isNeighbourValid = IsNeighbourWalkable(node.Position, xOffset, yOffset);
            return (isNeighbourValid &&
                    !(parentPos.X == node.Position.X + xOffset && parentPos.Y == node.Position.Y + yOffset));
        }

        private Node GetSolutionStart()
        {
            _currentSolutionNode = _startNode;
            return _startNode;
        }

        private Node GetSolutionNext()
        {
            if (_currentSolutionNode?.Child == null) return null;
            _currentSolutionNode = _currentSolutionNode.Child;
            return _currentSolutionNode;
        }

        private void FreeSolutionNodes()
        {
            _openList.Clear();
            _closedList.Clear();
        }

        public List<NodePosition> Calculate((int, int) startPoint, (int, int) goalPoint)
        {
            var nodeStart = new Node(startPoint, this, _getMapDelegate);
            var nodeEnd = new Node(goalPoint, this, _getMapDelegate);

            this.SetStartAndGoalNode(nodeStart, nodeEnd);
            SearchState searchState = SearchState.Searching;

            do
            {
                searchState = this.SearchStep();
            }
            while (searchState == SearchState.Searching);

            bool pathfinderSucceeded = searchState == SearchState.Succeeded;

            var newPath = new List<NodePosition>();

            if (pathfinderSucceeded)
            {
                Node node = this.GetSolutionStart();
                newPath.Add(node.Position);
                for (;;)
                {
                    node = this.GetSolutionNext();
                    if (node == null)
                    {
                        break;
                    }
                    newPath.Add(node.Position);
                };
                this.FreeSolutionNodes();
            }
            return newPath;
        }
    }
}