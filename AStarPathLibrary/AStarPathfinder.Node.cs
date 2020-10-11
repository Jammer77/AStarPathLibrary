using System;
using System.Diagnostics;

namespace AStarPathLibrary
{
    public partial class AStarPathfinder
    {
        // A node represents a possible state in the search
        // The user provided state type is included inside this type
        [DebuggerDisplay("{Position.X}, {Position.Y}")]
        private class Node
        {
            private Node _parent; // used during the search to record the parent of successor nodes
            private Node _child; // used after the search for the application to view the search in reverse

            private float _cost; // cost of this node + it's predecessors
            private float _distance; // heuristic estimate of distance to goal
            private float _distanceCostSum; // sum of cumulative cost of predecessors and self and heuristic

            public float Cost { get => this._cost; set => this._cost = value; }
            public float Distance { get => this._distance; set => this._distance = value; }
            public float DistanceCostSum { get => this._distanceCostSum; set => this._distanceCostSum = value; }

            public void ReInitialize()
            {
                Parent = null;
                Child = null;
                Cost = 0.0f;
                Distance = 0.0f;
                DistanceCostSum = 0.0f;
            }

            public Node Parent 
            { 
                get => this._parent; 
                set => this._parent = value; 
            }
            public Node Child 
            { 
                get => this._child; 
                set => this._child = value; 
            }

            private const int NON_WALKABLE = 9;
            private NodePosition _position;
            private readonly AStarPathfinder _pathfinder = null;
            private readonly Func<int, int, int> _getMapDelegate;

            public NodePosition Position { get => this._position; set => this._position = value; }

            public Node(AStarPathfinder pathfinder, Func<int, int, int> getMapDelegate)
            {
                ReInitialize();
                this._position = new NodePosition(0, 0);
                this._pathfinder = pathfinder;
                this._getMapDelegate = getMapDelegate;
            }

            // Here's the heuristic function that estimates the distance from a Node
            // to the Goal.
            public float DistanceEstimate(Node nodeGoal)
            {
                double diffX = (double)(Position.X - nodeGoal.Position.X);
                double diffY = (double)(Position.Y - nodeGoal.Position.Y);
                return (float)Math.Sqrt((diffX * diffX) + (diffY * diffY));
            }

            public bool IsEqual(Node nodeGoal) =>
                this.Position.X == nodeGoal.Position.X && this.Position.Y == nodeGoal.Position.Y;

            public bool IsNeigbourValid(int xOffset, int yOffset) =>
                this._getMapDelegate(Position.X + xOffset, Position.Y + yOffset) < NON_WALKABLE;

            private void AddNeighbourNode(int xOffset, int yOffset, NodePosition parentPos)
            {
                if (IsNeigbourValid(xOffset, yOffset) &&
                    !(parentPos.X == Position.X + xOffset && parentPos.Y == Position.Y + yOffset))
                {
                    var neighbourPos = new NodePosition(Position.X + xOffset, Position.Y + yOffset);
                    Node newNode = _pathfinder.AllocateMapSearchNode(neighbourPos);

                    _pathfinder.AddSuccessor(newNode);
                }
            }

            // This generates the successors to the given Node. It uses a helper function called
            // AddSuccessor to give the successors to the AStar class. The A* specific initialisation
            // is done for each node internally, so here you just set the state information that
            // is specific to the application
            public bool GetSuccessors(Node parentNode)
            {
                NodePosition parentPos = parentNode == null ? new NodePosition(0, 0) : parentNode.Position;

                // push each possible move except allowing the search to go backwards
                AddNeighbourNode(-1, 0, parentPos);
                AddNeighbourNode(0, -1, parentPos);
                AddNeighbourNode(1, 0, parentPos);
                AddNeighbourNode(0, 1, parentPos);

                return true;
            }

            // given this node, what does it cost to move to successor. In the case
            // of our map the answer is the map terrain value at this node since that is
            // conceptually where we're moving
            public float GetCost(Node successor) => _getMapDelegate(successor.Position.X, successor.Position.Y);

        };
    }}

