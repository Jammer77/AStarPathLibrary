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

            private readonly Func<int, int, int> _getMapDelegate;

            private NodePosition _position;
            public NodePosition Position { get => this._position; set => this._position = value; }

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

            public Node((int, int) coord, AStarPathfinder pathfinder, Func<int, int, int> getMapDelegate)
            {
                ReInitialize();
                this._position = new NodePosition(coord);
                this._getMapDelegate = getMapDelegate;
            }

            public float CalculateDistance(Node nodeGoal)
            {
                double diffX = Position.X - nodeGoal.Position.X;
                double diffY = Position.Y - nodeGoal.Position.Y;
                return (float)Math.Sqrt((diffX * diffX) + (diffY * diffY));
            }

            public bool IsEqual(Node nodeGoal) =>
                this.Position.X == nodeGoal.Position.X && this.Position.Y == nodeGoal.Position.Y;

            public float GetCost(Node successor) => _getMapDelegate(successor.Position.X, successor.Position.Y);

        };
    }}