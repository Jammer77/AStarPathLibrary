namespace AStarPathLibrary
{
    public partial class AStarPathfinder
    {
        // A node represents a possible state in the search
        // The user provided state type is included inside this type
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

            public Node()
            {
                ReInitialize();
            }

            public void ReInitialize()
            {
                Parent = null;
                Child = null;
                Cost = 0.0f;
                Distance = 0.0f;
                DistanceCostSum = 0.0f;
            }

            private MapSearchNode _userState;

            public MapSearchNode UserStateMapSearchNode 
            {
                get => this._userState; 
                set => this._userState = value;
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
        };
    }}

