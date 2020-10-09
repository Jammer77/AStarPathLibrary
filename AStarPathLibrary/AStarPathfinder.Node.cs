namespace AStarPathLibrary
{
    public partial class AStarPathfinder
    {
        // A node represents a possible state in the search
        // The user provided state type is included inside this type
        class Node
        {
            private Node _parent; // used during the search to record the parent of successor nodes
            private Node _child; // used after the search for the application to view the search in reverse

            public float g; // cost of this node + it's predecessors
            public float h; // heuristic estimate of distance to goal
            public float f; // sum of cumulative cost of predecessors and self and heuristic

            public Node()
            {
                ReInitialize();
            }

            public void ReInitialize()
            {
                Parent = null;
                Child = null;
                g = 0.0f;
                h = 0.0f;
                f = 0.0f;
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

