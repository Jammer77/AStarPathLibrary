using System;

namespace AStarPathLibrary
{
    public partial class AStarPathfinder
    {
        private class MapSearchNode
        {
            private const int NON_WALKABLE = 9; 
            private NodePosition _position;
            private readonly AStarPathfinder _pathfinder = null;
            private readonly Func<int, int, int> _getMapDelegate;

            public NodePosition Position { get => this._position; set => this._position = value; }

            public MapSearchNode(AStarPathfinder pathfinder, Func<int, int, int> getMapDelegate)
            {
                this._position = new NodePosition(0, 0);
                this._pathfinder = pathfinder;
                this._getMapDelegate = getMapDelegate;
            }

            public MapSearchNode(NodePosition position, AStarPathfinder pathfinder, Func<int, int, int> getMap)
            {
                this._position = new NodePosition(position.X, position.Y);
                this._pathfinder = pathfinder;
                this._getMapDelegate = getMap;
            }

            // Here's the heuristic function that estimates the distance from a Node
            // to the Goal.
            public float DistanceEstimate(MapSearchNode nodeGoal)
            {
                double diffX = (double)(Position.X - nodeGoal.Position.X);
                double diffY = (double)(Position.Y - nodeGoal.Position.Y);
                return (float)Math.Sqrt((diffX * diffX) + (diffY * diffY));
            }

            public bool IsEqual(MapSearchNode nodeGoal) =>
                this.Position.X == nodeGoal.Position.X && this.Position.Y == nodeGoal.Position.Y;

            public bool IsNeigbourValid(int xOffset, int yOffset) =>
                this._getMapDelegate(Position.X + xOffset, Position.Y + yOffset) < NON_WALKABLE;
                // Return true if the node is navigable and within grid bounds

            void AddNeighbourNode(int xOffset, int yOffset, NodePosition parentPos)
            {
                if (IsNeigbourValid(xOffset, yOffset) &&
                    !(parentPos.X == Position.X + xOffset && parentPos.Y == Position.Y + yOffset))
                {
                    var neighbourPos = new NodePosition(Position.X + xOffset, Position.Y + yOffset);
                    MapSearchNode newNode = _pathfinder.AllocateMapSearchNode(neighbourPos);
                    _pathfinder.AddSuccessor(newNode);
                }
            }

            // This generates the successors to the given Node. It uses a helper function called
            // AddSuccessor to give the successors to the AStar class. The A* specific initialisation
            // is done for each node internally, so here you just set the state information that
            // is specific to the application
            public bool GetSuccessors(MapSearchNode parentNode)
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
            public float GetCost(MapSearchNode successor) => _getMapDelegate(successor.Position.X, successor.Position.Y);
        }
    }
}
