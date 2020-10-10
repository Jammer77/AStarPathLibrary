using System;

namespace AStarPathLibrary
{
    public partial class AStarPathfinder
    {
        private class MapSearchNode
        {
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
            public float GoalDistanceEstimate(MapSearchNode nodeGoal)
            {
                //TODO зачем 2 преведения?
                double X = (double)Position.X - (double)nodeGoal.Position.X;
                double Y = (double)Position.Y - (double)nodeGoal.Position.Y;
                return ((float)Math.Sqrt((X * X) + (Y * Y)));
            }

            public bool IsGoal(MapSearchNode nodeGoal)
            {
                return (Position.X == nodeGoal.Position.X && Position.Y == nodeGoal.Position.Y);
            }

            public bool ValidNeigbour(int xOffset, int yOffset)
            {
                // Return true if the node is navigable and within grid bounds
                return (this._getMapDelegate(Position.X + xOffset, Position.Y + yOffset) < 9);
            }

            void AddNeighbourNode(int xOffset, int yOffset, NodePosition parentPos)
            {
                if (ValidNeigbour(xOffset, yOffset) &&
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
            public bool GetSuccessors(AStarPathfinder aStarSearch, MapSearchNode parentNode)
            {
                var parentPos = new NodePosition(0, 0);

                if (parentNode != null)
                {
                    parentPos = parentNode.Position;
                }

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

            public bool IsSameState(MapSearchNode rhs)
            {
                return (Position.X == rhs.Position.X &&
                    Position.Y == rhs.Position.Y);
            }
        }
    }
}
