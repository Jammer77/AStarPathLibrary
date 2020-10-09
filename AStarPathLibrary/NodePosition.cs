namespace AStarPathLibrary
{
    public struct NodePosition
    {
        private readonly int _x;
        private readonly int _y;

        public int X => this._x;
        public int Y => this._y;

        public NodePosition(int x, int y)
        {
            this._x = x;
            this._y = y;
        }

        public NodePosition((int,int) coord)
        {
            (int x, int y) = coord;
            this._x = x;
            this._y = y;
        }
    }
}
