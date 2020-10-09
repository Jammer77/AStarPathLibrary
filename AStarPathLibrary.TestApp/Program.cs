using System;
using System.Collections.Generic;

namespace AStarPathLibrary.TestApp
{
    class Program
    {
        static void Main(string[] args)
        {
            var map = new Map();
            var pathfinder = new AStarPathfinder((x, y) => map[x, y]);
            List<NodePosition> path = pathfinder.Calculate((0, 0), (2, 4));
            Console.ReadKey();
        }
    }
}
