using System.Numerics;

namespace NaturalNeighbor
{
    public struct Triangle
    {
        public Triangle(Vector2 p1, Vector2 p2, Vector2 p3)
        {
            P1 = p1;
            P2 = p2;
            P3 = p3;
        }

        public Vector2 P1 { get; }
        public Vector2 P2 { get; }
        public Vector2 P3 { get; }
    }

}