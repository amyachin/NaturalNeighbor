using System.Numerics;

namespace NaturalNeighbor
{
    /// <summary>
    /// Represents triangle on XY plane
    /// </summary>
    public class Triangle
    {
       /// <summary>
       /// Constructs an instance of the triangle
       /// </summary>
       /// <param name="p1"></param>
       /// <param name="p2"></param>
       /// <param name="p3"></param>
        public Triangle(Vector2 p1, Vector2 p2, Vector2 p3)
        {
            P1 = p1;
            P2 = p2;
            P3 = p3;
        }

        /// <summary>
        /// The first corner
        /// </summary>
        public Vector2 P1 { get; }
        /// <summary>
        /// The second corner
        /// </summary>
        public Vector2 P2 { get; }
        /// <summary>
        /// The third corner
        /// </summary>
        public Vector2 P3 { get; }
    }

}