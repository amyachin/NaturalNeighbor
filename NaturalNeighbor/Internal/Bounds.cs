using System;
using System.Numerics;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NaturalNeighbor.Internal
{
    internal class Bounds
    {
        public Bounds(double minX, double maxX, double minY, double maxY)
        {
            MinValue = new Vector2((float)Math.Min(minX, maxX), (float)Math.Min(minY, maxY));
            MaxValue = new Vector2((float)Math.Max(minX, maxX), (float)Math.Max(minY, maxY)); ;
        }

        public bool Contains(Vector2 point)
        {

            var min = MinValue;
            if (point.X < min.X || point.Y < min.Y)
            {
                return false;
            }

            var max = MaxValue;

            if (point.X > max.X || point.Y > max.Y)
            {
                return false;
            }

            return true;
        }

        public Vector2 MinValue { get; }
        public Vector2 MaxValue { get; }

        public float Width => this.MaxValue.X - this.MinValue.X;

        public float Height => this.MaxValue.Y - this.MinValue.Y;


        public static Bounds Union(Bounds left, Bounds right)
        {
            float minX = Math.Min(left.MinValue.X, right.MinValue.X);
            float minY = Math.Min(left.MinValue.Y, right.MinValue.Y);

            float maxX = Math.Max(left.MaxValue.X, right.MaxValue.X);
            float maxY = Math.Max(left.MaxValue.Y, right.MaxValue.Y);

            return new Bounds(minX, maxX, minY, maxY);
        }
    }
}
