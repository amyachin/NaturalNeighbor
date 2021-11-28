
using System.Numerics;

namespace NaturalNeighbor.Internal
{
    internal class ImmutableVertexData
    {
        private ImmutableVertexData()
        {
            firstEdge = 0;
            type = -1;
            pt = new Vector2(float.NaN, float.NaN);
        }

        public static readonly ImmutableVertexData Default = new ImmutableVertexData();

        public ImmutableVertexData(Vector2 pt, bool isvirtual, int firstEdge)
        {
            this.firstEdge = firstEdge;
            this.type = isvirtual ? 1 : 0;
            this.pt = pt;
        }

        public ImmutableVertexData(Vector2 pt, int type, int firstEdge)
        {
            this.firstEdge = firstEdge;
            this.type = type;
            this.pt = pt;
        }


        public bool IsVirtual => type > 0;
        public bool IsFree => type < 0;

        public readonly int firstEdge;

        public readonly int type;

        public readonly Vector2 pt;
    }
}
