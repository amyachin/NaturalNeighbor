using System.Numerics;
using System.Collections.Immutable;

namespace NaturalNeighbor 
{
    public class VoronoiFacet
    {
        public VoronoiFacet(NodeId id, Vector2 center, ImmutableArray<Vector2> vertices)
        {
            Id = id;
            Center = center;
            Vertices = vertices;
        }

        public VoronoiFacet(NodeId id, Vector2 center)
        {
            Id = id;
            Center = center;
            Vertices = ImmutableArray<Vector2>.Empty;
        }

        public double Area => Internal.Utils.ComputePolygonArea2(Vertices) * 0.5;

        public bool IsVirtual => Vertices.IsDefaultOrEmpty;

        public NodeId Id { get; }

        public Vector2 Center { get; }

        public ImmutableArray<Vector2> Vertices { get; }
    }


}