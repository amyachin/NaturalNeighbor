using System.Numerics;
using System.Collections.Immutable;

namespace NaturalNeighbor 
{

    /// <summary>
    /// Represents a cell from Voronoi diagram
    /// </summary>
    public class VoronoiFacet
    {
        /// <summary>
        /// Constructs a new facet instance 
        /// </summary>
        /// <param name="id">Node id</param>
        /// <param name="center">Node location on XY pane</param>
        /// <param name="vertices">Perimieter of the voronoi cell forming a convex polygon. Points are arranged counterclockwise</param>
        public VoronoiFacet(NodeId id, Vector2 center, ImmutableArray<Vector2> vertices)
        {
            Id = id;
            Center = center;
            Vertices = vertices;
        }

        /// <summary>
        /// Constructs an empty facet 
        /// </summary>
        /// <param name="id"></param>
        /// <param name="center"></param>
        public VoronoiFacet(NodeId id, Vector2 center)
        {
            Id = id;
            Center = center;
            Vertices = ImmutableArray<Vector2>.Empty;
        }


        /// <summary>
        /// Area of the polygon formed by the vertices
        /// </summary>
        public double Area => Internal.Utils.ComputePolygonArea2(Vertices) * 0.5;

        /// <summary>
        /// True if facet is outide of a voronoy diagram boundaries
        /// </summary>
        public bool IsVirtual => Vertices.IsDefaultOrEmpty;

        /// <summary>
        /// ID of the node representing the center coordinate
        /// </summary>
        public NodeId Id { get; }

        /// <summary>
        /// Node position on XY plane
        /// </summary>
        public Vector2 Center { get; }

        /// <summary>
        /// Vertices of the polygon forming the facet perimeter sorted counterclockwise
        /// </summary>
        public ImmutableArray<Vector2> Vertices { get; }
    }


}