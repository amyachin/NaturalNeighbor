using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;

namespace NaturalNeighbor
{
    using Internal;

    /// <summary>
    /// Performs planar subdivision
    /// </summary>
    public class SubDivision2d
    {

        /// <summary>
        /// Constructs a new subdivision including the specified points
        /// </summary>
        /// <param name="points">XY coordinates</param>
        /// <param name="margin">Margin for the bounding box</param>
        public SubDivision2d(Vector2[] points, double margin)
        {
            Bounds bounds = Utils.CalculateBounds(points, (float)margin, null, null);
            _impl = new SubDiv2D_Mutable(bounds);
            _searchContext = new SearchContext();
            InsertRange(points);
        }

        internal int NumberOfSamples { get; private set; }

        private SearchContext _searchContext;

        /// <summary>
        /// Constructs a new subdivision including the specified points
        /// </summary>
        /// <param name="points">XY coordinates</param>
        public SubDivision2d(Vector2[] points):this(points, 0.0)
        {

        }

        /// <summary>
        /// Constructs an empty subdivision with the specified boundaries
        /// </summary>
        /// <param name="min">the bottom left corner</param>
        /// <param name="max">the top right corner</param>
        public SubDivision2d(Vector2 min, Vector2 max)
        {
            _impl = new SubDiv2D_Mutable(new Bounds(min, max));
            _searchContext = new SearchContext();
        }

        internal SubDiv2D_Mutable _impl;



        /// <summary>
        /// The bottom left corner of the bounding box
        /// </summary>
        public Vector2 MinValue => _impl.Bounds.MinValue;


        /// <summary>
        /// The top right corner of the bounding box
        /// </summary>
        public Vector2 MaxValue => _impl.Bounds.MaxValue;

        /// <summary>
        /// Inserts a node into Delaunay graph 
        /// </summary>
        /// <param name="point">xy coordinate</param>
        /// <returns>node Id</returns>
        public NodeId Insert(Vector2 point)
        {
            return _impl.Insert(point, _searchContext);
        }

        /// <summary>
        /// Inserts a node into Delaunay graph 
        /// </summary>
        /// <param name="x">x coordinate</param>
        /// <param name="y">y coordinate</param>
        /// <returns>node Id</returns>
        public NodeId Insert(float x, float y)
        {

            var result = _impl.Insert(new Vector2(x, y), _searchContext);
            if (_searchContext.Vertex == 0)
            {
                NumberOfSamples++;
            }

            return result;
        }

        /// <summary>
        /// Inserts a sequence of nodes into Delaunay graph 
        /// </summary>
        /// <param name="points">xy coordinates</param>
        public void InsertRange(IEnumerable<Vector2> points)
        {
            foreach (var p in points) 
            {
                _impl.Insert(p, _searchContext);
                if (_searchContext.Vertex == 0)
                {
                    NumberOfSamples++;
                }
            }
        }


        /// <summary>
        /// Clears the graph retaining the bounding box
        /// </summary>

        public void Clear()
        {
            var bounds = _impl.Bounds;
            _impl = new SubDiv2D_Mutable(bounds);
            _searchContext.Clear();
            NumberOfSamples = 0;
        }

        /// <summary>
        /// Reads delaunay triangles in the current node set
        /// </summary>
        /// <returns>list of triangles</returns>

        public IEnumerable<Triangle> GetDelaunayTriangles()
        {
            return _impl.GetTriangles();
        }

        /// <summary>
        /// Reads all edges of graph
        /// </summary>
        /// <returns>A list of endpoints forming an edge</returns>
        public IEnumerable<(Vector2, Vector2)> GetEdges()
        {
            return _impl.GetEdges();
        }


        /// <summary>
        /// Finds a node closest to the specified location
        /// </summary>
        /// <param name="point">Location xy coordinate</param>
        /// <param name="result">Closest node location</param>
        /// <returns> Found node Id, null if point is out of bounds</returns>
        public NodeId? FindNearest(Vector2 point, out Vector2 result)
        {
            return _impl.FindNearest(point, _searchContext, out result);
        }


        /// <summary>
        /// Finds a node closest to the specified location
        /// </summary>
        /// <param name="x">Location X coordinate</param>
        /// <param name="y">Location Y coordinate</param>
        /// <param name="result">Closest node location</param>
        /// <returns>Id of the found node or null if point is out of bounds</returns>
        public NodeId? FindNearest(float x, float y, out Vector2 result)
        {
            return _impl.FindNearest(new Vector2(x, y), _searchContext, out result);
        }


        /// <summary>
        /// Reads all voronoi facets
        /// </summary>
        /// <returns>A list of facets</returns>
        public IEnumerable<VoronoiFacet> GetVoronoiFacets()
        {
            return _impl.GetVoronoiFacets();
        }


        /// <summary>
        /// Reads specific voronoi facets
        /// </summary>
        /// <param name="vertices">node ids</param>
        /// <returns>A list of requested facets</returns>
        public IEnumerable<VoronoiFacet> GetVoronoiFacets(IReadOnlyList<NodeId> vertices)
        {
            return _impl.GetVoronoiFacets(vertices);
        }

    }
}
