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
        public SubDivision2d(Vector2[] points, double margin)
        {
            Bounds bounds = Utils.CalculateBounds(points, margin);
            _impl = new SubDiv2D_Mutable(bounds);
            InsertRange(points);
        }

        public SubDivision2d(Vector2[] points):this(points, 0.0)
        {

        }

        public SubDivision2d(Vector2 min, Vector2 max)
        {
            _impl = new SubDiv2D_Mutable(new Bounds(min.X, max.X, min.Y, max.Y));
        }

        private SubDiv2D_Mutable _impl;


        public Vector2 Min => _impl.Bounds.MinValue;

        public Vector2 Max => _impl.Bounds.MaxValue;

        public NodeId Insert(Vector2 point)
        {
            return _impl.Insert(point);
        }

        public NodeId Insert(float x, float y)
        {
            return _impl.Insert(new Vector2(x, y));
        }

        public void InsertRange(IEnumerable<Vector2> points)
        {
            foreach (var p in points) 
            {
                _impl.Insert(p);
            }
        }

        public void Clear()
        {
            var bounds = _impl.Bounds;
            _impl = new SubDiv2D_Mutable(bounds);
        }

        public IEnumerable<Triangle> GetDelaunayTriangles()
        {
            return _impl.GetTriangles();
        }

        public IEnumerable<(Vector2, Vector2)> GetEdges()
        {
            return _impl.GetEdges();
        }

        public NodeId? FindNearest(Vector2 point, out Vector2 neigborLocation)
        {
            return _impl.FindNearest(point, out neigborLocation);
        }
        
        public NodeId? FindNearest(float x, float y, out Vector2 neigborLocation)
        {
            return _impl.FindNearest(new Vector2(x, y), out neigborLocation);
        }

        public IEnumerable<VoronoiFacet> GetVoronoiFacets()
        {
            return _impl.GetVoronoiFacets();
        }

        public IEnumerable<VoronoiFacet> GetVoronoiFacets(IReadOnlyList<NodeId> vertices)
        {
            return _impl.GetVoronoiFacets(vertices);
        }

    }
}
