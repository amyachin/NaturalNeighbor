
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;

namespace NaturalNeighbor
{
    using Internal;


    /// <summary>
    /// Spatial interpolation based on Voronoi tesellation (by Robin Sibson). Provides a smoother approximation compared to nearest neighbor.
    /// </summary>
    public class Interpolator2d
    {

        public Interpolator2d()
        {
            _zHeights = new Dictionary<NodeId, double>();
        }

        public void Generate(Vector2[] points, double[] heights)
        {
            Generate(points, heights, margin: float.Epsilon * 10);
        }

        public void Generate(Vector2[] points, double[] heights, double margin)
        {
            margin = Math.Max(margin, float.Epsilon);

            if (points == null)
            {
                throw new ArgumentNullException(nameof(points));
            }

            if (heights == null)
            {
                throw new ArgumentNullException(nameof(heights));
            }

            if (points.Length != heights.Length)
            {
                throw new ArgumentOutOfRangeException("Points and heights lengths do not match.");
            }

            var bounds = Utils.CalculateBounds(points, margin);
            GenerateCore(points, heights, bounds);
        }

        private void GenerateCore(Vector2[] points, double[] heights, Bounds bounds)
        {

            if (points.Length != heights.Length)
            {
                throw new ArgumentOutOfRangeException("Point array length does not match the heights array length.");
            }

            for (int i = 0; i < points.Length; ++i)
            {
                if (!bounds.Contains(points[i]))
                {
                    throw new ArgumentOutOfRangeException($"points[{i}] is outside the specified boundaries.");
                }
            }

            // Initialize 
            _zHeights.Clear();
            _snapshot = null;
            _facets = null;

            _impl = new SubDiv2D_Mutable(bounds);

            // Generate delaunay graph for the specified points
            for (int i = 0; i < points.Length; ++i)
            {
                var id = _impl.Insert(points[i]);
                _zHeights.Add(id, heights[i]);
            }
        }


        private void InitSnapshot()
        {
            if (_snapshot != null)
            {
                return;
            }

            _snapshot = _impl.ToImmutable();
            _facets = _impl.GetVoronoiFacets().ToDictionary(it => it.Id);
        }


        /// <summary>
        ///  Computes Z height applying z heights from neigboring data
        /// </summary>
        /// <param name="target"> interpolation target</param>
        /// <returns></returns>
        public double Lookup(Vector2 target)
        {
            
            InitSnapshot();

            var vid = _impl.FindNearest(target, out var nearestVertexPt);

            if (!vid.HasValue)
            {
                throw new InvalidOperationException("A point is out of bounds.");
            }

            double minDistance2 = Math.Max(float.Epsilon, (double) MinDistanceThreshold * MinDistanceThreshold);

            double dx = (double) target.X - nearestVertexPt.X;
            double dy = (double) target.Y - nearestVertexPt.Y;

            if (dx*dx + dy*dy < minDistance2)
            {
                return _zHeights[vid.Value];
            }

            _snapshot.RecentEdge = _impl.RecentEdge;

            _neighbors.Clear();


            var newFacet = SubDiv2D_Immutable.SynthesizeFacet(_snapshot, target, _neighbors);
            double newFacetArea = Utils.ComputePolygonArea2(newFacet.Vertices);
            double sumOfOverlapArea = 0.0;

            double z = 0.0;
            var pts = new List<Vector2>();
            foreach (var nf in _neighbors)
            {
                if (!_zHeights.TryGetValue(nf, out var zn))
                {
                    continue;
                }


                pts.Clear();

                _intersectionHelper.Intersect(newFacet.Vertices, _facets[nf].Vertices, pts);

                if (pts.Count > 0)
                {
                    double overlapArea = Utils.ComputePolygonArea2(pts);
                    z += zn * overlapArea / newFacetArea;
                    sumOfOverlapArea += overlapArea;
                }
            }

            double k = newFacetArea / sumOfOverlapArea;
            return z * k;
        }

        public double Lookup(float x, float y) => Lookup(new Vector2(x, y)); 

        public float MinDistanceThreshold { get; set; }

        private readonly HashSet<NodeId> _neighbors = new HashSet<NodeId>();
        private readonly ConvexPolygonIntersectionHelper _intersectionHelper = new ConvexPolygonIntersectionHelper();
        private readonly Dictionary<NodeId, double> _zHeights;
        private Dictionary<NodeId, VoronoiFacet> _facets = new Dictionary<NodeId, VoronoiFacet>();
        private SubDiv2D_Mutable _impl;
        private SubDiv2D_Immutable _snapshot;
    }
}
