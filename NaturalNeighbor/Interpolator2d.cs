
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

        /// <summary>
        /// Creates an empty interpolator
        /// </summary>
        public Interpolator2d()
        {
            _zHeights = new Dictionary<NodeId, double>();
            Method = InterpolationMethod.Natural;
        }


        /// <summary>
        /// Creates an empty interpolator with a specified bounding box
        /// </summary>
        /// <param name="minValue">New bottom left corner of the bounding box</param>
        /// <param name="maxValue">New top right corner of the bounding box</param>
        public Interpolator2d(Vector2 minValue, Vector2 maxValue)  
        {
            _zHeights = new Dictionary<NodeId, double>();
            Method = InterpolationMethod.Natural;
            SetBounds(minValue, maxValue);
        }

        /// <summary>
        /// Bottom-left corner of the bounding box
        /// </summary>
        public Vector2? MinValue { get; private set; }

        /// <summary>
        /// Top-right corner of the bounding box
        /// </summary>
        public Vector2? MaxValue { get; private set; }

        /// <summary>
        /// Sets explicit interpolation boundaries reconstructing triangulation data if needed
        /// </summary>
        /// <param name="minValue">New bottom left corner of the bounding box</param>
        /// <param name="maxValue">New top right corner of the bounding box</param>
        public void SetBounds(Vector2 minValue, Vector2 maxValue)
        {
            if (_impl == null)
            {
                this.MinValue = minValue;
                this.MaxValue = maxValue;
                return;
            }


            // Reconstruct existing tesselation using new boundaries

            var bounds = _impl.Bounds;

            float minX = Math.Min(minValue.X, bounds.MinValue.X);
            float minY = Math.Min(minValue.Y, bounds.MinValue.Y);

            float maxX = Math.Max(maxValue.X, bounds.MaxValue.X);
            float maxY = Math.Max(maxValue.Y, bounds.MaxValue.Y);

            bounds = new Bounds(minX, maxX, minY, maxY);

            var newImpl = new SubDiv2D_Mutable(bounds);
            var newZHeights = new Dictionary<NodeId, double>();

            foreach (var pair in _zHeights)
            {
                var pt = _impl[pair.Key];
                var newId = newImpl.Insert(pt);
                newZHeights.Add(newId, pair.Value);
            }

            _impl = newImpl;
            _zHeights = newZHeights;
            _snapshot = null;
            _facets = null;
        }



        /// <summary>
        /// Initializes interpolator with scattered 2D data
        /// </summary>
        /// <param name="points">coordinates on XY plane</param>
        /// <param name="heights">Z values </param>

        public void Generate(Vector2[] points, double[] heights)
        {
            Generate(points, heights, margin: float.Epsilon * 10);
        }

        /// <summary>
        /// Initializes interpolator with scattered 2D data
        /// </summary>
        /// <param name="points">coordinates on XY plane</param>
        /// <param name="heights">Z values</param>
        /// <param name="margin">extrapolation margin</param>
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

            var bounds = Utils.CalculateBounds(points, margin, MinValue, MaxValue);
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

            MinValue = bounds.MinValue;
            MaxValue = bounds.MaxValue;
        }


        /// <summary>
        /// Creates an initialized interpolator
        /// </summary>
        /// <param name="points">coordinates on XY plane</param>
        /// <param name="heights">Z values</param>
        /// <param name="margin">extrapolation margin</param>
        /// <returns>interpolator instance</returns>
        public static Interpolator2d Create(Vector2[] points, double[] heights, double margin)
        {
            var interpolator = new Interpolator2d();
            interpolator.Generate(points, heights, margin);
            return interpolator;
        }


        /// <summary>
        /// Creates an initialized interpolator
        /// </summary>
        /// <param name="points">XYZ points</param>
        /// <returns>interpolator instance</returns>
        public static Interpolator2d Create(Vector3[] points) 
        {
            return Create(points, margin: 0);
        }

        /// <summary>
        /// Creates an initialized interpolator
        /// </summary>
        /// <param name="points">XYZ points</param>
        /// <param name="margin">extrapolation margin</param>
        /// <returns>interpolator instance</returns>
        public static Interpolator2d Create(Vector3[] points, double margin) 
        {
            if (points == null) 
            {
                throw new ArgumentNullException(nameof(points));
            }

            Vector2[] xypoints = new Vector2[points.Length];
            double[] heights = new double[points.Length]; 

            for(int i = 0; i < xypoints.Length; ++i)
            {
                var pt = points[i];
                xypoints[i] = new Vector2(pt.X, pt.Y);
                heights[i] = pt.Z;
            }
            var interpolator = new Interpolator2d();
            interpolator.Generate(xypoints, heights, margin);
            return interpolator;
        }


        /// <summary>
        /// Creates an initialized interpolator
        /// </summary>
        /// <param name="points">XYZ points</param>
        /// <param name="minValue">Bottom left corner of the bounding box</param>
        /// <param name="maxValue">Top right corner of the bounding box</param>
        /// <returns>interpolator instance</returns>
        public static Interpolator2d Create(Vector3[] points, Vector2 minValue, Vector2 maxValue)
        {
            if (points == null)
            {
                throw new ArgumentNullException(nameof(points));
            }

            Vector2[] xypoints = new Vector2[points.Length];
            double[] heights = new double[points.Length];

            for (int i = 0; i < xypoints.Length; ++i)
            {
                var pt = points[i];
                xypoints[i] = new Vector2(pt.X, pt.Y);
                heights[i] = pt.Z;
            }
            var interpolator = new Interpolator2d(minValue, maxValue);
            interpolator.Generate(xypoints, heights);
            return interpolator;
        }


        /// <summary>
        /// Creates an initialized interpolator
        /// </summary>
        /// <param name="points">Coordinates on XY plane</param>
        /// <param name="heights">Z values</param>
        /// <returns>interpolator instance</returns>
        public static Interpolator2d Create(Vector2[] points, double[] heights)
        {
            var interpolator = new Interpolator2d();
            interpolator.Generate(points, heights);
            return interpolator;
        }


        /// <summary>
        /// Creates an initialized interpolator
        /// </summary>
        /// <param name="points">coordinates on XY plane</param>
        /// <param name="heights">Z values</param>
        /// <param name="minValue">Bottom left corner of the bounding box</param>
        /// <param name="maxValue">Top right corner of the bounding box</param>
        /// <returns>interpolator instance</returns>

        public static Interpolator2d Create(Vector2[] points, double[] heights, Vector2 minValue, Vector2 maxValue)
        {
            var interpolator = new Interpolator2d(minValue, maxValue);
            interpolator.Generate(points, heights);
            return interpolator;
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
        ///  Computes interpolated Z value using the current <see cref="Method"/>
        /// </summary>
        /// <param name="target">A point on XY plane</param>
        /// <returns>interpolated Z value</returns>
        public double Lookup(Vector2 target)
        {

            if (Method == InterpolationMethod.Natural)
            {
                InitSnapshot();
            }

            var vid = _impl.FindNearest(target, out var nearestVertexPt);

            if (!vid.HasValue)
            {
                // Cannot interpolate the point - possibly out of bounds 
                return double.NaN;
            }


            switch (Method)
            {
                case InterpolationMethod.Nearest:
                    return _zHeights.TryGetValue(vid.Value, out var result) ? result : double.NaN;

                case InterpolationMethod.Linear:
                    return LookupLinear(target, vid.Value, nearestVertexPt);

                case InterpolationMethod.Natural:
                    return LookupNatural(target, vid.Value, nearestVertexPt);

                default:
                    throw new InvalidOperationException($"Method not supported: {Method}.");

            }

        }

        bool IsNearVertex(Vector2 target, Vector2 nearestVertexPt)
        {
            double minDistance2 = Math.Max(float.Epsilon, (double) MinDistanceThreshold * MinDistanceThreshold);
            double dx = (double) target.X - nearestVertexPt.X;
            double dy = (double) target.Y - nearestVertexPt.Y;
            return dx * dx + dy * dy < minDistance2;
        }

        private double LookupLinear(Vector2 target, NodeId nearestVertexId, Vector2 nearestVertexPt)
        {

            if (IsNearVertex(target, nearestVertexPt))
            {
                return _zHeights[nearestVertexId];
            }

            (var triangle, var n1, var n2, var n3) = _impl.GetNearestTriangle();

#if DEBUG
            if (!Utils.IsPointInTriangle(triangle, target))
            {
                System.Diagnostics.Trace.WriteLine($"Point not  in triangle {target}");
            }
#endif

            var points = new List<Vector3>(3);
            double z;

            if (_zHeights.TryGetValue(n1, out z))
            {
                points.Add(new Vector3(triangle.P1, (float) z));
            }

            if (_zHeights.TryGetValue(n2, out z))
            {
                points.Add(new Vector3(triangle.P2, (float) z));
            }

            if (_zHeights.TryGetValue(n3, out z))
            {
                points.Add(new Vector3(triangle.P3, (float) z));
            }

            return Utils.Lerp(points, target); 
        }

        private double LookupNatural(Vector2 target, NodeId nearestVertexId, Vector2 nearestVertexPt)
        {
            if (IsNearVertex(target, nearestVertexPt))
            {
                return _zHeights[nearestVertexId];
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


        /// <summary>
        ///  Computes interpolated Z value using the current <see cref="Method"/>
        /// </summary>
        /// <param name="x">x position</param>
        /// <param name="y">y position </param>
        /// <returns>interpolated Z value</returns>
        public double Lookup(float x, float y) => Lookup(new Vector2(x, y));


        /// <summary>
        ///  Minimal distance between two distinct points on XY plane
        /// </summary>
        public float MinDistanceThreshold { get; set; }



        /// <summary>
        /// Interpolation method 
        /// </summary>
        public InterpolationMethod Method { get; set; }

        private readonly HashSet<NodeId> _neighbors = new HashSet<NodeId>();
        private readonly ConvexPolygonIntersectionHelper _intersectionHelper = new ConvexPolygonIntersectionHelper();
        private Dictionary<NodeId, double> _zHeights;
        private Dictionary<NodeId, VoronoiFacet> _facets = new Dictionary<NodeId, VoronoiFacet>();
        private SubDiv2D_Mutable _impl;
        private SubDiv2D_Immutable _snapshot;
    }


}
