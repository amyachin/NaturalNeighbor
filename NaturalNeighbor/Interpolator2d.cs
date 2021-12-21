
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;

namespace NaturalNeighbor
{
    using Internal;

    using System.Threading.Tasks;


    /// <summary>
    /// Spatial interpolation based on Voronoi tesellation (by Robin Sibson). Provides a smoother approximation compared to nearest neighbor.
    /// </summary>
    public class Interpolator2d : ISharedMethods
    {

        /// <summary>
        /// Creates an empty interpolator
        /// </summary>
        public Interpolator2d()
        {
            _nodeEvaluator = DefaultNodeEvaluator;
            _sharedContext = new SearchContext();
            SetMethod(InterpolationMethod.Natural);
        }

        static readonly Func<NodeId, double> DefaultNodeEvaluator = _ => double.NaN;


        private Func<NodeId, double> _nodeEvaluator;

        private void InitFromSource(SubDivision2d source, Func<NodeId, double> nodeEvaluator)
        {
            _nodeEvaluator = nodeEvaluator;
            _impl = source._impl;
            _sharedContext.Clear();

            this.MinValue = source.MinValue;
            this.MaxValue = source.MaxValue;
            this.NumberOfSamples = source.NumberOfSamples;
        }


        /// <summary>
        /// Creates an empty interpolator with the specified bounding box
        /// </summary>
        /// <param name="minValue">The bottom left corner of the bounding box</param>
        /// <param name="maxValue">The top right corner of the bounding box</param>
        public Interpolator2d(Vector2 minValue, Vector2 maxValue): this()  
        {
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
        /// <remarks>Points outside of the new bounding box are deleted</remarks>
        public void SetBounds(Vector2 minValue, Vector2 maxValue)
        {
            if (_impl == null)
            {
                this.MinValue = minValue;
                this.MaxValue = maxValue;
                return;
            }


            // Reconstruct triangulation graph excluding points outside of the new boundaries
            var newBounds = new Bounds(minValue, maxValue);
            var newImpl = new SubDiv2D_Mutable(newBounds);
            var map = new Dictionary<NodeId, double>();

            _sharedContext.Clear();


            foreach (var nodeId in _impl.GetNodes())
            {
                var pt = _impl[nodeId];
                if (newBounds.Contains(pt))
                {
                    var z = _nodeEvaluator(nodeId);
                    if (!double.IsNaN(z))
                    {
                        var newId = newImpl.Insert(pt, _sharedContext);
                        map.Add(newId, z);
                    }
                }

            }

            MinValue = minValue;
            MaxValue = maxValue;

            _impl = newImpl;
            _nodeEvaluator = CreateNodeEvaluator(map);
            NumberOfSamples = map.Count;
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

            var bounds = Utils.CalculateBounds(points, (float)margin, MinValue, MaxValue);
            GenerateCore(points, heights, bounds, checkBounds: false);
        }

        private void GenerateCore(Vector2[] points, double[] heights, Bounds bounds, bool checkBounds)
        {

            if (points.Length != heights.Length)
            {
                throw new ArgumentOutOfRangeException("Point array length does not match the heights array length.");
            }

            if (checkBounds)
            {
                for (int i = 0; i < points.Length; ++i)
                {
                    if (!bounds.Contains(points[i]))
                    {
                        throw new ArgumentOutOfRangeException($"points[{i}] is out of bounds.");
                    }
                }
            }


            // Initialize 
            var map = new Dictionary<NodeId, double>(heights.Length); 

            _voronoiReady = false;
            _impl = new SubDiv2D_Mutable(bounds);
            _nodeEvaluator = CreateNodeEvaluator(map);


            // Generate delaunay graph for the specified points
            for (int i = 0; i < points.Length; ++i)
            {
                var id = _impl.Insert(points[i], _sharedContext);
                if (_sharedContext.Vertex == 0)
                {
                    map.Add(id, heights[i]);
                }
            }

            MinValue = bounds.MinValue;
            MaxValue = bounds.MaxValue;
            NumberOfSamples = map.Count;

        }

        static Func<NodeId, double> CreateNodeEvaluator(IDictionary<NodeId, double> map) 
        {
            return id => map.TryGetValue(id, out var z) ? z : double.NaN;
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

        /// <summary>
        /// Creates an initialized interpolator
        /// </summary>
        /// <param name="source">Source subdivision</param>
        /// <param name="nodeEvaluator">node evaluator</param>
        /// <returns>interpolator instance</returns>
        public static Interpolator2d Create(SubDivision2d source, Func<NodeId, double> nodeEvaluator)
        {
            if (source == null)
            {
                throw new ArgumentNullException(nameof(source));
            }

            if (nodeEvaluator == null)
            {
                throw new ArgumentNullException(nameof(nodeEvaluator));
            }

            var interpolator = new Interpolator2d();
            interpolator.InitFromSource(source, nodeEvaluator);
            return interpolator;
        }

        private void EnsureVoronoi()
        {
            if (_voronoiReady == false)
            {
                lock (_syncRoot)
                {
                    if (_voronoiReady)
                    {
                        return;
                    }

                    _impl.CalcVoronoi();
                    _voronoiReady = true;
                }
            }
        }

        private bool _voronoiReady;

        private readonly object _syncRoot = new object();

        /// <summary>
        ///  Computes interpolated Z value using the current <see cref="Method"/>
        /// </summary>
        /// <param name="target">A point on XY plane</param>
        /// <returns>interpolated Z value</returns>
        public double Lookup(Vector2 target)
        {
            CheckInitialized();
            return _evaluator.Evaluate(target.X, target.Y);
        }


        /// <summary>
        /// Computes interpolated Z values for the specified locations
        /// </summary>
        /// <param name="points">XY coordinates for interpolation</param>
        /// <param name="values">Array recieving the interpolation results.  Must be with the same length as points</param>
        /// <param name="parallelOptions">parallel options, see <see cref="ParallelOptions"/> for details</param>
        public void LookupRange(Vector2[] points, double[] values, ParallelOptions parallelOptions)
        {
            if (points == null)
            {
                throw new ArgumentNullException(nameof(points));
            }

            if (values == null)
            {
                throw new ArgumentNullException(nameof(values));
            }

            if (points.Length != values.Length)
            {
                throw new ArgumentException("Points and values array lengths dont match.");
            }

            CheckInitialized();

            if (Method == InterpolationMethod.Nearest)
            {
                EnsureVoronoi();
            }

            Parallel.For
            (
                0,
                points.Length,
                parallelOptions: parallelOptions,
                localInit: () => _evaluator.Clone(),
                localFinally: _ => { },
                body: (index, state, evaluator) =>
                {
                    var pt = points[index];
                    values[index] = evaluator.Evaluate(pt.X, pt.Y);
                    return evaluator;
                }
           ); 
        }

        /// <summary>
        /// Computes interpolated Z values for the specified locations
        /// </summary>
        /// <param name="points"></param>
        /// <param name="values"></param>
        /// <remarks>Single thread execution. For parallel execution use <see cref="LookupRange(Vector2[], double[], ParallelOptions)"/> </remarks>
        public void LookupRange(Vector2[] points, double[] values) 
        {
            LookupRange(points, values, new ParallelOptions { MaxDegreeOfParallelism = 1 });
        }


        /// <summary>
        /// Computes interpolated Z values for the specified locations
        /// </summary>
        /// <param name="x">X coordinates for interpolation</param>
        /// <param name="y">Y coordinates for interpolation</param>
        /// <param name="values">Array recieving the interpolation results.  Must be with the same length as x and y </param>
        /// <param name="parallelOptions">parallel options, see <see cref="ParallelOptions"/> for details/></param>
        public void LookupRange(float[] x, float[] y, double[] values, ParallelOptions parallelOptions)
        {

            CheckInitialized();

            if (Method == InterpolationMethod.Nearest)
            {
                EnsureVoronoi();
            }

            Parallel.For
            (
              0,
              x.Length,
              parallelOptions: parallelOptions,
              localInit: () => _evaluator.Clone(),
              localFinally: _ => { },
              body: (index, state, evaluator) =>
              {
                  values[index] = evaluator.Evaluate(x[index], y[index]);
                  return evaluator;
              }
            );
        }


        /// <summary>
        /// Computes interpolated Z values for the specified locations
        /// </summary>
        /// <param name="x">X coordinates for interpolation</param>
        /// <param name="y">Y coordinates for interpolation</param>
        /// <param name="values">Array recieving the interpolation results.  Must be with the same length as x and y </param>
        /// <remarks>Single thread execution. For parallel execution use <see cref="LookupRange(float[], float[], double[], ParallelOptions)"/> </remarks>
        public void LookupRange(float[] x, float[] y, double[] values) 
        {
            LookupRange(x, y, values, new ParallelOptions { MaxDegreeOfParallelism = 1 });
        }


        internal List<int> CreateReferenceEnvelope(double x, double y)
        {
            CheckInitialized();

            SubDiv2D_Immutable immutable = _impl.ToImmutable();
            return SubDiv2D_Immutable.CreateEnvelope(immutable, _sharedContext, new Vector2((float) x, (float) y));
        }

        internal List<int> CreateEnvelope(double x, double y)
        {
            CheckInitialized();

            var locType = _impl.Locate(new Vector2((float) x, (float) y), _sharedContext);

            if (locType == PointLocationType.Edge || locType == PointLocationType.Inside)
            {
                return _impl.GetBowyerWatsonEnvelope(x, y, _sharedContext.Edge);
            }
            else
            {
                return null;
            }
        }

        private void CheckInitialized()
        {
            if (_impl == null)
            {
                throw new InvalidOperationException("Iterator is not initialized.");
            }
        }

        internal (NodeId, Vector2) GetEdgeOrigin(int edge)
        {
            var nodeID = _impl.EdgeOrigin(edge);
            return (nodeID, _impl[nodeID]);
        }

        internal (NodeId, Vector2) GetEdgeDestination(int edge)
        {
            var nodeID = _impl.EdgeDestination(edge);
            return (nodeID, _impl[nodeID]);
        }


        bool IsNearVertex(Vector2 target, Vector2 nearestVertexPt)
        {
            double minDistance2 = Math.Max(float.Epsilon, (double) MinDistanceThreshold * MinDistanceThreshold);
            double dx = (double) target.X - nearestVertexPt.X;
            double dy = (double) target.Y - nearestVertexPt.Y;
            return dx * dx + dy * dy < minDistance2;
        }

        double ISharedMethods.LookupLinear(double x, double y, SearchContext context)
        {
            var target = new Vector2((float) x, (float) y);

            var locType = _impl.Locate(target, context);

            switch (locType)
            {
                case PointLocationType.Vertex:
                    return _nodeEvaluator(new NodeId(context.Vertex));

                case PointLocationType.Error:
                case PointLocationType.OutsideRect:
                    return double.NaN;
            }

            var edge = context.Edge;

            System.Diagnostics.Debug.Assert(edge != 0);

            (var triangle, var n1, var n2, var n3) = _impl.GetTriangle(edge);

#if DEBUG
            if (!Utils.IsPointInTriangle(triangle, target))
            {
                System.Diagnostics.Trace.WriteLine($"Point not in the triangle {target}");
            }
#endif

            var points = new List<Vector3>(3);

            if ( !n1.IsSentinel)
            {
                double z = _nodeEvaluator(n1);

                if (!double.IsNaN(z))
                {
                    points.Add(new Vector3(triangle.P1, (float) z));
                }
            }

            if (!n2.IsSentinel)
            {
                double z = _nodeEvaluator(n2);
                if (!double.IsNaN(z))
                {
                    points.Add(new Vector3(triangle.P2, (float) z));
                }
            }

            if (!n3.IsSentinel)
            {
                double z = _nodeEvaluator(n3);
                if (!double.IsNaN(z))
                {
                    points.Add(new Vector3(triangle.P3, (float) z));
                }
            }

            return Utils.Lerp(points, target); 
        }

        double ISharedMethods.LookupNearest(double x, double y, SearchContext context)
        {
            EnsureVoronoi();

            var vid = _impl.FindNearest(new Vector2((float) x, (float) y), context, out var _);
            return vid.HasValue ? _nodeEvaluator(vid.Value) : double.NaN;
        }

        double ISharedMethods.LookupNatural(double x, double y, SearchContext context)
        {

            var target = new Vector2((float) x, (float) y);

            var locationType = _impl.Locate(target, context);

            if (locationType == PointLocationType.Error || locationType == PointLocationType.OutsideRect)
            {
                return double.NaN;
            }

            if (locationType == PointLocationType.Vertex)
            {
                return _nodeEvaluator(new NodeId(context.Vertex));
            }

            (var triangle, var n1, var n2, var n3) =  _impl.GetTriangle(context.Edge);

            if (!n1.IsSentinel && IsNearVertex(target, triangle.P1)) 
            {
                return _nodeEvaluator(n1);
            }

            if (!n2.IsSentinel && IsNearVertex(target, triangle.P2)) 
            {
                return _nodeEvaluator(n2);
            }

            if (!n3.IsSentinel && IsNearVertex(target, triangle.P3))
            {
                return _nodeEvaluator(n3);
            }


            // ------------------------------------------------------
            // The fundamental idea of natural neighbor interpolation is
            // based on measuring how the local geometry of a Voronoi
            // Diagram would change if a new vertex were inserted.
            // (recall that the Voronoi is the dual of a Delaunay Triangulation).
            // Thus the NNI interpolation has common element with an
            // insertion into a TIN.  In writing the code below, I have attempted
            // to preserve similarities with the IncrementalTIN insertion logic
            // where appropriate.
            //
            // Step 1 -----------------------------------------------------
            // Create an array of edges that would connect to the radials
            // from an inserted vertex if it were added at coordinates (x,y).
            // This array happens to describe a Thiessen Polygon around the
            // inserted vertex.


            var envelope = _impl.GetBowyerWatsonEnvelope(x, y, context.Edge);


            if (envelope == null || envelope.Count < 3)
            {
                return double.NaN;
            }

            // The envelope contains a series of edges definining the cavity
            double[] w = _impl.GetBarycentricCoordinates(envelope, x, y);
            
            if (w == null)
            {
                // the coordinate is on the perimeter, no Barycentric coordinates
                // are available.
                return Double.NaN;
            }

            double zSum = 0;
            for (int i = 0; i < envelope.Count; ++i)
            {

                // Skip non-contributing nodes (such as sentinels)
                if (w[i] == 0.0)
                {
                    continue;
                }

                var org = _impl.EdgeOrigin(envelope[i]);
                double z = _nodeEvaluator(org);
                zSum += w[i] * z;
            }

            return zSum;

        }


        /// <summary>
        ///  Computes interpolated Z value using the current <see cref="Method"/>
        /// </summary>
        /// <param name="x">x position</param>
        /// <param name="y">y position </param>
        /// <returns>interpolated Z value</returns>
        public double Lookup(float x, float y)
        {
            return _evaluator.Evaluate(x, y);
        }

        /// <summary>
        ///  Computes interpolated Z value using the current <see cref="Method"/>
        /// </summary>
        /// <param name="x">x position</param>
        /// <param name="y">y position </param>
        /// <returns>interpolated Z value</returns>
        public double Lookup(double x, double y)
        {
            return _evaluator.Evaluate(x, y);
        }


        /// <summary>
        ///  Minimal distance between two distinct points on XY plane
        /// </summary>
        public float MinDistanceThreshold { get; set; }


        /// <summary>
        /// Number of samples used by the interpolator
        /// </summary>
        public int NumberOfSamples { get; private set; }


        /// <summary>
        /// Interpolation method 
        /// </summary>
        public InterpolationMethod Method 
        {
            get 
            { 
                return _method; 
            }
            set 
            {
                if (value != _method)
                {
                    SetMethod(value);
                }
            }

        }


        private void SetMethod(InterpolationMethod value)
        {
            switch (value)
            {
                case InterpolationMethod.Linear:
                    _evaluator = new PiecewiseLinearEvaluator(this, _sharedContext);
                    break;

                case InterpolationMethod.Natural:
                    _evaluator = new NaturalNeighborEvaluator(this, _sharedContext);
                    break;

                case InterpolationMethod.Nearest:
                    _evaluator = new NearestNeighborEvaluator(this, _sharedContext);
                    break;

                default:
                    throw new ArgumentOutOfRangeException(nameof(value));
            }
                
            _method = value;
        }

        private SubDiv2D_Mutable _impl;
        private InterpolationMethod _method;
        private InterpolationEvaluator _evaluator;
        private SearchContext _sharedContext;
    }

}
