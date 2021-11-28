using System;
using System.Collections.Generic;
using System.Numerics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NaturalNeighbor.Internal
{
    internal class ConvexPolygonIntersectionHelper
    {
        public ConvexPolygonIntersectionHelper()
        {
            _events = new PriorityQueue<SweepEvent>(comparer);
            _activeEdges = new int[4];
            _clippingEdges = new (Vector2, Vector2)?[4];

            DistanceEpsilon = float.Epsilon;
        }


        public bool RemoveDuplicates { get; set; }

        public double DistanceEpsilon { get; set; }


        public void Intersect(IReadOnlyList<Vector2> poly1, IReadOnlyList<Vector2> poly2, List<Vector2> result)
        {
            int startIndex = result.Count;

            Reset();

            var top1 = FindTopCorner(poly1);

            int leftEdges = EnqueueEndpoints(poly1, top1, C1_LEFT, true, poly1.Count);
            EnqueueEndpoints(poly1, top1, C1_RIGHT, false, poly1.Count - leftEdges);

            var top2 = FindTopCorner(poly2);

            leftEdges = EnqueueEndpoints(poly2, top2, C2_LEFT, true, poly2.Count);
            EnqueueEndpoints(poly2, top2, C2_RIGHT, false, poly2.Count - leftEdges);

            while (!_events.IsEmpty)
            {
                var e = _events.Dequeue();

                switch (e.EventType)
                {
                    case EventType.StartPoint:
                        HandleStartPoint(e, poly1, poly2, result);
                        break;

                    case EventType.Endpoint:
                        HandleEndpoint(e, result);
                        break;

                    case EventType.Intersection:
                        HandleIntersection((IntersectionEvent)e, poly1, poly2, result);
                        break;
                }
            }

            int count = result.Count - startIndex;

            if (count > 0)
            {
                var centroid = Utils.GetCentroid(startIndex, count, result);
                var sorter = new CoordinateSorter(new Vector2((float)centroid.Item1, (float)centroid.Item2));
                result.Sort(startIndex, count, sorter);

                if (RemoveDuplicates)
                {
                    double epsilon2 = Math.Max(DistanceEpsilon * DistanceEpsilon, float.Epsilon);

                    Vector2[] temp;
                    if (startIndex == 0)
                    {
                        temp = result.ToArray();
                        result.Clear();
                    }
                    else
                    {
                        temp = new Vector2[count];
                        result.CopyTo(startIndex, temp, 0, count);
                        result.RemoveRange(startIndex, count);
                    }

                    result.Add(temp[0]);

                    Vector2 prev = temp[0];
                    for (int i = 1; i < temp.Length - 1; ++i)
                    {
                        var curr = temp[i];
                        if ((curr - prev).LengthSquared() > epsilon2)
                        {
                            result.Add(curr);
                            prev = curr;
                        }
                    }

                    var last = temp[temp.Length - 1];
                    if ((last - prev).LengthSquared() > epsilon2 &&
                        (last - temp[0]).LengthSquared() > epsilon2)
                    {
                        result.Add(last);
                    }

                }
            }
        }


        private void AddPointToResult(List<Vector2> results, Vector2 point)
        {
            if (DistanceEpsilon > 0.0)
            {
                double epsilon2 = Math.Max(DistanceEpsilon * DistanceEpsilon, float.Epsilon);
                
                int count = results.Count;

                if (count > 0 && (results[count - 1] - point).LengthSquared() < epsilon2)
                {
                    return;
                }
            }

            results.Add(point);
        }

        private void HandleIntersection(IntersectionEvent e, IReadOnlyList<Vector2> poly1, IReadOnlyList<Vector2> poly2, List<Vector2> results)
        {
            bool edgeIsLeft = IsLeft(e.EdgeType);
            bool otherEdgeIsLeft = IsLeft(e.OtherEdgeType);

            GetEdgePoints(Polygon(e.EdgeType, poly1, poly2), e.Edge, edgeIsLeft, out var p0, out var p1);
            GetEdgePoints(Polygon(e.OtherEdgeType, poly1, poly2), e.OtherEdge, otherEdgeIsLeft, out var q0, out var q1);

            _clippingEdges[e.EdgeType] = (p0, p1);
            _clippingEdges[e.OtherEdgeType] = (q0, q1);

            if (e.IntersectionType != SegmentIntersectionType.Overlap)
            {
                AddPointToResult(results, e.Pt);
            }
        }

        private void HandleStartPoint(SweepEvent e, IReadOnlyList<Vector2> poly1, IReadOnlyList<Vector2> poly2, List<Vector2> results)
        {
            // Look for intersections with edges of the second polygon
            int otherLeft = GetOtherPolygonEdge(e.EdgeType, true);
            int otherRight = GetOtherPolygonEdge(e.EdgeType, false);

            if (_activeEdges[otherLeft] != -1)
            {
                CheckIntersections(e.Edge, e.EdgeType, _activeEdges[otherLeft], otherLeft, poly1, poly2);
            }

            if (_activeEdges[otherRight] != -1)
            {
                CheckIntersections(e.Edge, e.EdgeType, _activeEdges[otherRight], otherRight, poly1, poly2);
            }

            if (!_clippingEdges[e.EdgeType].HasValue)
            {
                // It is one of the two topmost edges for poligon 

                // Render the edge start if it is within the clipping regions of both polygons
                if (TestClippingBounds(e.EdgeType, e.Pt))
                {
                    AddPointToResult(results, e.Pt);
                }
            }

            _activeEdges[e.EdgeType] = e.Edge;

            // Update clipping edges of this polygon
            GetEdgePoints(Polygon(e.EdgeType, poly1, poly2), e.Edge, IsLeft(e.EdgeType), out var p0, out var p1);
            _clippingEdges[e.EdgeType] = (p0, p1);
        }

        private void HandleEndpoint(SweepEvent e, List<Vector2> results)
        {

            if (_activeEdges[e.EdgeType] == e.Edge)
            {
                _activeEdges[e.EdgeType] = -1;
            }

            // Render endpoint if it is within the clipping regions of both polygons
            if (TestClippingBounds(e.EdgeType, e.Pt))
            {
                AddPointToResult(results, e.Pt);
            }
        }

        private bool TestClippingBounds(int edgeType,  Vector2 pt) 
        {
            var otherLeft = _clippingEdges[GetOtherPolygonEdge(edgeType, true)];
            var otherRight = _clippingEdges[GetOtherPolygonEdge(edgeType, false)];

            if (!otherLeft.HasValue || !otherRight.HasValue) 
            {
                return false;
            }

            if ( Utils.IsLeft(otherLeft.Value.Item1, otherLeft.Value.Item2, pt) <= 0 ||
                 Utils.IsLeft(otherRight.Value.Item1, otherRight.Value.Item2, pt) >= 0)
            {
                return false;
            }

            float minX = Math.Min(otherLeft.Value.Item1.X, otherLeft.Value.Item2.X);
            float maxX = Math.Max(otherRight.Value.Item1.X, otherRight.Value.Item2.X);
            return pt.X > minX && pt.X < maxX;
        }

        private static bool IsLeft(int edgeType) 
        {
            return edgeType == C1_LEFT || edgeType == C2_LEFT;
        }

        private static int GetOtherPolygonEdge(int edgeType, bool left)
        {
            if (edgeType == C1_LEFT || edgeType == C1_RIGHT)
            {
                return left ? C2_LEFT : C2_RIGHT;
            }
            else
            {
                return left ? C1_LEFT : C1_RIGHT;
            }
        }


        private static IReadOnlyList<Vector2> Polygon(int edgeType, IReadOnlyList<Vector2> c1, IReadOnlyList<Vector2> c2)
        {
            return edgeType == C1_LEFT || edgeType == C1_RIGHT ? c1 : c2;
        }


        private static int FindTopCorner(IReadOnlyList<Vector2> polygon)
        {
            int top = -1;

            float maxPos = float.MinValue;

            for (int i = 0; i < polygon.Count; ++i)
            {
                var pt = polygon[i];
                if (pt.Y > maxPos)
                {
                    maxPos = pt.Y;
                    top = i;
                }
            }

            return top;
        }

        private int EnqueueEndpoints(IReadOnlyList<Vector2> polygon, int topIndex, int edgeType, bool left, int maxEdges)
        {
            int prevIdx = topIndex;
            int count = polygon.Count;

            int increment = left ? 1 : -1;

            int sequenceNumber = 1;
            int edges = 0;


            for (int i = 0; i < maxEdges; i++)
            {
                var idx = GetIndex(prevIdx + increment, count);

                var p0 = polygon[prevIdx];
                var p1 = polygon[idx];

                if (p1.Y > p0.Y)
                {
                    break;
                }

                var startEvent = new SweepEvent(p0, EventType.StartPoint) { Edge = idx, EdgeType = edgeType, SequenceNumber = sequenceNumber++ };
                var endEvent = new SweepEvent(p1, EventType.Endpoint) { Edge = idx, EdgeType = edgeType, SequenceNumber = sequenceNumber++ };

                if (p1.Y.CompareTo(p0.Y) == 0)
                {
                    // Needed for handling degenerative cases ?
                    startEvent.IsHorizontal = true;
                    endEvent.IsHorizontal = true;
                }
                
                _events.Enqueue(startEvent);
                _events.Enqueue(endEvent);

                prevIdx = idx;
                edges++;
            }
            

            return edges;
        }

        private void CheckIntersections(
            int edgeIndex, 
            int edgeType, 
            int otherEdgeIndex, 
            int otherEdgeType, 
            IReadOnlyList<Vector2> c1, 
            IReadOnlyList<Vector2> c2)
        {
            GetEdgePoints(Polygon(edgeType, c1, c2), edgeIndex, IsLeft(edgeType), out var a, out var b);
            GetEdgePoints(Polygon(otherEdgeType, c1, c2), otherEdgeIndex, IsLeft(otherEdgeType), out var c, out var d);

            var test = Utils.CheckSegmentIntersection(a, b, c, d);

                switch (test)
            {
                case SegmentIntersectionType.Inner:
                case SegmentIntersectionType.Endpoint:

                    var pt = Utils.IntersectLines(a, b, c, d);
                    _events.Enqueue(new IntersectionEvent(pt)
                    {
                        Edge = edgeIndex,
                        EdgeType = edgeType,
                        IntersectionType = test,
                        OtherEdge = otherEdgeIndex,
                        OtherEdgeType = otherEdgeType
                    });
                    break;

                case SegmentIntersectionType.Overlap:

                    // Pick the point with max Y from the overlapping section
                    (var p1, var p2) = Utils.OverlapSegments(a, b, c, d);
                    var ppt = p1.Y > p2.Y ? p1 : p2;

                    _events.Enqueue(new IntersectionEvent(ppt)
                    {
                        Edge = edgeIndex,
                        EdgeType = edgeType,
                        IntersectionType = test,
                        OtherEdge = otherEdgeIndex,
                        OtherEdgeType = otherEdgeType
                    });

                    break;
            }
        }

        private static void GetEdgePoints(IReadOnlyList<Vector2> polygon, int edgeIndex, bool left, out Vector2 p0, out Vector2 p1)
        {
            int increment = left ? -1 : 1;
            int startIndex = GetIndex(edgeIndex + increment, polygon.Count);

            p0 = polygon[startIndex];
            p1 = polygon[edgeIndex];
        }

        private static int GetIndex(int idx, int count)
        {
            int result = idx;

            if (result >= count)
            {
                result -= count;
            }

            if (result < 0)
            {
                result += count;
            }

            return result;
        }

        private void Reset()
        {
            _events.Clear();

            for (int i = 0; i < _activeEdges.Length; ++i)
            {
                _activeEdges[i] = -1;
            }

            for (int i = 0; i < _clippingEdges.Length; ++i)
            {
                _clippingEdges[i] = null;
            }
        }

        private readonly PriorityQueue<SweepEvent> _events;
        private readonly int[] _activeEdges;
        private readonly (Vector2, Vector2)?[] _clippingEdges;


        private static readonly SweeplinePriorityComparer comparer = new SweeplinePriorityComparer();

        const int C1_LEFT = 0;
        const int C1_RIGHT = 1;
        const int C2_LEFT = 2;
        const int C2_RIGHT = 3;

    }

    internal enum EventType
    {
        StartPoint,
        Endpoint,
        Intersection
    }

    class CoordinateSorter : IComparer<Vector2>
    {
        public CoordinateSorter(Vector2 center)
        {
            _center = center;
        }

        readonly Vector2 _center;

        public int Compare(Vector2 x, Vector2 y)
        {
            double xx = Utils.AngleBetween(1, 0, (double)x.X - _center.X, (double)x.Y - _center.Y, true);
            double yy = Utils.AngleBetween(1, 0, (double)y.X - _center.X , (double)y.Y- _center.Y, true);
            return xx.CompareTo(yy);
        }
    }

    class SweeplinePriorityComparer : IComparer<SweepEvent>
    {
        public int Compare(SweepEvent x, SweepEvent y)
        {
            double vx = x.Pt.Y;
            double vy = y.Pt.Y;
            int result = vy.CompareTo(vx);

            if (result == 0)
            {
                // A tie breaker is needed for deterministic behavior of horizontal edges (e.g. end point follows start point)
                // NOTE: As side-effect intersection points have higher priority because of their sequence number (0)
                result = x.SequenceNumber - y.SequenceNumber;
            }

            return result;
        }
    }


    class SweepEvent
    {
        public SweepEvent(Vector2 pt, EventType eventType)
        {
            this.Pt = pt;
            this.EventType = eventType;
        }

        public Vector2 Pt { get; }

        public EventType EventType { get; }

        public int Edge { get; set; }

        public int EdgeType { get; set; }

        public bool IsHorizontal { get; set; }

        public int SequenceNumber { get; set; }

    }

    class IntersectionEvent : SweepEvent
    {
        public IntersectionEvent(Vector2 pt) : base(pt, EventType.Intersection)
        {

        }
        public int OtherEdge { get; set; }

        public int OtherEdgeType { get; set; }

        public SegmentIntersectionType IntersectionType { get; set; }
    }


}
