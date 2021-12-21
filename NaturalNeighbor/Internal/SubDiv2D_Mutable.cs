using System;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Diagnostics;
using System.Linq;
using System.Numerics;

namespace NaturalNeighbor.Internal
{
    sealed class SubDiv2D_Mutable : SubDiv2D_Base
    {

        public SubDiv2D_Mutable(Bounds bounds) : base(bounds)
        {
            InitDelaunay();
        }

        public IEnumerable<VoronoiFacet> GetVoronoiFacets()
        {
            var sequence = _vertices.Skip(4).Select((it, idx) => new NodeId(idx + 4));
            return GetVoronoiFacets(sequence);
        }

        public IEnumerable<VoronoiFacet> GetVoronoiFacets(IEnumerable<NodeId> vertices)
        {
            CalcVoronoi();

            var buff = ImmutableArray.CreateBuilder<Vector2>();

            foreach (var nodeId in vertices)
            {
                var facet = CreateVoronoiFacet(nodeId, buff);
                yield return facet;
            }
        }

        private VoronoiFacet CreateVoronoiFacet(NodeId nodeId, ImmutableArray<Vector2>.Builder buffer)
        {

            var v = _vertices[(int) nodeId];
            if (v.IsFree || v.IsVirtual)
            {
                return new VoronoiFacet(nodeId, v.pt);
            }

            buffer.Clear();

            int edge = RotateEdge(v.firstEdge, 1);
            var t = edge;

            do
            {

                var pt = _vertices[EdgeOrg(t)].pt;
                t = GetEdge(t, TargetEdgeType.NEXT_AROUND_LEFT);

                int cnt = buffer.Count;
                if (cnt != 0)
                {
                    var prevPt = buffer[cnt - 1];
                    double dx = pt.X - prevPt.X;
                    double dy = pt.Y - prevPt.Y;

                    // Remove duplicate points
                    if (dx * dx + dy * dy <= float.Epsilon)
                    {
                        continue;
                    }

                    // Check convex hull
                    if (cnt > 1 && Utils.IsLeft(buffer[cnt - 2], prevPt, pt) <= 0)
                    {
                        buffer[cnt - 1] = pt;
                        continue;
                    }

                    if (t == edge)
                    {
                        dx = pt.X - buffer[0].X;
                        dy = pt.Y - buffer[0].Y;
                        if (dx * dx + dy * dy < float.Epsilon)
                        {
                            continue;
                        }

                        if (cnt > 1 && Utils.IsLeft(prevPt, pt, buffer[0]) <= 0)
                        {
                            buffer[0] = pt;
                            continue;
                        }
                    }
                }

                buffer.Add(pt);

            } while (t != edge);

            return new VoronoiFacet(nodeId, v.pt, buffer.ToImmutable());
        }

        public IEnumerable<(Vector2, Vector2)> GetEdges()
        {
            for (int i = 4; i < _quadEdges.Count; i++)
            {
                var q = _quadEdges[i];

                if (!q.IsFree && q.pts[0] > 0 && q.pts[2] > 0)
                {
                    Vector2 org = _vertices[q.pts[0]].pt;
                    Vector2 dst = _vertices[q.pts[2]].pt;

                    yield return (org, dst);
                }
            }
        }

        public IEnumerable<Triangle> GetTriangles()
        {
            int total = _quadEdges.Count * 4;
            var edgemask = new bool[total];

            for (var i = 4; i < total; i += 2)
            {
                if (edgemask[i])
                    continue;

                Vector2 a, b, c;
                int edge_a = i;
                EdgeOrg(edge_a, out a);

                if (!Bounds.Contains(a))
                    continue;

                int edge_b = GetEdge(edge_a, TargetEdgeType.NEXT_AROUND_LEFT);
                EdgeOrg(edge_b, out b);

                if (!Bounds.Contains(b))
                    continue;

                int edge_c = GetEdge(edge_b, TargetEdgeType.NEXT_AROUND_LEFT);
                EdgeOrg(edge_c, out c);

                if (!Bounds.Contains(c))
                    continue;

                edgemask[edge_a] = true;
                edgemask[edge_b] = true;
                edgemask[edge_c] = true;
                yield return new Triangle(a, b, c);
            }
        }

        public IEnumerable<NodeId> GetNodes()
        {
            for (int i = 4; i < _vertices.Count; ++i)
            {
                var v = _vertices[i];
                if (v.IsVirtual || v.IsFree)
                {
                    continue;
                }

                yield return new NodeId(i);
            }
        }


        public int GetFirstEdge(NodeId nodeId)
        {
            return _vertices[(int) nodeId].firstEdge;
        }



        public List<int> GetBowyerWatsonEnvelope(double x, double y, int locatorEdge)
        {

            if (locatorEdge == 0)
            {
                return null;
            }

            int v2 = EdgeDst(NextEdge(locatorEdge));

            double h;
            int c = locatorEdge;
            var list = new List<int>();
            var stack = new Stack<int>();

            while (true)
            {
                int n0 = SymEdge(c);
                int n1 = GetEdge(n0, TargetEdgeType.NEXT_AROUND_LEFT);

                // nInCircle++
                EdgeOrg(n0, out var p1);
                EdgeOrg(n1, out var p2);
                EdgeDst(n1, out var p3);


                double a11 = p1.X - x;
                double a21 = p2.X - x;
                double a31 = p3.X - x;

                double a12 = p1.Y - y;
                double a22 = p2.Y - y;
                double a32 = p3.Y - y;

                h = (a11 * a11 + a12 * a12) * (a21 * a32 - a31 * a22)
                    + (a21 * a21 + a22 * a22) * (a31 * a12 - a11 * a32)
                    + (a31 * a31 + a32 * a32) * (a11 * a22 - a21 * a12);


                if (h >= 0)
                {
                    // The vertex is within the circumcircle the associated
                    // triangle.  The Thiessen triangle will extend to include
                    // that triangle and, perhaps, its neighbors.
                    // So continue the search.
                    stack.Push(n0);
                    c = n1;
                }
                else
                {
                    if (list.Count > 0 && list[0] == c)
                    {
                        throw new InvalidOperationException("Infininte loop detected.");
                    }

                    list.Add(c);
                    c = GetEdge(c, TargetEdgeType.NEXT_AROUND_LEFT);

                    if (stack.Count > 0)
                    {
                        var p = stack.Peek();
                        while (c == p)
                        {
                            stack.Pop();
                            c = GetEdge(SymEdge(c), TargetEdgeType.NEXT_AROUND_LEFT);

                            if (stack.Count == 0)
                            {
                                break;
                            }

                            p = stack.Peek();
                        }
                    }

                    if (c == locatorEdge)
                    {
                        break;
                    }
                }
            }

            //DumpEnvelope(list, x, y);

            return list;
        }

        //private void DumpEnvelope(List<int> list, double x, double y)
        //{

        //    System.Text.StringBuilder sb = new System.Text.StringBuilder();
        //    sb.AppendLine($"Enevelope({x}\ty={y})");

        //    for (int i = 0; i < list.Count; ++i)
        //    {
        //        EdgeOrg(list[i], out var pt);
        //        sb.AppendLine($"p[{i}]\t{pt.X}\t{pt.Y}");
        //    }

        //    Trace.Write(sb.ToString());
        //}

        public double[] GetBarycentricCoordinates(List<int> envelope, double x, double y)
        {
            double[] weights = new double[envelope.Count];

            var c0 = default(Circumcircle);
            var c1 = default(Circumcircle);
            var c2 = default(Circumcircle);
            var c3 = default(Circumcircle);

            double wSum = 0;

            for (int i0 = 0, nEdge = envelope.Count; i0 < nEdge; i0++)
            {
                int i1 = (i0 + 1) % nEdge; // next index

                var e0 = envelope[i0];
                var e1 = envelope[i1];

                EdgeOrg(e0, out var a);
                var currNode = EdgeOrg(e1, out var b); // Same as EdgeDst(e0)
                EdgeDst(e1, out var c);


                double ax = a.X - x;
                double ay = a.Y - y;
                double bx = b.X - x;
                double by = b.Y - y;
                double cx = c.X - x;
                double cy = c.Y - y;


                double x0 = (ax + bx) * 0.5;
                double y0 = (ay + by) * 0.5;
                double x1 = (bx + cx) * 0.5;
                double y1 = (by + cy) * 0.5;

                // for the first edge processed, the code needs to initialize values
                // for c0 and c3.  But after that, the code can reuse values from
                // the previous calculation.

                if (i0 == 0)
                {
                    c0 = Utils.ComputeCurcumcircle(ax, ay, bx, by, 0, 0);
                    EdgeDst(GetEdge(e0, TargetEdgeType.NEXT_AROUND_LEFT), out var pt);
                    c3 = Utils.ComputeCurcumcircle(ax, ay, bx, by, pt.X - x, pt.Y - y);
                }
                else
                {
                    c0 = c1;
                }

                c1 = Utils.ComputeCurcumcircle(bx, by, cx, cy, 0, 0);

                // Skip non-contributing nodes from the weight calculation
                if (currNode >= 4)
                {

                    // compute the reduced "component area" of the Theissen polygon
                    // constructed around point B, the second point of edge[i0].
                    double wXY = (x0 * c0.CenterY - c0.CenterX * y0)
                      + (c0.CenterX * c1.CenterY - c1.CenterX * c0.CenterY)
                      + (c1.CenterX * y1 - x1 * c1.CenterY);


                    // compute the full "component area" of the Theissen polygon
                    // constructed around point B, the second point of edge[i0]
                    var n = GetEdge(e0, TargetEdgeType.NEXT_AROUND_LEFT);
                    double wThiessen = x0 * c3.CenterY - c3.CenterX * y0;

                    while (n != e1)
                    {
                        var n1 = SymEdge(n);
                        n = GetEdge(n1, TargetEdgeType.NEXT_AROUND_LEFT);
                        c2 = c3;

                        EdgeOrg(n1, out a);
                        EdgeOrg(n, out b); // Same As EdgeDst(n1))
                        EdgeDst(n, out c);


                        ax = a.X - x;
                        ay = a.Y - y;
                        bx = b.X - x;
                        by = b.Y - y;
                        cx = c.X - x;
                        cy = c.Y - y;

                        c3 = Utils.ComputeCurcumcircle(ax, ay, bx, by, cx, cy);
                        wThiessen += c2.CenterX * c3.CenterY - c3.CenterX * c2.CenterY;
                    }

                    wThiessen += c3.CenterX * y1 - x1 * c3.CenterY;




                    // Compute wDelta, the amount of area that the Theissen polygon
                    // constructed around vertex B would yield to an insertion at
                    // the query point.
                    //    for convenience, both the wXY and wThiessen weights were
                    // computed in a clockwise order, which means they are the
                    // negative of what we need for the weight computation, so
                    // negate them and  -(wTheissen-wXY) becomes wXY-wTheissen
                    // Also, there would normally be a divide by 2 factor from the
                    // shoelace area formula, but that is ommitted because it will
                    // drop out when we unitize the sum of the set of the weights.

                    double wDelta = wXY - wThiessen;
                    wSum += wDelta;
                    weights[i1] = wDelta;
                }
            }

            // Normalize the weights
            for (int i = 0; i < weights.Length; ++i)
            {
                weights[i] /= wSum;
            }

            return weights;
        }

        public NodeId EdgeOrigin(int edge) => new NodeId(EdgeOrg(edge));

        public NodeId EdgeDestination(int edge) => new NodeId(EdgeDst(edge));

        public NodeId? FindNearest(Vector2 pt, SearchContext context, out Vector2 nearestVertexPt)
        {
            if (!_validGeometry)
            {
                CalcVoronoi();
            }


            // Find the closest delaunay triangle (one of it's corners)
            var loc = Locate(pt, context);

            if (loc == PointLocationType.Vertex)
            {
                nearestVertexPt = _vertices[context.Vertex].pt;
                return new NodeId(context.Vertex);
            }

            if (loc != PointLocationType.Edge && loc != PointLocationType.Inside)
            {
                nearestVertexPt = default(Vector2);
                return null;
            }

            int vertex = 0;

            EdgeOrg(context.Edge, out var start);
            Vector2 diff = pt - start;

            // Switch to voronoi space
            int edge = RotateEdge(context.Edge, 1);
            int total = _vertices.Count;

            for (int i = 0; i < total; ++i)
            {
                Vector2 t;

                while (true)
                {
                    var dest = EdgeDst(edge, out t);

                    Debug.Assert((int) dest > 0);

                    if (IsRightOf2(t, start, diff) >= 0)
                        break;

                    edge = GetEdge(edge, TargetEdgeType.NEXT_AROUND_LEFT);
                }

                while (true)
                {
                    var org = EdgeOrg(edge, out t);

                    Debug.Assert((int) org > 0);

                    if (IsRightOf2(t, start, diff) < 0)
                        break;

                    edge = GetEdge(edge, TargetEdgeType.PREV_AROUND_LEFT);
                }


                EdgeDst(edge, out var tempDest);
                EdgeOrg(edge, out t);

                if (IsRightOf2(pt, t, tempDest - t) >= 0)
                {
                    vertex = EdgeOrg(RotateEdge(edge, 3));
                    break;
                }

                edge = SymEdge(edge);

            }

            if (vertex > 0)
            {
                nearestVertexPt = _vertices[vertex].pt;
                return new NodeId(vertex);
            }

            else
            {
                nearestVertexPt = default(Vector2);
                return null;
            }

        }

        public (Triangle, NodeId, NodeId, NodeId) GetTriangle(int edge)
        {
            int edge1 = GetEdge(edge, TargetEdgeType.NEXT_AROUND_LEFT);
            int edge2 = GetEdge(edge1, TargetEdgeType.NEXT_AROUND_LEFT);

            var n1 = EdgeOrg(edge1, out var p1);
            var n2 = EdgeDst(edge1, out var p2);
            var n3 = EdgeDst(edge2, out var p3);

            return (new Triangle(p1, p2, p3), new NodeId(n1), new NodeId(n2), new NodeId(n3));
        }

        public SubDiv2D_Immutable ToImmutable()
        {
            var vertices = this._vertices.Select((it, idx) => new KeyValuePair<int, ImmutableVertexData>(idx, new ImmutableVertexData(it.pt, it.type, it.firstEdge)));
            var quadEdges = this._quadEdges.Select((it, idx) => new KeyValuePair<int, ImmutableQuadEdgeData>(idx, new ImmutableQuadEdgeData(next: ImmutableArray.CreateRange(it.next), pts: ImmutableArray.CreateRange(it.pts))));
            return new SubDiv2D_Immutable(this.Bounds, vertices.ToImmutableDictionary(), quadEdges.ToImmutableDictionary());
        }

        private void ClearVoronoi()
        {
            int i, total = _quadEdges.Count;

            for (i = 0; i < total; i++)
            {
                var pts = _quadEdges[i].pts;
                pts[1] = pts[3] = 0;
            }

            total = _vertices.Count;
            for (i = 0; i < total; i++)
            {
                if (_vertices[i].IsVirtual)
                {
                    DeletePoint(i);
                }
            }

            _validGeometry = false;
        }
        internal void CalcVoronoi()
        {
            if (_validGeometry)
            {
                return;
            }

            ClearVoronoi();


            int total = _quadEdges.Count;

            // loop through all quad-edges, except for the first 4 entries (#1, #2, #3  - boundaries, and #0 is a sentinel node)
            for (int i = 4; i < total; i++)
            {
                QuadEdgeData quadedge = _quadEdges[i];

                if (quadedge.IsFree)
                    continue;

                int edge0 = (i * 4);
                Vector2 org0, dst0, org1, dst1;

                if (quadedge.pts[3] == 0)
                {
                    int edge1 = GetEdge(edge0, TargetEdgeType.NEXT_AROUND_LEFT);
                    int edge2 = GetEdge(edge1, TargetEdgeType.NEXT_AROUND_LEFT);

                    EdgeOrg(edge0, out org0);
                    EdgeDst(edge0, out dst0);
                    EdgeOrg(edge1, out org1);
                    EdgeDst(edge1, out dst1);

                    Vector2 virt_point = Utils.ComputeVoronoiPoint(org0, dst0, org1, dst1);

                    if (Math.Abs(virt_point.X) < float.MaxValue * 0.5 &&
                       Math.Abs(virt_point.Y) < float.MaxValue * 0.5)
                    {
                        quadedge.pts[3] = _quadEdges[edge1 >> 2].pts[3 - (edge1 & 2)] =
                        _quadEdges[edge2 >> 2].pts[3 - (edge2 & 2)] = NewPoint(virt_point, true);
                    }
                }

                if (quadedge.pts[1] == 0)
                {
                    int edge1 = GetEdge(edge0, TargetEdgeType.NEXT_AROUND_RIGHT);
                    int edge2 = GetEdge(edge1, TargetEdgeType.NEXT_AROUND_RIGHT);

                    EdgeOrg(edge0, out org0);
                    EdgeDst(edge0, out dst0);
                    EdgeOrg(edge1, out org1);
                    EdgeDst(edge1, out dst1);

                    var virt_point = Utils.ComputeVoronoiPoint(org0, dst0, org1, dst1);

                    if (Math.Abs(virt_point.X) < float.MaxValue * 0.5 &&
                        Math.Abs(virt_point.Y) < float.MaxValue * 0.5)
                    {
                        quadedge.pts[1] = _quadEdges[edge1 >> 2].pts[1 + (edge1 & 2)] =
                        _quadEdges[edge2 >> 2].pts[1 + (edge2 & 2)] = NewPoint(virt_point, true);
                    }
                }
            }

            _validGeometry = true;
        }

        private readonly List<VertexData> _vertices = new List<VertexData>();
        private readonly List<QuadEdgeData> _quadEdges = new List<QuadEdgeData>();

        private int _freeQEdge;
        private int _freePoint;
        private bool _validGeometry;

        private void InitDelaunay()
        {
            _vertices.Clear();
            _quadEdges.Clear();
            _validGeometry = false;

            float rx = Bounds.MinValue.X;
            float ry = Bounds.MinValue.Y;

            float big_coord = 3.0f * Math.Max(Bounds.Width, Bounds.Height);

            var ppA = new Vector2(rx + big_coord, ry);
            var ppB = new Vector2(rx, ry + big_coord);
            var ppC = new Vector2(rx - big_coord, ry - big_coord);

            _vertices.Add(new VertexData());
            _quadEdges.Add(new QuadEdgeData());

            _freeQEdge = 0;
            _freePoint = 0;

            int pA = NewPoint(ppA, false);
            int pB = NewPoint(ppB, false);
            int pC = NewPoint(ppC, false);

            int edge_AB = NewEdge();
            int edge_BC = NewEdge();
            int edge_CA = NewEdge();

            SetEdgePoints(edge_AB, pA, pB);
            SetEdgePoints(edge_BC, pB, pC);
            SetEdgePoints(edge_CA, pC, pA);

            Splice(edge_AB, SymEdge(edge_CA));
            Splice(edge_BC, SymEdge(edge_AB));
            Splice(edge_CA, SymEdge(edge_BC));
        }


        protected override int EdgeDst(int edge)
        {
            return _quadEdges[edge >> 2].pts[(edge + 2) & 3];
        }

        protected override int EdgeOrg(int edge)
        {
            return _quadEdges[edge >> 2].pts[edge & 3];
        }

        protected override int GetQuadEdgesCount()
        {
            return _quadEdges.Count;
        }

        protected override void InvalidateGeometry()
        {
            _validGeometry = false;
        }

        protected override Vector2 GetVertexLocation(int vidx)
        {
            return _vertices[vidx].pt;
        }

        protected override int NextEdge(int edge)
        {
            var result = _quadEdges[edge >> 2].next[edge & 3];
            return result;
        }

        protected override int GetEdge(int edge, TargetEdgeType targetType)
        {
            int tmpEdge = _quadEdges[edge >> 2].next[(edge + (int) targetType) & 3];
            var result = (tmpEdge & ~3) + ((tmpEdge + ((int) targetType >> 4)) & 3);
            return result;
        }

        protected override int NewPoint(Vector2 pt, bool isvirtual, int firstEdge = 0)
        {
            if (_freePoint == 0)
            {
                _vertices.Add(new VertexData());
                _freePoint = _vertices.Count - 1;
            }

            int vidx = _freePoint;
            _freePoint = _vertices[vidx].firstEdge;
            _vertices[vidx] = new VertexData(pt, isvirtual, firstEdge);
            return vidx;
        }

        protected override void SetEdgePoints(int edge, int orgPt, int dstPt)
        {
            var pts = _quadEdges[edge >> 2].pts;

            pts[edge & 3] = orgPt;
            pts[(edge + 2) & 3] = dstPt;

            _vertices[orgPt].firstEdge = edge;
            _vertices[dstPt].firstEdge = edge ^ 2;
        }

        protected override int NewEdge()
        {
            if (_freeQEdge <= 0)
            {
                _quadEdges.Add(new QuadEdgeData());
                _freeQEdge = _quadEdges.Count - 1;
            }

            int edge = _freeQEdge * 4;
            _freeQEdge = _quadEdges[edge >> 2].next[1];
            _quadEdges[edge >> 2] = new QuadEdgeData(edge);
            return edge;
        }

        protected override void Splice(int edgeA, int edgeB)
        {
            int[] a_nextArray = _quadEdges[edgeA >> 2].next;
            int[] b_nextArray = _quadEdges[edgeB >> 2].next;

            int a_nextIndex = edgeA & 3;
            int b_nextIndex = edgeB & 3;

            int a_rot = RotateEdge(a_nextArray[a_nextIndex], 1);
            int b_rot = RotateEdge(b_nextArray[b_nextIndex], 1);

            int[] a_rot_nextArray = _quadEdges[a_rot >> 2].next;
            int[] b_rot_nextArray = _quadEdges[b_rot >> 2].next;

            int a_rot_nextIndex = a_rot & 3;
            int b_rot_nextIndex = b_rot & 3;

            var tmp = a_nextArray[a_nextIndex];
            a_nextArray[a_nextIndex] = b_nextArray[b_nextIndex];
            b_nextArray[b_nextIndex] = tmp;

            tmp = a_rot_nextArray[a_rot_nextIndex];
            a_rot_nextArray[a_rot_nextIndex] = b_rot_nextArray[b_rot_nextIndex];
            b_rot_nextArray[b_rot_nextIndex] = tmp;
        }

        protected override void DeletePoint(int vidx)
        {
            _vertices[vidx].firstEdge = _freePoint;
            _vertices[vidx].type = -1;
            _freePoint = vidx;
        }

        protected override void EdgeMarkDeleted(int edge)
        {
            edge >>= 2;
            _quadEdges[edge].next[0] = 0;
            _quadEdges[edge].next[1] = _freeQEdge;
            _freeQEdge = edge;
        }

        class VertexData
        {
            public VertexData()
            {
                firstEdge = 0;
                type = -1;
                pt = new Vector2(float.NaN, float.NaN);
            }

            public VertexData(Vector2 pt, bool isvirtual, int firstEdge)
            {
                this.firstEdge = firstEdge;
                this.type = isvirtual ? 1 : 0;
                this.pt = pt;
            }


            public bool IsVirtual => type > 0;
            public bool IsFree => type < 0;

            public int firstEdge;

            public int type;

            public Vector2 pt;
        }

        class QuadEdgeData
        {
            public QuadEdgeData()
            {

            }

            public QuadEdgeData(int edgeidx)
            {
                next[0] = edgeidx;
                next[1] = edgeidx + 3;
                next[2] = edgeidx + 2;
                next[3] = edgeidx + 1;
            }

            public bool IsFree => next[0] <= 0;

            public readonly int[] next = new int[4];
            public readonly int[] pts = new int[4];
        }
    }


    internal class SearchContext
    {
        const int DEFAULT_EDGE_ID = 4;
        
        public int RecentEdge { get; set; }
        public int Edge{ get; set; }
        public int Vertex { get; set; }

        public SearchContext()
        {
            Clear();
        }

        public void Clear()
        {
            RecentEdge = DEFAULT_EDGE_ID;
            Edge = 0;
            Vertex = 0;
        }
    }
}
