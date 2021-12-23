using System.Collections.Immutable;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using System.IO;

namespace NaturalNeighbor.Internal
{
    sealed class SubDiv2D_Immutable : SubDiv2D_Base
    {
        internal SubDiv2D_Immutable(Bounds bounds, ImmutableDictionary<int, ImmutableVertexData> vertices, ImmutableDictionary<int, ImmutableQuadEdgeData> quadedges)
            : base(bounds)
        {
            _vertices = vertices;
            _quadEdges = quadedges;

            _freeQEdge = quadedges.First(it => it.Value.IsFree).Key;
            _freePoint = vertices.First(it => it.Value.IsFree).Key;
        }

        private SubDiv2D_Immutable(SubDiv2D_Immutable prototype) : base(prototype.Bounds)
        {
            _vertices = prototype._vertices;
            _quadEdges = prototype._quadEdges;

            _freePoint = prototype._freePoint;
            _freeQEdge = prototype._freeQEdge;
        }

        internal static VoronoiFacet SynthesizeFacet(SubDiv2D_Immutable prototype, SearchContext context, Vector2 pt, HashSet<NodeId> neighbors)
        {
            neighbors.Clear();
            var impl = new SubDiv2D_Immutable(prototype);
            return impl.SynthesizeFacetCore(pt, context, neighbors);
        }

        internal static List<int> CreateEnvelope(SubDiv2D_Immutable prototype, SearchContext context, Vector2 pt)
        {
            var impl = new SubDiv2D_Immutable(prototype);
            return impl.SynthesizeEnvelope(pt, context);
        }



        private VoronoiFacet SynthesizeFacetCore(Vector2 samplePt, SearchContext context, HashSet<NodeId> neighbors)
        {
            var vid = Insert(samplePt, context);
            return CreateVoronoiFacet(vid, neighbors);
        }

        private List<int> SynthesizeEnvelope(Vector2 samplePt, SearchContext context)
        {
            var vid = Insert(samplePt, context);
            var v = _vertices[(int) vid];
            var edge = v.firstEdge;

            List<int> list = new List<int>();

            edge = RotateEdge(edge, 1);
            var t = edge;

            // Gather cell points
            do
            {
                t = GetEdge(t, TargetEdgeType.NEXT_AROUND_LEFT);
                var e  = GetEdge(RotateEdge(t, -1), TargetEdgeType.NEXT_AROUND_LEFT);
                list.Add(e);

            }
            while (t != edge);

            return list;
        }


        //private void GetEnvelope(NodeId nodeId, List<int> polygon)
        //{
        //    var v = _vertices[(int) nodeId];
        //    var edge = v.firstEdge;

        //    EdgeDst(edge);

        //    edge = RotateEdge(edge, 1);
        //    var t = edge;
        //    // Gather cell points
        //    do
        //    {
        //        t = GetEdge(t, TargetEdgeType.NEXT_AROUND_LEFT);
        //        polygon.Add(t);
        //    }
        //    while (t != edge);
        //}

        private VoronoiFacet CreateVoronoiFacet(NodeId nodeId, HashSet<NodeId> neighbors)
        {
            var v = _vertices[(int) nodeId];
            var edge = v.firstEdge;
            var center = v.pt;

            var buff = ImmutableArray.CreateBuilder<Vector2>();

            EdgeDst(edge, out var prevDestPt);
            edge = RotateEdge(edge, 1);
            var t = edge;

            // Gather cell points
            do
            {
                t = GetEdge(t, TargetEdgeType.NEXT_AROUND_LEFT);

                int dest = EdgeDst(RotateEdge(t, -1), out var destPt);
                var pt = Utils.ComputeVoronoiPoint(center, prevDestPt, center, destPt);

                neighbors.Add(new NodeId(dest));
                prevDestPt = destPt;

                int cnt = buff.Count;
                if (cnt > 0)
                {
                    double dx = pt.X - buff[cnt - 1].X;
                    double dy = pt.Y - buff[cnt - 1].Y;

                    // Exclude duplicate entries
                    if (dx * dx + dy * dy < float.Epsilon)
                    {
                        continue;
                    }

                    // Remove points that are not part of convex hull 
                    if (cnt > 1 && Utils.IsLeft(buff[cnt - 2], buff[cnt - 1], pt) <= 0)
                    {
                        buff[cnt - 1] = pt;
                        continue;
                    }

                    if (t == edge)
                    {
                        dx = pt.X - buff[0].X;
                        dy = pt.Y - buff[0].Y;

                        if (dx * dx + dy * dy < float.Epsilon)
                        {
                            continue;
                        }

                        if (cnt > 1 && Utils.IsLeft(buff[cnt - 1], pt, buff[0]) <= 0)
                        {
                            buff[0] = pt;
                            continue;
                        }
                    }
                }

                buff.Add(pt);
            }
            while (t != edge);

            return new VoronoiFacet(nodeId, center, buff.ToImmutable());
        }

        protected override void DeletePoint(int vidx)
        {
            var prototype = _vertices[vidx];
            _vertices = _vertices.SetItem(vidx, new ImmutableVertexData(prototype.pt, type: -1, firstEdge: _freePoint)); 
            _freePoint = vidx;
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
                _vertices = _vertices.Add(_vertices.Count, ImmutableVertexData.Default);
                _freePoint = _vertices.Count - 1;
            }

            int vidx = _freePoint;
            _freePoint = _vertices[vidx].firstEdge;
            _vertices = _vertices.SetItem(vidx, new ImmutableVertexData(pt, isvirtual, firstEdge));
            return vidx;
        }

        protected override void SetEdgePoints(int edge, int orgPt, int dstPt)
        {
            var prototype = _quadEdges[edge >> 2];

            var pts = prototype.pts.ToBuilder();
            pts[edge & 3] = orgPt;
            pts[(edge + 2) & 3] = dstPt;

            _quadEdges = _quadEdges.SetItem(edge >> 2, 
                new ImmutableQuadEdgeData(next: prototype.next, pts: pts.ToImmutable()));

            var vorg = _vertices[orgPt];
            var vdest = _vertices[dstPt];

            _vertices = _vertices.SetItems(new KeyValuePair<int, ImmutableVertexData>[] {
                new KeyValuePair<int, ImmutableVertexData>(orgPt, new ImmutableVertexData(pt: vorg.pt, type: vorg.type, firstEdge: edge)),
                new KeyValuePair<int, ImmutableVertexData>(dstPt, new ImmutableVertexData(pt: vdest.pt, type : vdest.type, firstEdge: edge ^ 2))
            });
        }


        protected override void EdgeMarkDeleted(int edge)
        {
            edge >>= 2;

            var prototype = _quadEdges[edge];
            var builder = prototype.next.ToBuilder();

            builder[0] = 0;
            builder[1] = _freeQEdge;

            _quadEdges = _quadEdges.SetItem(edge, new ImmutableQuadEdgeData(next: builder.ToImmutable(), pts: prototype.pts));
            _freeQEdge = edge;
        }

        protected override int NewEdge()
        {
            if (_freeQEdge <= 0)
            {
                _quadEdges = _quadEdges.Add(_quadEdges.Count, ImmutableQuadEdgeData.Default);
                _freeQEdge = _quadEdges.Count - 1;
            }

            int edge = _freeQEdge * 4;
            _freeQEdge = _quadEdges[edge >> 2].next[1];
            _quadEdges = _quadEdges.SetItem(edge >> 2, new ImmutableQuadEdgeData(edge));
            return edge;
        }

        protected override void Splice(int edgeA, int edgeB)
        {
            var pendingChanges = new Dictionary<int, (ImmutableArray<int>.Builder, ImmutableArray<int>)>(4);

            ImmutableArray<int>.Builder a_nextArray, b_nextArray;
            {
                int idxA = edgeA >> 2;
                var edgeDataA = _quadEdges[idxA];
                a_nextArray = edgeDataA.next.ToBuilder();
                pendingChanges.Add(idxA, (a_nextArray, edgeDataA.pts));

                int idxB = edgeB >> 2;
                if (idxA != idxB)
                {
                    var edgeDataB = _quadEdges[idxB];
                    b_nextArray = edgeDataB.next.ToBuilder();
                    pendingChanges.Add(idxB, (b_nextArray, edgeDataB.pts));
                }
                else
                {
                    // Special case when a and B are pointing to the same quadedge
                    b_nextArray = a_nextArray;
                }
            }

            int a_nextIndex = edgeA & 3;
            int b_nextIndex = edgeB & 3;

            int a_rot = RotateEdge(a_nextArray[a_nextIndex], 1);
            int b_rot = RotateEdge(b_nextArray[b_nextIndex], 1);

            ImmutableArray<int>.Builder a_rot_nextArray, b_rot_nextArray;
            {
                int idxA = a_rot >> 2;
                if (pendingChanges.TryGetValue(idxA, out var data))
                {
                    // Special case when a_rot shares a slot with another entry
                    a_rot_nextArray = data.Item1;
                }
                else
                {
                    var edgeDataA = _quadEdges[idxA];
                    a_rot_nextArray = edgeDataA.next.ToBuilder();
                    pendingChanges.Add(idxA, (a_rot_nextArray, edgeDataA.pts));
                }

                int idxB = b_rot >> 2;
                if (idxB == idxA)
                {
                    // Special case when both a_rot and b_rot point to the same slot
                    b_rot_nextArray = a_rot_nextArray;
                }
                else if (pendingChanges.TryGetValue(idxB, out data))
                {
                    // Special case when b_rot shares a slot with another entry
                    b_rot_nextArray = data.Item1;
                }
                else
                {
                    var edgeDataB = _quadEdges[idxB];
                    b_rot_nextArray = edgeDataB.next.ToBuilder();
                    pendingChanges.Add(idxB, (b_rot_nextArray, edgeDataB.pts));
                }
            }

            int a_rot_nextIndex = a_rot & 3;
            int b_rot_nextIndex = b_rot & 3;

            var tmp = a_nextArray[a_nextIndex];
            a_nextArray[a_nextIndex] = b_nextArray[b_nextIndex];
            b_nextArray[b_nextIndex] = tmp;

            tmp = a_rot_nextArray[a_rot_nextIndex];
            a_rot_nextArray[a_rot_nextIndex] = b_rot_nextArray[b_rot_nextIndex];
            b_rot_nextArray[b_rot_nextIndex] = tmp;


            var builder = _quadEdges.ToBuilder();
            foreach (var item in pendingChanges)
            {
                builder[item.Key] = new ImmutableQuadEdgeData(next: item.Value.Item1.ToImmutable(), pts: item.Value.Item2);
            }

            _quadEdges = builder.ToImmutable();

        }

        private int _freeQEdge;
        private int _freePoint;
        private ImmutableDictionary<int, ImmutableVertexData> _vertices;
        private ImmutableDictionary<int, ImmutableQuadEdgeData> _quadEdges;
    }
}
