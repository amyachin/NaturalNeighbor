using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;

namespace NaturalNeighbor.Internal
{
    abstract class SubDiv2D_Base
    {
        internal SubDiv2D_Base(Bounds bounds)
        {
            this.Bounds = bounds;
        }

        internal Bounds Bounds { get; }

        public NodeId Insert(Vector2 pt, SearchContext context)
        {
            var location = Locate(pt, context);

            switch (location)
            {
                case PointLocationType.Error:
                    throw new InvalidOperationException("Unexpected error when locating the point.");

                case PointLocationType.OutsideRect:
                    throw new InvalidOperationException("Point is outside of the bounding box.");

                case PointLocationType.Vertex:
                    return new NodeId(context.Vertex);

                case PointLocationType.Edge:
                    int deleted_edge = context.Edge;
                    context.Edge = GetEdge(context.Edge, TargetEdgeType.PREV_AROUND_ORG);
                    DeleteEdge(deleted_edge);
                    break;

                case PointLocationType.Inside:
                    break;

                default:
                    throw new InvalidOperationException($"Locate returned invalid location = {location}.");
            }

            int curr_edge = context.Edge;
            Debug.Assert(curr_edge != 0);

            InvalidateGeometry();


            int curr_point = NewPoint(pt, false);
            int base_edge = NewEdge();
            int first_point = EdgeOrg(curr_edge);
            SetEdgePoints(base_edge, first_point, curr_point);
            Splice(base_edge, curr_edge);

            do
            {
                base_edge = ConnectEdges(curr_edge, SymEdge(base_edge));
                curr_edge = GetEdge(base_edge, TargetEdgeType.PREV_AROUND_ORG);
            }
            while (EdgeDst(curr_edge) != first_point);

            curr_edge = GetEdge(base_edge, TargetEdgeType.PREV_AROUND_ORG);

            int max_edges = GetQuadEdgesCount() * 4;

            for (int i = 0; i < max_edges; i++)
            {
                int temp_edge = GetEdge(curr_edge, TargetEdgeType.PREV_AROUND_ORG);

                int temp_dst = EdgeDst(temp_edge);
                int curr_org = EdgeOrg(curr_edge);
                int curr_dst = EdgeDst(curr_edge);

                if (IsRightOf(GetVertexLocation(temp_dst), curr_edge) > 0 &&
                   IsPtInCircle3(GetVertexLocation(curr_org), GetVertexLocation(temp_dst),
                                 GetVertexLocation(curr_dst), GetVertexLocation(curr_point)) < 0)
                {
                    SwapEdges(curr_edge);
                    curr_edge = GetEdge(curr_edge, TargetEdgeType.PREV_AROUND_ORG);
                }
                else if (curr_org == first_point)
                    break;
                else
                    curr_edge = GetEdge(NextEdge(curr_edge), TargetEdgeType.PREV_AROUND_LEFT);
            }

            return new NodeId(curr_point);
        }


        public Vector2 this[NodeId node] => GetVertexLocation((int)node);



        protected virtual void InvalidateGeometry()
        {
        }

        protected static int IsRightOf2(Vector2 pt, Vector2 org, Vector2 diff)
        {
            double cw_area = ((double)org.X - pt.X) * diff.Y - ((double)org.Y - pt.Y) * diff.X;
            return Math.Sign(cw_area);
        }

        internal static double TriangleArea(Vector2 a, Vector2 b, Vector2 c)
        {
            return ((double) b.X - a.X) * ((double)c.Y - a.Y) - ((double)b.Y - a.Y) * ((double)c.X - a.X);
        }

        internal int IsPtInCircle3(Vector2 pt, Vector2 a, Vector2 b, Vector2 c)
        {
            const double eps = float.Epsilon * 0.125;
            double val = (a.X * a.X + a.Y * a.Y) * TriangleArea(b, c, pt);
            val -= (b.X * b.X + b.Y * b.Y) * TriangleArea(a, c, pt);
            val += (c.X * c.X + c.Y * c.Y) * TriangleArea(a, b, pt);
            val -= (pt.X * pt.X + pt.Y * pt.Y) * TriangleArea(a, b, c);

            return val > eps ? 1 : val < -eps ? -1 : 0;
        }

        protected static int SymEdge(int edge)
        {
            return edge ^ 2;
        }

        protected static int RotateEdge(int edge, int rotate)
        {
            return (edge & ~3) + ((edge + rotate) & 3);
        }

        protected abstract int EdgeDst(int edge);

        protected abstract int EdgeOrg(int edge);

        protected int EdgeDst(int edge, out Vector2 dest)
        {
            var vix = EdgeDst(edge);
            dest = GetVertexLocation(vix);
            return vix;
        }

        protected int EdgeOrg(int edge, out Vector2 dest)
        {
            var vix = EdgeOrg(edge);
            dest = GetVertexLocation(vix);
            return vix;
        }

        protected abstract int GetQuadEdgesCount();

        protected abstract Vector2 GetVertexLocation(int vidx);

        protected abstract int NextEdge(int edge);

        protected abstract int GetEdge(int edge, TargetEdgeType targetType);

        internal PointLocationType Locate(Vector2 pt, SearchContext context)
        {

            var quadEdgesCount = GetQuadEdgesCount();

            if (quadEdgesCount < 4)
            {
                throw new InvalidOperationException("Subdivision is empty.");
            }

            int maxEdges = quadEdgesCount * 4;

            var bounds = this.Bounds;

            if (!bounds.Contains(pt))
            {
                context.Edge = 0;
                context.Vertex = 0;
                return PointLocationType.OutsideRect;
            }

            int edge = context.RecentEdge;
            int vertex = 0;

            Debug.Assert(edge > 0);

            PointLocationType location = PointLocationType.Error;

            int right_of_curr = IsRightOf(pt, edge);
            if (right_of_curr > 0)
            {
                edge = SymEdge(edge);
                right_of_curr = -right_of_curr;
            }

            for (int i = 0; i < maxEdges; i++)
            {
                int onext_edge = NextEdge(edge);
                int dprev_edge = GetEdge(edge, TargetEdgeType.PREV_AROUND_DST);

                int right_of_onext = IsRightOf(pt, onext_edge);
                int right_of_dprev = IsRightOf(pt, dprev_edge);

                if (right_of_dprev > 0)
                {
                    if (right_of_onext > 0 || (right_of_onext == 0 && right_of_curr == 0))
                    {
                        location = PointLocationType.Inside;
                        break;
                    }
                    else
                    {
                        right_of_curr = right_of_onext;
                        edge = onext_edge;
                    }
                }
                else
                {
                    if (right_of_onext > 0)
                    {
                        if (right_of_dprev == 0 && right_of_curr == 0)
                        {
                            location = PointLocationType.Inside;
                            break;
                        }
                        else
                        {
                            right_of_curr = right_of_dprev;
                            edge = dprev_edge;
                        }
                    }
                    else if (right_of_curr == 0 &&
                            IsRightOf(GetVertexLocation(EdgeDst(onext_edge)), edge) >= 0)
                    {
                        edge = SymEdge(edge);
                    }
                    else
                    {
                        right_of_curr = right_of_onext;
                        edge = onext_edge;
                    }
                }
            }

            context.RecentEdge = edge;

            if (location == PointLocationType.Inside)
            {
                EdgeOrg(edge, out var org_pt);
                EdgeDst(edge, out var dst_pt);

                double t1 = Math.Abs(pt.X - org_pt.X) + Math.Abs(pt.Y - org_pt.Y);
                double t2 = Math.Abs(pt.X - dst_pt.X) + Math.Abs(pt.Y - dst_pt.Y);
                double t3 = Math.Abs(org_pt.X - dst_pt.X) + Math.Abs(org_pt.Y - dst_pt.Y);

                if (t1 < float.Epsilon)
                {
                    location = PointLocationType.Vertex;
                    vertex = EdgeOrg(edge);
                    edge = 0;
                }
                else if (t2 < float.Epsilon)
                {
                    location = PointLocationType.Vertex;
                    vertex = EdgeDst(edge);
                    edge = 0;
                }
                else if ((t1 < t3 || t2 < t3) && Math.Abs(TriangleArea(pt, org_pt, dst_pt)) < float.Epsilon)
                {
                    location = PointLocationType.Edge;
                    vertex = 0;
                }
            }

            if (location == PointLocationType.Error)
            {
                edge = 0;
                vertex = 0;
            }


            context.Edge = edge;
            context.Vertex = vertex;
            return location;
        }

        protected abstract int NewPoint(Vector2 pt, bool isvirtual, int firstEdge = 0);

        protected abstract void SetEdgePoints(int edge, int orgPt, int dstPt);

        protected abstract int NewEdge();

        protected int ConnectEdges(int edgeA, int edgeB)
        {
            int edge = NewEdge();

            Splice(edge, GetEdge(edgeA, TargetEdgeType.NEXT_AROUND_LEFT));
            Splice(SymEdge(edge), edgeB);

            SetEdgePoints(edge, EdgeDst(edgeA), EdgeOrg(edgeB));
            return edge;
        }

        protected abstract void Splice(int edgeA, int edgeB);

        protected void DeleteEdge(int edge)
        {
            Splice(edge, GetEdge(edge, TargetEdgeType.PREV_AROUND_ORG));
            int sedge = SymEdge(edge);
            Splice(sedge, GetEdge(sedge, TargetEdgeType.PREV_AROUND_ORG));

            EdgeMarkDeleted(edge);
        }

        protected abstract void DeletePoint(int vidx);

        protected abstract void EdgeMarkDeleted(int edge);


        protected void SwapEdges(int edge)
        {
            int sedge = SymEdge(edge);
            int a = GetEdge(edge, TargetEdgeType.PREV_AROUND_ORG);
            int b = GetEdge(sedge, TargetEdgeType.PREV_AROUND_ORG);

            Splice(edge, a);
            Splice(sedge, b);

            SetEdgePoints(edge, EdgeDst(a), EdgeDst(b));

            Splice(edge, GetEdge(a, TargetEdgeType.NEXT_AROUND_LEFT));
            Splice(sedge, GetEdge(b, TargetEdgeType.NEXT_AROUND_LEFT));
        }

        internal int IsRightOf(Vector2 pt, int edge)
        {
            EdgeOrg(edge, out var org);
            EdgeDst(edge, out var dest);
            double cw_area = TriangleArea(pt, dest, org);
            return Math.Sign(cw_area);
        }


        internal enum TargetEdgeType
        {
            NEXT_AROUND_ORG = 0x00,
            NEXT_AROUND_DST = 0x22,
            PREV_AROUND_ORG = 0x11,
            PREV_AROUND_DST = 0x33,
            NEXT_AROUND_LEFT = 0x13,
            NEXT_AROUND_RIGHT = 0x31,
            PREV_AROUND_LEFT = 0x20,
            PREV_AROUND_RIGHT = 0x02
        };

    }


}
