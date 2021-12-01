using System;
using System.Collections.Generic;
using System.Numerics;
namespace NaturalNeighbor.Internal
{
    internal static class Utils
    {
        public static Bounds CalculateBounds(IReadOnlyList<Vector2> points, double margin, Vector2? minValue, Vector2? maxValue)
        {
            if (margin < 0.0)
            {
                throw new ArgumentOutOfRangeException(nameof(margin), "Margin cannot be negative.");
            }

            if (points.Count == 0)
            {
                throw new ArgumentOutOfRangeException(nameof(points), "Points collection cannot be empty.");
            }

            double minX = float.MaxValue;
            double minY = float.MaxValue;

            double maxX = float.MinValue;
            double maxY = float.MinValue;

            for (int i = 0; i < points.Count; ++i)
            {
                var pt = points[i];

                minX = Math.Min(minX, pt.X);
                maxX = Math.Max(maxX, pt.X);

                minY = Math.Min(minY, pt.Y);
                maxY = Math.Max(maxY, pt.Y);
            }

            minX -= margin;
            minY -= margin;
            maxX += margin;
            maxY += margin;


            if (minValue.HasValue)
            {
                minX = Math.Min(minX, minValue.Value.X);
                minY = Math.Min(minY, minValue.Value.Y);
            }

            if (maxValue.HasValue)
            {
                maxX = Math.Max(maxX, maxValue.Value.X);
                maxY = Math.Max(maxY, maxValue.Value.Y);
            }

            return new Bounds(minX, maxX, minY, maxY);
        }

        internal static (double, double) GetCentroid(int startIndex, int count, IReadOnlyList<Vector2> polygon)
        {

            double sumX = 0.0;
            double sumY = 0.0;

            for (int i = startIndex; i < count; i++)
            {
                var pt = polygon[i];
                sumX += pt.X;
                sumY += pt.Y;
            }

            double k = 1.0 / count;
            return (k * sumX, k * sumY);
        }

        internal static (double, double) GetCentroid(IReadOnlyList<Vector2> polygon)
        {
            return GetCentroid(0, polygon.Count, polygon);
        }

        public static (Vector2, Vector2) OverlapSegments(Vector2 p0, Vector2 p1, Vector2 q0, Vector2 q1)
        {
            Vector2 direction;
            Vector2 start;



            if ((p1 - p0).LengthSquared() > (q1 - q0).LengthSquared())
            {
                start = p0;
                direction = Vector2.Normalize(p1 - p0);
            }
            else
            {
                start = q0;
                direction = Vector2.Normalize(q1 - q0);
            }


            double mup0 = DotProduct((double) p0.X - start.X, (double) p0.Y - start.Y, direction.X, direction.Y);
            double mup1 = DotProduct((double) p1.X - start.X, (double) p1.Y - start.Y, direction.X, direction.Y);
            double muq0 = DotProduct((double) q0.X - start.X, (double) q0.Y - start.Y, direction.X, direction.Y);
            double muq1 = DotProduct((double) q1.X - start.X, (double) q1.Y - start.Y, direction.X, direction.Y);

            double a0 = Math.Min(mup0, mup1);
            double a1 = Math.Max(mup0, mup1);
            double b0 = Math.Min(muq0, muq1);
            double b1 = Math.Max(muq0, muq1);


            var k1 = Math.Min(a0, b0);
            var k2 = Math.Max(a1, b1);

            var pt1 = new Vector2((float) (start.X + direction.X * k1), (float) (start.Y + direction.Y * k1));
            var pt2 = new Vector2((float) (start.X + direction.X * k2), (float) (start.Y + direction.Y * k2));
            return (pt1, pt2);
        }

        public static double IsLeft(Vector2 p0, Vector2 p1, Vector2 target)
        {
            return ((double) p1.X - p0.X) * ((double) target.Y - p0.Y) - ((double) target.X - p0.X) * ((double) p1.Y - p0.Y);
        }

        public static double AngleBetween(double x0, double y0, double x1, double y1, bool ccw)
        {
            double y = x0 * y1 - x1 * y0;
            double x = x0 * x1 + y0 * y1;

            double result = Math.Atan2(y, x);

            // Normalize angle

            if (ccw && result < 0)
            {
                result = result + Math.PI * 2.0;
            }

            if (!ccw && result > 0)
            {
                result = result - Math.PI * 2.0;
            }

            return (float) result;

        }

        public static double CrossProduct(double x0, double y0, double x1, double y1)
        {
            return x0 * y1 - x1 * y0;
        }

        public static double DotProduct(double x0, double y0, double x1, double y1)
        {
            return x0 * x1 + y0 * y1;
        }

        public static bool IsPointInTriangle(Triangle triangle, Vector2 pt)
        {
            return 
                IsLeft(triangle.P1, triangle.P2, pt) >= 0 &&
                IsLeft(triangle.P2, triangle.P3, pt) >= 0 &&
                IsLeft(triangle.P3, triangle.P1, pt) >= 0;
        }

        internal static double Lerp(IReadOnlyList<Vector3> points, Vector2 pt) 
        {
            switch (points.Count)
            {
                case 0: 
                    return double.NaN;
                case 1: 
                    return points[0].Z;
                case 2: 
                    return Lerp2Pts(points[0], points[1], pt);
                default: 
                    return Lerp3Pts(points[0], points[1], points[2], pt);
            }
        }

        private static double Lerp2Pts(Vector3 p1, Vector3 p2, Vector2 loc)
        {
            double dx = (double)p2.X - p1.X;
            double dy = (double) p2.Y - p1.Y;

            double dx1 = (double) loc.X - p1.X;
            double dy1 = (double) loc.Y - p1.Y;

            double mu1 = DotProduct(dx, dy, dx1, dy1);
            double mu2 = DotProduct(dx, dy, dx, dy);

            double t2 = mu1 / mu2;
            double t1 = 1.0 - t2;

            return t1 * p1.Z + t2 * p2.Z;
        }

        private static double Lerp3Pts(Vector3 p1, Vector3 p2, Vector3 p3, Vector2 loc)
        {
            // Using barycentric coordinates as explained in
            // https://stackoverflow.com/questions/8697521/interpolation-of-a-triangle
            // https://codeplea.com/triangular-interpolation
            // https://en.wikipedia.org/wiki/Barycentric_coordinate_system

            double det = ((double) p2.Y - p3.Y) * ((double) p1.X - p3.X) + ((double) p3.X - p2.X) * ((double) p1.Y - p3.Y);
            double t1 = (((double) p2.Y - p3.Y) * ((double) loc.X - p3.X) + ((double) p3.X - p2.X) * ((double) loc.Y - p3.Y)) / det;
            double t2 = (((double) p3.Y - p1.Y) * ((double) loc.X - p3.X) + ((double) p1.X - p3.X) * ((double) loc.Y - p3.Y)) / det;
            double t3 = 1.0 - t1 - t2;

            return t1 * p1.Z + t2 * p2.Z + t3 * p3.Z;
        }

        public static SegmentIntersectionType CheckSegmentIntersection(Vector2 p0, Vector2 p1, Vector2 q0, Vector2 q1)
        {
            double x0 = IsLeft(p0, p1, q0);
            double y0 = IsLeft(p0, p1, q1);

            if (Math.Abs(x0) < float.Epsilon)
            {
                x0 = 0.0f;
            }

            if (Math.Abs(y0) < float.Epsilon)
            {
                y0 = 0.0f;
            }

            int sx0 = Math.Sign(x0);
            int sy0 = Math.Sign(y0);

            if (sx0 != 0 && sx0 == sy0)
            {
                return SegmentIntersectionType.None;
            }


            double x1 = IsLeft(q0, q1, p0);
            double y1 = IsLeft(q0, q1, p1);

            if (Math.Abs(x1) < float.Epsilon)
            {
                x1 = 0.0f;
            }

            if (Math.Abs(y1) < float.Epsilon)
            {
                y1 = 0.0f;
            }

            int sx1 = Math.Sign(x1);
            int sy1 = Math.Sign(y1);

            if (sx1 != 0 && sx1 == sy1)
            {
                return SegmentIntersectionType.None;
            }

            if (sx0 == 0 && sx1 == 0 && sy0 == 0 && sy1 == 0)
            {

                double vpx = (double) p1.X - p0.X;
                double vpy = (double) p1.Y - p0.Y;

                double mu1 = DotProduct(vpx, vpy, (double) q0.X - p0.X, (double) q0.Y - p0.Y);
                double mu2 = DotProduct(vpx, vpy, (double) q1.X - p0.X, (double) p1.Y - p0.X);
                double mu0 = DotProduct(vpx, vpy, vpx, vpy);

                if (mu1 < 0 && mu2 < 0 || mu1 > mu0 && mu2 > mu0)
                {
                    return SegmentIntersectionType.None;
                }

                else
                {
                    return SegmentIntersectionType.Overlap;
                }
            }

            // Check "Proper"
            bool inner = (sx0 > 0) ^ (sy0 > 0) &&
                         (sx1 > 0) ^ (sy1 > 0);

            return inner ? SegmentIntersectionType.Inner : SegmentIntersectionType.Endpoint;
        }

        public static Vector2 IntersectLines(Vector2 p0, Vector2 p1, Vector2 q0, Vector2 q1)
        {
            double v1x = (double)p0.X - p1.X;
            double v1y = (double)p0.Y - p1.Y;

            double v2x = (double) q0.X - q1.X;
            double v2y = (double) q0.Y - q1.Y;

            double det = CrossProduct(v1x, v1y, v2x, v2y);
            double a = CrossProduct(p0.X, p0.Y, p1.X, p1.Y);
            double b = CrossProduct(q0.X, q0.Y, q1.X, q1.Y);

            double k = 1.0 / det;
            double x = (a * v2x - b * v1x) * k;
            double y = (a * v2y - b * v1y) * k;

            return new Vector2((float)x, (float)y);
        }

        internal static double ComputePolygonArea2(IReadOnlyList<Vector2> poly)
        {
            (var cx, var cy)= GetCentroid(poly);
            
            var p0 = poly[poly.Count - 1];
            var p1 = poly[0];


            double area = CrossProduct(p0.X - cx, p0.Y - cy, p1.X - cx, p1.Y - cy);
            p0 = p1;

            for (int i = 1; i < poly.Count; ++i)
            {
                p1 = poly[i];
                area += CrossProduct(p0.X - cx, p0.Y - cy, p1.X - cx, p1.Y - cy);
                p0 = p1;
            }

            return area;
        }

        internal static Vector2 ComputeVoronoiPoint(Vector2 org0, Vector2 dst0, Vector2 org1, Vector2 dst1)
        {
            double a0 = (double) dst0.X - org0.X;
            double b0 = (double) dst0.Y - org0.Y;
            double c0 = -0.5 * (a0 * ((double)dst0.X + org0.X) + b0 * ((double) dst0.Y + org0.Y));

            double a1 = (double) dst1.X - org1.X;
            double b1 = (double) dst1.Y - org1.Y;
            double c1 = -0.5 * (a1 * ((double) dst1.X + org1.X) + b1 * ((double) dst1.Y + org1.Y));

            double det = a0 * b1 - a1 * b0;

            if (det != 0)
            {
                var invDet = 1.0 / det;
                return new Vector2((float) ((b0 * c1 - b1 * c0) * invDet), (float) ((a1 * c0 - a0 * c1) * invDet));
            }

            return new Vector2(float.MaxValue, float.MaxValue);
        }

    }

    internal enum SegmentIntersectionType
    {
        None,
        Inner,
        Endpoint,
        Overlap
    }

}
