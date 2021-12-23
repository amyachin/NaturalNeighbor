using MathNet.Numerics.Interpolation;
using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NaturalNeighbor.Internal
{
    internal class CubicSmoothingSpline : PPoly, ISmoothingSpline
    {
        private readonly double[] _x;
        private readonly double[] _y;
        private readonly double[] _weights;
        private readonly double[,] _coeffs;
        private readonly double _smoothingFactor;

        protected override double[] X => _x;
        protected override double[,] Coeffs => _coeffs;

        private static VectorBuilder<double> vb => Vector<double>.Build;
        private static MatrixBuilder<double> mb => Matrix<double>.Build;

        public CubicSmoothingSpline(
            double[] x, 
            double[] y,
            double[] w,
            double? smoothingFactor,
            bool normalizedSmooth)
        {
            _x = x;
            _y = y;
            var (xdata, ydata, weights) = PrepareData(x, y, w);
            _weights = weights.AsArray();
            (_coeffs, _smoothingFactor) = MakeSpline(xdata, ydata, weights, smoothingFactor, normalizedSmooth);
        }

        private static (Vector<double> xData, Vector<double> yData, Vector<double> weights) PrepareData(
            double[] x, double[] y, double[] w)
        {
            if (x.Length < 2)
                throw new ArgumentException("X must have at least 2 elements.");

            if (x.Length != y.Length)
                throw new ArgumentException("X and Y must have the same number of elements.");

            if (w == null)
                w = Enumerable.Repeat(1.0, x.Length).ToArray();

            if (w.Length != x.Length)
                throw new ArgumentException("Weights vector size must be equal to xdata size.");

            return (vb.DenseOfArray(x), vb.DenseOfArray(y), vb.DenseOfArray(w));
        }

        private static (double[,] c, double sf) MakeSpline(
            Vector<double> x, Vector<double> y, Vector<double> w, double? smoothingFactor, bool normalizedSmooth)
        {
            int pcount = x.Count;
            Vector<double> dx = Diff(x);

            if (!dx.All(v => v > 0))
            {
                throw new ArgumentException(
                    "Items of 'xdata' vector must satisfy the condition: x1 < x2 < ... < xN");
            }

            Vector<double> dy = Diff(y);
            Vector<double> dy_dx = dy / dx;

            if (pcount == 2)
            {
                // The corner case for the data with 2 points (1 breaks interval)
                // In this case we have 2-ordered spline and linear interpolation in fact
                Matrix<double> co = mb.Dense(2, 1, new[] { dy_dx[0], y[0] });
                double sf = 1.0;
                return (co.ToArray(), sf);
            }

            var dx_1 = dx.SubVector(1, dx.Count - 1);
            var dx_m1 = dx.SubVector(0, dx.Count - 1);

            // Create diagonal sparse matrices
            Matrix<double> diags_r = mb.DenseOfRowVectors(dx_1, 2 * (dx_1 + dx_m1), dx_m1);
            Matrix<double> r = SpDiags(diags_r, new[] { -1, 0, 1 }, pcount - 2, pcount - 2);

            Vector<double> dx_recip = 1.0 / dx;

            var dx_recip_1 = dx_recip.SubVector(1, dx_recip.Count - 1);
            var dx_recip_m1 = dx_recip.SubVector(0, dx_recip.Count - 1);

            Matrix<double> diags_qtw = mb.DenseOfRowVectors(dx_recip_m1, -(dx_recip_1 + dx_recip_m1), dx_recip_1);
            Matrix<double> diags_sqrw_recip = mb.DenseOfRowVectors(1.0 / w.PointwiseSqrt());

            Matrix<double> qtw = (SpDiags(diags_qtw, new[] { 0, 1, 2 }, pcount - 2, pcount) *
                                  SpDiags(diags_sqrw_recip, new[] { 0 }, pcount, pcount));
            qtw = qtw * qtw.Transpose();

            double p;

            if (normalizedSmooth)
                p = NormalizeSmooth(x, w, smoothingFactor);
            else if (!smoothingFactor.HasValue)
                p = ComputeSmooth(r, qtw);
            else
                p = smoothingFactor.Value;

            double pp = (6.0 * (1.0 - p));

            // Solve linear system for the 2nd derivatives
            Matrix<double> a = pp * qtw + p * r;
            Matrix<double> b = mb.DenseOfRowVectors(Diff(dy_dx)).Transpose();

            Matrix<double> u = a.Solve(b).Transpose();

            Vector<double> Pad(Vector<double> v) =>
                vb.DenseOfEnumerable(Enumerable.Repeat(0.0, 1).Concat(v.Enumerate()).Concat(Enumerable.Repeat(0.0, 1)));

            Vector<double> d1 = Diff(Pad(u.Row(0))) / dx;
            Vector<double> d2 = Diff(Pad(d1));

            Vector<double> diags_w_recip = 1.0 / w;
            Matrix<double> w_m = SpDiags(mb.DenseOfRowVectors(diags_w_recip), new[] { 0 }, pcount, pcount);

            Vector<double> yi = y - (pp * w_m) * d2;
            Vector<double> pu = Pad(p * u.Row(0));

            var pu_1 = pu.SubVector(1, pu.Count - 1);
            var pu_m1 = pu.SubVector(0, pu.Count - 1);

            Vector<double> c1 = Diff(pu) / dx;
            Vector<double> c2 = 3.0 * pu_m1;
            Vector<double> c3 = Diff(yi) / dx - dx.PointwiseMultiply(2.0 * pu_m1 + pu_1);
            Vector<double> c4 = yi.SubVector(0, yi.Count - 1);

            Matrix<double> c = mb.DenseOfRowVectors(c1, c2, c3, c4);

            return (c.ToArray(), p);
        }

        /// <summary>
        /// Implements scipy's spdiags operation
        /// </summary>
        private static Matrix<double> SpDiags(Matrix<double> diags, int[] offsets, int rows, int cols)
        {
            if (diags.RowCount != offsets.Length)
            {
                throw new ArgumentException("Number of rows in data array must match diag length");
            }

            Matrix<double> ret = mb.Sparse(rows, cols, 0.0);

            for (int i = 0; i < offsets.Length; i++)
            {
                int offset = offsets[i];
                Vector<double> diag = diags.Row(i);

                for (int j = 0; j < diag.Count; j++)
                {
                    int row = j;
                    int col = j;

                    if (offset < 0)
                        row -= offset;
                    else
                        col += offset;

                    if (row < ret.RowCount && col < ret.ColumnCount)
                        ret[row, col] = diag[j];
                }
            }

            return ret;
        }

        /// <summary>
        /// The calculation of the smoothing spline requires the solution of a
        /// linear system whose coefficient matrix has the form p* A + (1-p)*B, with
        /// the matrices A and B depending on the data sites x.The default value
        /// of p makes p* trace(A) equal (1 - p)*trace(B).
        /// </summary>
        private static double ComputeSmooth(Matrix<double> a, Matrix<double> b)
        {
            double Trace(Matrix<double>  m) => m.Diagonal().Sum();

            return 1.0 / (1.0 + Trace(a) / (6.0 * Trace(b)));
        }

        /// <summary>
        /// See the explanation here: https://github.com/espdev/csaps/pull/47
        /// </summary>
        private static double NormalizeSmooth(Vector<double> x, Vector<double> w, double? smooth)
        {
            double span = x.Enumerate().Max() - x.Enumerate().Min();

            double eff_x = 1 + Math.Pow(span, 2) / Diff(x).PointwisePower(2).Sum();
            double eff_w = Math.Pow(w.Sum(), 2) / w.PointwisePower(2).Sum();
            double k = 80 * Math.Pow(span, 3) * Math.Pow(x.Count, -2) * Math.Pow(eff_x, -0.5) * Math.Pow(eff_w, -0.5);

            double s = smooth.HasValue ? smooth.Value : 0.5;
            double p = s / (s + (1 - s) * k);

            return p;
        }

        private static Vector<double> Diff(Vector<double> v) =>
            Vector<double>.Build.DenseOfEnumerable(v.Zip(v.Skip(1), (a, b) => (b - a)));

        double[] ISmoothingSpline.Evaluate(double[] xi) => Evaluate(xi);

        public double SmoothingFactor => _smoothingFactor;
    }
}
