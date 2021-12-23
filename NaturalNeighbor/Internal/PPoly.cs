using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NaturalNeighbor.Internal
{
    /// <summary>
    ///   Piecewise polynomial in terms of coefficients and breakpoints
    ///   The polynomial between ``x[i]`` and ``x[i + 1]`` is written in the
    ///   local power basis::
    ///     S = sum(c[m, i] * (xp - x[i])** (k-m) for m in range(k+1))
    ///     where ``k`` is the degree of the polynomial.
    /// </summary>
    /// <remarks>Based on scipy's PPoly</remarks>
    internal abstract class PPoly
    {
        /// <summary>
        /// Polynomial breakpoints. Must be sorted in increasing order.
        /// </summary>
        protected abstract double[] X { get; }

        /// <summary>
        /// Polynomial coefficients, order `k` and `m` intervals.
        /// </summary>
        protected abstract double[,] Coeffs { get; }

        protected double[] Evaluate(double[] xi)
        {
            double[] x = X;
            double[,] c = Coeffs;
            double[] ret = new double[xi.Length];

            int interval = 0;

            if (x[1] < x[0])
            {
                throw new ArgumentException("X input must be ascending");
            }

            for (int ip = 0; ip < xi.Length; ip++)
            {
                double xval = xi[ip];
                // Find the correct interval
                int i = FindIntervalAscending(x, xval, interval);

                if (i < 0)
                {
                    // xval was nan, etc.
                    ret[ip] = double.NaN;
                    continue;
                }
                else
                {
                    interval = i;
                }

                // Evaluate the local polynomial
                ret[ip] = EvaluatePoly1(xval - x[interval], c, interval);
            }

            return ret;
        }

        /// <summary>
        /// Find an interval such that x[interval] <= xval < x[interval+1]. Assuming
        /// that x is sorted in the ascending order.
        /// If xval<x[0], then interval = 0, if xval > x[-1] then interval = n - 2.
        /// </summary>
        private static int FindIntervalAscending(double[] x, double xval, int prevInterval)
        {
            double a = x[0];
            double b = x[x.Length - 1];

            int interval = prevInterval;

            if (interval < 0 || interval >= x.Length)
                interval = 0;

            if (!(a <= xval) && (xval <= b))
            {
                // Out of bounds (or nan)
                if (xval < a)
                {
                    // below
                    interval = 0;
                }
                else if (xval > b)
                {
                    // above
                    interval = x.Length - 2;
                }
                else
                {
                    // nan
                    interval = -1;
                }
            }
            else if (xval == b)
            {
                // Make the interval closed from the right
                interval = x.Length - 2;
            }
            else
            {
                int low, high;

                // Find the interval the coordinate is in
                // (binary search with locality)
                if (xval >= x[interval])
                {
                    low = interval;
                    high = x.Length - 2;
                }
                else
                {
                    low = 0;
                    high = interval;
                }

                if (xval < x[low+1])
                {
                    high = low;
                }

                while (low < high)
                {
                    int mid = (high + low) / 2;

                    if (xval < x[mid])
                    {
                        high = mid;
                    }
                    else if (xval >= x[mid + 1])
                    {
                        low = mid + 1;
                    }
                    else
                    {
                        low = mid;
                        break;
                    }
                }

                interval = low;
            }

            return interval;
        }

        /// <summary>
        /// Evaluate polynomial in a single interval.
        /// </summary>
        private static double EvaluatePoly1(double s, double[,] c, int ci)
        {
            double res = 0.0;
            double z = 1.0;

            int cshape = c.GetLength(0);

            for (int kp = 0; kp < cshape; kp++)
            {
                res += c[cshape - kp - 1, ci] * z;

                // compute x**max(k-dx, 0)
                if ((kp < cshape - 1) && (kp >= 0))
                {
                    z *= s;
                }
            }

            return res;
        }
    }
}
