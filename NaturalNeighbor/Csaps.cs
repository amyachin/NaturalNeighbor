using System;
using System.Collections.Generic;
using NaturalNeighbor.Internal;

namespace NaturalNeighbor
{
    /// <summary>
    /// C# port of matlab and scipy's csaps methods.
    /// </summary>
    /// <remarks>See https://github.com/espdev/csaps </remarks>
    public static class Csaps
    {
        /// <summary>
        /// Compute the smoothing spline for the given inputs
        /// </summary>
        /// <param name="x">X values</param>
        /// <param name="y">Y values</param>
        /// <param name="weights">Optional weights</param>
        /// <param name="p">Optional smoothing factor between 0 and 1. 
        /// When P is 0, the smoothing spline is the least-squares straight line fit
        /// to the data, while, at the other extreme, i.e., when P is 1, it is the
        /// `natural' or variational cubic spline interpolant. The transition region
        /// between these two extremes is usually only a rather small range of values
        /// for P and its location strongly depends on the data sites.It is in this
        /// small range that P is chosen when it is not supplied, or when an empty
        /// P or a negative P is input.</param>
        /// <param name="normalizedSmooth">If True, the smooth parameter is normalized such that results are invariant to xdata range and less sensitive to nonuniformity of weights and xdata clumping</param>
        /// <returns>A Smoothing spline which can be evaluated at any interval</returns>
        public static ISmoothingSpline Compute(
            double[] x, 
            double[] y,
            double[] weights = null,
            double? p = null,
            bool normalizedSmooth = false)
        {
            return new CubicSmoothingSpline(x, y, weights, p, normalizedSmooth);
        }
    }
}
