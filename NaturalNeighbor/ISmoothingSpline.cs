namespace NaturalNeighbor
{
    /// <summary>
    /// Represents a smoothing spline which can be evaluated
    /// </summary>
    public interface ISmoothingSpline
    {
        /// <summary>
        /// The smoothing factor used to generate this spline
        /// </summary>
        double SmoothingFactor { get; }

        /// <summary>
        /// Evaluate the given x data and returned the smoothed results
        /// </summary>
        double[] Evaluate(double[] xi);
    }
}