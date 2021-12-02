using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NaturalNeighbor
{
    /// <summary>
    /// Interpolation method
    /// </summary>
    public enum InterpolationMethod
    {
        /// <summary>
        /// Nearest Neighbor interpolation
        /// </summary>
        Nearest = 0,

        /// <summary>
        /// Piecewise linear interpolation
        /// </summary>
        Linear = 1,

        /// <summary>
        /// Natural Neighbor interpolation
        /// </summary>
        Natural = 2,
    }
}
