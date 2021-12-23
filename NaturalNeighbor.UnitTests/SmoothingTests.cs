using MathNet.Numerics;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Xunit;

namespace NaturalNeighbor.UnitTests
{
    public class SmoothingTests
    {
        /// <summary>
        /// X Data which matches csaps unit test data
        /// </summary>
        private static double[] TestX = new double[] {
            -5.0       , -4.58333333, -4.16666667, -3.75      , -3.33333333,
            -2.91666667, -2.5       , -2.08333333, -1.66666667, -1.25      ,
            -0.83333333, -0.41666667,  0.0       ,  0.41666667,  0.83333333,
             1.25      ,  1.66666667,  2.08333333,  2.5       ,  2.91666667,
             3.33333333,  3.75      ,  4.16666667,  4.58333333,  5.0        };

        /// <summary>
        /// X Data which matches csaps unit test data
        /// </summary>
        private static double[] TestY = new double[] {
            0.23720047, 0.06960935, 0.05735217, 0.10676731, 0.27933082,
            0.37503917, 0.5972338 , 0.63530492, 0.80585238, 0.91487174,
            1.05915376, 1.2009965 , 0.94251649, 0.94453779, 0.92445043,
            0.91572414, 0.82412415, 0.70100456, 0.59727372, 0.41348136,
            0.30175591, 0.26063531, 0.14245623, 0.07237209, 0.09020902 };

        [Fact]
        public void SmoothingSpline1D()
        {
            const int ptCnt = 150;
            const double smoothing = 0.85;
            ISmoothingSpline spline = Csaps.Compute(TestX, TestY, p: smoothing);
            double[] smoothed = spline.Evaluate(Generate.LinearSpaced(ptCnt, -5.0, 5.0));

            Assert.Equal(spline.SmoothingFactor, smoothing);
            Assert.Equal(ptCnt, smoothed.Length);
        }
    }
}
