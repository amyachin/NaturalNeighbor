using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

using Xunit;

namespace NaturalNeighbor.UnitTests
{
    public class NaturalNeighborGraphTests
    {


        public NaturalNeighborGraphTests()
        {
            var modelSpec = new GridSpec { OffsetX = -2, OffsetY = -2, Width = 4, Height = 4, Rows = 20, Cols = 20 };
            var sampleSpec = new GridSpec { OffsetX = -2, OffsetY = -2, Width = 4, Height = 4, Rows = 50, Cols = 50 };
            var sampleSpec2 = new GridSpec { OffsetX = -2, OffsetY = -2, Width = 4, Height = 4, Rows = 150, Cols = 150 };

            //var points = TestHelpers.CreateCircle(new Vector2(0, 0), 1.6f, 200).ToArray();
            var points = TestHelpers.CreateGrid(modelSpec).ToArray();

            var heights = points.Select(it => TestFunctions.SampleFunc(it.X, it.Y)).ToArray();
            _gridInterpolator = Interpolator2d.Create(points, heights, 0.5);

            var points2 = TestHelpers.ReadTestData("TestData\\ZGrid_filtered.csv").ToArray();

            _gridWithCavitiesInterpolator = Interpolator2d.Create(points2, 80000);
        }

        Interpolator2d _gridInterpolator;
        Interpolator2d _gridWithCavitiesInterpolator;


        [Theory]
        [InlineData(NaturalNeighborTestPattern.Grid, -1.918367386, -2)]
        [InlineData(NaturalNeighborTestPattern.GridWithCavities, 167036.709214, -320947.760376)]

        public void NaturalNeighbors_TestNaturalNeighborWeights(NaturalNeighborTestPattern pattern, double x, double y)
        {

            var interpolator = GetInterpolator(pattern);

            var referenceWeights = Internal.TestUtils.ComputeReferenceNeighborWeights(interpolator, x, y);
            var weights = Internal.TestUtils.ComputeNeighborWeights(interpolator, x, y);

            Assert.Equal(referenceWeights.Length, weights.Length);
            TestHelpers.CompareNodeWeights(referenceWeights, weights, 5);
        }

        private Interpolator2d GetInterpolator(NaturalNeighborTestPattern pattern)
        {
            switch (pattern)
            {
                case NaturalNeighborTestPattern.Grid:
                    return _gridInterpolator;
                case NaturalNeighborTestPattern.GridWithCavities:
                    return _gridWithCavitiesInterpolator;

                default:
                    throw new ArgumentOutOfRangeException(nameof(pattern));
            }

        }

    }


    public enum NaturalNeighborTestPattern
    {
        Grid = 0,
        GridWithCavities = 1
    }
}
