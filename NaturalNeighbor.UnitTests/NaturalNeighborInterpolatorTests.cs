using System;
using System.Numerics;
using Xunit;
using System.Collections.Generic;
using System.Linq;

namespace NaturalNeighbor.UnitTests
{
    public class NaturalNeighborInterpolatorTests 
    {
        [Fact]
        public void NaturalNeighborInterolatorSingleQuery()
        {
            var gridSpec = new GridSpec { OffsetX = -2, OffsetY = -2, Width = 4, Height = 4, Cols = 30, Rows = 30 };

            var points = TestHelpers.CreateGrid(gridSpec).ToList();

            var heights = points.Select(it => SampleFunc(it.X, it.Y)).ToList();
            var interpolator = new Interpolator2d();
            interpolator.Generate(points.ToArray(), points.Select(it => SampleFunc(it.X, it.Y)).ToArray());

            double estZ = SampleFunc(-0.5f, -0.01f);
            double z = interpolator.Lookup(-0.5f, -0.01f);
            Assert.Equal(estZ, z, 2);

        }

        double SampleFunc(float x, float y)
        {
            return (float) (x * Math.Exp(-(x * x) - (y * y)));
        }

        [Fact]
        public void NaturalNeighborMaxErrorOnGrid() 
        {
            var modelSpec = new GridSpec { OffsetX = -2, OffsetY = -2, Width = 4, Height = 4, Rows = 20, Cols = 20 };
            var sampleSpec = new GridSpec { OffsetX = -2, OffsetY = -2, Width = 4, Height = 4, Rows = 50, Cols = 50 };
            var sampleSpec2 = new GridSpec { OffsetX = -2, OffsetY = -2, Width = 4, Height = 4, Rows = 60, Cols = 60};

            //var points = TestHelpers.CreateCircle(new Vector2(0, 0), 1.6f, 200).ToArray();
            var points = TestHelpers.CreateGrid(modelSpec).ToArray();
            var heights = points.Select(it => SampleFunc(it.X, it.Y)).ToArray();

            var interpolator = new Interpolator2d();
            interpolator.Generate(points, heights, 0.5);

            TestHelpers.CalculateErrors(SampleFunc, (float x, float y) => interpolator.Lookup(x, y), sampleSpec, out var maxError, out var stdError);
            Assert.True(maxError < 0.02);
            Assert.True(stdError < 0.004);
//            TestHelpers.SaveGrid(@"C:\Work\matlab-Z-orig.csv", SampleFunc, modelSpec);
//            TestHelpers.SaveGrid(@"C:\Work\matlab-Z.csv", (float x, float y) => interpolator.Lookup(x, y), sampleSpec2);

        }

    }   
}