using System;
using System.Numerics;
using Xunit;
using System.Collections.Generic;
using System.Linq;

namespace NaturalNeighbor.UnitTests
{
    public class Interpolator2dTests 
    {
        [Fact]
        public void NaturalNeighborInterolatorSingleQuery()
        {
            var gridSpec = new GridSpec { OffsetX = -2, OffsetY = -2, Width = 4, Height = 4, Cols = 30, Rows = 30 };

            var points = TestHelpers.CreateGrid(gridSpec).ToList();

            var heights = points.Select(it => SampleFunc(it.X, it.Y)).ToList();
            var interpolator = Interpolator2d.Create(points.ToArray(), points.Select(it => SampleFunc(it.X, it.Y)).ToArray());

            var samplePt = new Vector2(-0.5f, -0.01f);
            double estZ = SampleFunc(samplePt.X, samplePt.Y);

            double z = interpolator.Lookup(samplePt);
            Assert.Equal(estZ, z, 2);

        }


        [Theory]
        [InlineData(InterpolationMethod.Nearest, 0.3, 0.3, 4.0, 4)]
        [InlineData(InterpolationMethod.Natural, 0.3, 0.3, 5.2, 4)]
        [InlineData(InterpolationMethod.Linear, 0.3, 0.3, 5.2, 4)]
        public void TestLookup(InterpolationMethod method, double x, double y, double expectedZ, int precision)
        {
            var modelSpec = new GridSpec { OffsetX = -1, OffsetY = -1, Width = 2, Height = 2, Rows = 3, Cols = 3};
            var points = TestHelpers.CreateGrid(modelSpec).ToArray();
            var heights = new double[points.Length];

            for (int i = 0; i < points.Length; ++i)
            {
                heights[i] = i;
            }

            var interpolator = Interpolator2d.Create(points, heights, 1.0);
            interpolator.Method = method;

            var val = interpolator.Lookup((float)x, (float)y);
            Assert.Equal(expectedZ, val, precision);
        }


        double SampleFunc(float x, float y)
        {
            return (float) (x * Math.Exp(-(x * x) - (y * y)));
        }


        [Fact]
        public void NaturalNeighborOutOfBounds()
        {
            var modelSpec = new GridSpec { OffsetX = -2, OffsetY = -2, Width = 4, Height = 4, Rows = 5, Cols = 5};
            var points = TestHelpers.CreateGrid(modelSpec).ToArray();
            var heights = points.Select(it => SampleFunc(it.X, it.Y)).ToArray();

            var interpolator = Interpolator2d.Create(points, heights, 0.0);
            double result = interpolator.Lookup(-3.0f, -3.0f);
            Assert.True(double.IsNaN(result));
        }


        [Fact]
        public void TestSetBounds()
        {
            var modelSpec = new GridSpec { OffsetX = -2, OffsetY = -2, Width = 4, Height = 4, Rows = 5, Cols = 5 };
            var points = TestHelpers.CreateGrid(modelSpec).ToArray();
            var heights = points.Select(it => SampleFunc(it.X, it.Y)).ToArray();

            var interpolator = Interpolator2d.Create(points, heights);
            Assert.Equal(25, interpolator.NumberOfSamples);

            interpolator.SetBounds(new Vector2(-1.1f, -1.1f), new Vector2(1.1f, 1.1f));
            Assert.Equal(9, interpolator.NumberOfSamples);

            Assert.True(interpolator.MinValue.HasValue);
            Assert.True(interpolator.MaxValue.HasValue);

            Assert.Equal(-1.1f, interpolator.MinValue.Value.X);
            Assert.Equal(-1.1f, interpolator.MinValue.Value.Y);
            Assert.Equal(1.1f, interpolator.MaxValue.Value.X);
            Assert.Equal(1.1f, interpolator.MaxValue.Value.Y);
        }


        [Fact]
        public void NaturalNeighborResidualError() 
        {
            var modelSpec = new GridSpec { OffsetX = -2, OffsetY = -2, Width = 4, Height = 4, Rows = 20, Cols = 20 };
            var sampleSpec = new GridSpec { OffsetX = -2, OffsetY = -2, Width = 4, Height = 4, Rows = 50, Cols = 50 };
            var sampleSpec2 = new GridSpec { OffsetX = -2, OffsetY = -2, Width = 4, Height = 4, Rows = 150, Cols = 150};

            //var points = TestHelpers.CreateCircle(new Vector2(0, 0), 1.6f, 200).ToArray();
            var points = TestHelpers.CreateGrid(modelSpec).ToArray();
            var heights = points.Select(it => SampleFunc(it.X, it.Y)).ToArray();
            var interpolator = Interpolator2d.Create(points, heights, 0.5);

            TestHelpers.CalculateErrors(SampleFunc, (float x, float y) => interpolator.Lookup(x, y), sampleSpec, out var maxError, out var stdError);

            Assert.True(maxError < 0.02);
            Assert.True(stdError < 0.004);

            //TestHelpers.SaveGrid(@"C:\Work\matlab-Z-orig.csv", SampleFunc, modelSpec);

            //interpolator.Method = InterpolationMethod.Nearest;
            //TestHelpers.SaveGrid(@"C:\Work\matlab-Z-nearest.csv", (float x, float y) => interpolator.Lookup(x, y), sampleSpec2);

            //interpolator.Method = InterpolationMethod.Linear;
            //TestHelpers.SaveGrid(@"C:\Work\matlab-Z-linear.csv", (float x, float y) => interpolator.Lookup(x, y), sampleSpec2);

            //interpolator.Method = InterpolationMethod.Natural;
            //TestHelpers.SaveGrid(@"C:\Work\matlab-Z-natural.csv", (float x, float y) => interpolator.Lookup(x, y), sampleSpec2);

        }

    }
}