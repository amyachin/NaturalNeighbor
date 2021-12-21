using System;
using System.Collections.Generic;
using System.Linq;
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
            _interpolator = Interpolator2d.Create(points, heights, 0.5);
        }

        Interpolator2d _interpolator;

        [Theory]
        [InlineData(-1.918367386, -2, 5)]

        public void NaturalNeighbors_GetBowyerWatsonEnvelope(double x, double y, int expectedEdgeCount)
        {
            var refenvelope = _interpolator.CreateReferenceEnvelope(x, y);
            var envelope = _interpolator.CreateEnvelope(x, y);

            Assert.Equal(expectedEdgeCount, refenvelope.Count);
            Assert.Equal(expectedEdgeCount, envelope.Count);

            for (int i = 0; i < expectedEdgeCount; ++i) 
            {
                Assert.Equal(refenvelope[i], envelope[i]);
            }
        }

    }
}
