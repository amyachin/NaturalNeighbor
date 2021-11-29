using System;
using System.Collections.Generic;
using Xunit;
using System.Linq;
using System.Numerics;

namespace NaturalNeighbor.UnitTests
{
    public class SubDivision2dTests
    {
     
        [Fact]
        public void FindNearestInsideBoundaries()
        {

            var gridSpec = new GridSpec { OffsetX = -2.0f, OffsetY = -2.0f, Width = 4, Height = 4, Cols = 5, Rows = 5 };
            var grid = TestHelpers.CreateGrid(gridSpec);

            var subdivision = new SubDivision2d(grid.ToArray(), 0.2f); 
            var pt = new Vector2(0.3f, 0.3f);

            var insertedNodeId = subdivision.Insert(pt);
            var foundNodeId = subdivision.FindNearest(new Vector2(0.31f, 0.29f), out var foundPt);

            Assert.True(foundNodeId.HasValue);
            Assert.Equal(insertedNodeId, foundNodeId.Value);
            Assert.Equal(pt, foundPt);
        }


        [Fact]
        public void FindNearestOutsideBoundaries()
        {
            var gridSpec = new GridSpec { OffsetX = -2.0f, OffsetY = -2.0f, Width = 4, Height = 4, Cols = 5, Rows = 5 };
            var grid = TestHelpers.CreateGrid(gridSpec);
            var subdivision = new SubDivision2d(grid.ToArray(), 0.2f); 
            var foundNodeId = subdivision.FindNearest(new Vector2(subdivision.Min.X - 1000, subdivision.Max.Y + 1000), out var pt);
            Assert.False(foundNodeId.HasValue);
        }


        [Fact]
        public void LookupSingleVoronoiFaucet()
        {

            var gridSpec = new GridSpec { OffsetX = -2.0f, OffsetY = -2.0f, Width = 4, Height = 4, Cols = 5, Rows = 5 };
            var grid = TestHelpers.CreateGrid(gridSpec);
            var subdivision = new SubDivision2d(grid.ToArray(), 0.2f); 

            var pt = new Vector2(0.3f, 0.3f);
            var nodeId = subdivision.Insert(pt);
            var faucets = subdivision.GetVoronoiFacets(new NodeId[] { nodeId });

            Assert.NotNull(faucets);
            Assert.Single(faucets);
            
            var found = faucets.First();
            Assert.Equal(nodeId, found.Id);
            Assert.Equal(pt, found.Center);
            Assert.Equal(4, found.Vertices.Length);
            Assert.True(found.Area > 0.0);
        }


        [Fact]
        public void TestDelaunayTriangles()
        {
            var gridSpec = new GridSpec { OffsetX = -2.0f, OffsetY = -2.0f, Width = 4, Height = 4, Cols = 3, Rows = 3 };
            var grid = TestHelpers.CreateGrid(gridSpec);

            var subdivision = new SubDivision2d(grid.ToArray(), 0.2f);
            var triangles = subdivision.GetDelaunayTriangles().ToArray();
            Assert.Equal(8, triangles.Length);

        }




    }
}
