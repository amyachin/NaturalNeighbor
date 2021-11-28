using System;
using System.Numerics;
using Xunit;
using System.Collections.Generic;

namespace NaturalNeighbor.UnitTests
{
    using NaturalNeighbor.Internal;

    public class IntersectionTests
    {
        [Theory]
        [InlineData("10,-10; 0,20; -10,-10", "10,10; -10,10; 0,-20", 6)]
        [InlineData("10,-10; 0,20; -10,-10; 0,-10", "10,10; -10,10; -10,-10; 10,-10", 5 )]
        [InlineData("10,-10; 0,20; -10,-10", "10,10; -10,10; 10,-10", 4)]
        [InlineData("0.5,0.800000012; -0.200000003,0.5; 0.5,-0.200000003; 0.800000012,0.5", "0.5,0.5; 0.5,1.5; -0.5,1.5; -0.5,0.5", 3)]
        [InlineData("0.5,0.800000012; -0.200000003,0.5; 0.5,-0.200000003; 0.800000012,0.5", "1.5,-0.5; 1.5,0.5; 0.5,0.5; 0.5,-0.5", 3)]
        [InlineData("0.5,0.800000012; -0.200000003,0.5; 0.5,-0.200000003; 0.800000012,0.5", "0.5,-0.5; 0.5,0.5; -0.5,0.5; -0.5,-0.5;", 3)]
        [InlineData("0.5,0.800000012; -0.200000003,0.5; 0.5,-0.200000003; 0.800000012,0.5", "1.5,0.5; 1.5,1.5; 0.5,1.5; 0.5,0.5", 3)]
        [InlineData("-157533.8125,165020.1406;-148651.3594,167622.8281;-156574.6875,182498.0781;-157533.8125,182712.2656;-157865.2188,182498.0781;-160603,167622.8281",
            "-142852.3438,167622.8281;-157533.8125,167622.8281;-157533.8125,152747.5625;-142852.3438,152747.5625", 3)]

        public void TestPolygonIntersection(string data1, string data2, int expectedCount)
        {
            var poly1 = ParseArguments(data1);
            var poly2 = ParseArguments(data2);

            var helper = new ConvexPolygonIntersectionHelper() {RemoveDuplicates = true, DistanceEpsilon = 0.002 };
            List<Vector2> results = new List<Vector2>();
            helper.Intersect(poly1, poly2, results);
            Assert.Equal(expectedCount, results.Count);
        }


        static readonly char[] _separators = new char[] { ';' };

        static Vector2[] ParseArguments(string data)
        {
            List<Vector2> list = new List<Vector2>();

            var parts = data.Split(_separators, StringSplitOptions.RemoveEmptyEntries );

            for (int i = 0; i < parts.Length; ++i)
            {
                string txt = parts[i];
                int idx = txt.IndexOf(',');
                if (idx != -1)
                {
                    var x = float.Parse(txt.Substring(0, idx));
                    var y = float.Parse(txt.Substring(idx + 1));

                    list.Add(new Vector2(x, y));
                }
            }

            return list.ToArray();
        }

    }

}