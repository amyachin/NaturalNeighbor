using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NaturalNeighbor.UnitTests
{
    static class TestFunctions
    {
        public static double SampleFunc(float x, float y)
        {
            return (float) (x * Math.Exp(-(x * x) - (y * y)));
        }
    }
}
