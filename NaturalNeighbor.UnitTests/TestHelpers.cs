using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading.Tasks;
using System.Numerics;
namespace NaturalNeighbor.UnitTests
{
    static class TestHelpers
    {
        internal static IEnumerable<Vector2> CreateGrid(GridSpec spec) 
        {
            return CreateGridEx(spec).Select(it => it.Item3);
        }

        internal static IEnumerable<(int, int, Vector2)> CreateGridEx(
            float offsetX, float offsetY, 
            float width,  float height, 
            int rows, int cols) 
        {
            float scaleX = width / (cols - 1);
            float scaleY = height / (rows - 1);

            for (int i = 0; i < rows; ++i)
            {
                var y = offsetY +  i * scaleX;

                for (int j = 0; j < cols; ++j)
                {
                    var x = offsetX + j * scaleY;
                    yield return (i, j, new Vector2(x, y));
                }
            }
        }

        internal static IEnumerable<(int, int, Vector2)> CreateGridEx(GridSpec spec)
        {
            return CreateGridEx(spec.OffsetX, spec.OffsetY, spec.Width, spec.Height, spec.Rows, spec.Cols);
        }


        internal static IEnumerable<Vector2> CreateCircle(
            Vector2 center, float radius, int count) 
        {
            float angularStep = (float)(Math.PI * 2 / count);

            for(int i = 0; i < count; ++i)
            {
                float t = angularStep * i;
                var x = (float)Math.Cos(t) * radius;
                var y = (float)Math.Sin(t) * radius;

                yield return new Vector2(x, y);
            }
        }


        internal static void CalculateErrors(Func<float, float, double> target, Func<float, float, double> model, GridSpec gridSpec, out double maxError, out double stdError)
        {
            var samples = CreateGrid(gridSpec);
            CalculateErrors(target, model, samples, out maxError, out stdError); 
        }

        internal static void CalculateErrors(Func<float, float, double> target, Func<float, float, double> model, IEnumerable<Vector2> samples, out double maxError, out double stdError)
        {
            double sum2 = 0.0;
            double max = 0.0;


            int count = 0;
            foreach (var it in samples)
            {
                var v1 = target(it.X, it.Y);
                var v2 = model(it.X, it.Y);

                var delta = v1 - v2;
                sum2 += delta * delta;
                max = Math.Max(max, Math.Abs(delta));
                count++;
            }

            maxError = max;
            stdError = Math.Sqrt(sum2 / (count - 1));
        }

        internal static void SaveGrid(string filename, Func<float, float, double> func, GridSpec spec)
        {
            var sequence = CreateGridEx(spec);
            using (var writer = File.CreateText(filename))
            {
                writer.WriteLine("col\trow\tX\tY\tZ");

                foreach (var it in sequence)
                {
                    var z = func(it.Item3.X, it.Item3.Y);
                    writer.WriteLine($"{it.Item1}\t{it.Item2}\t{it.Item3.X}\t{it.Item3.Y}\t{z}");
                }

            }

        }
    }

    public class GridSpec
    {
        public float OffsetX { get; set; }
        public float OffsetY { get; set; }
        public float Width { get; set; }
        public float Height { get; set; }
        public int Rows { get; set; }
        public int Cols { get; set; }
    }
}
