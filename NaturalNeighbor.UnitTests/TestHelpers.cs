using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading.Tasks;
using System.Numerics;
using Xunit;

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


        internal static IEnumerable<Vector3> ReadTestData(string filename)
        {
            using (var txt = File.OpenText(filename))
            {
                txt.ReadLine(); // Skip columns

                var s = txt.ReadLine();
                while (s != null)
                {
                    var parts = s.Split('\t');
                    if (parts.Length == 3 && 
                        double.TryParse(parts[0], out var x) &&
                        double.TryParse(parts[1], out var y) &&
                        double.TryParse(parts[2], out var z))
                    {
                        yield return new Vector3((float) x, (float) y, (float) z);
                    }

                    s = txt.ReadLine();
                }
            }
        }

        internal static void ComparePolygons(IList<int> polygon1, IList<int> polygon2)
        {
            Assert.Equal(polygon1.Count, polygon2.Count);


            int edgeId = polygon1[0];
            int offset = polygon2.IndexOf(edgeId);
            
            Assert.NotEqual(-1, offset);

            int cnt = polygon1.Count;
            for (int i = 0; i < cnt; ++i)
            {
                int i1 = (i + offset) % cnt;
                Assert.Equal(polygon1[i], polygon2[i1]);
            }
        }


        internal static void CompareNodeWeights(IReadOnlyList<Internal.NodeWeight> referenceWeights, IReadOnlyList<Internal.NodeWeight> weights, int precision)
        {
            var poly1 = referenceWeights.Select(it => (int) it.NodeId).ToArray();
            var poly2 = weights.Select(it => (int) it.NodeId).ToArray();

            ComparePolygons(poly1, poly2);
            var expectedWeights = RemapWeights(poly1, poly2, referenceWeights.Select(it => it.Weight).ToArray());

            for (int i = 0; i < weights.Count; ++i)
            {
                Assert.Equal(expectedWeights[i], weights[i].Weight, precision);
            }

        }



        internal static IReadOnlyList<double> RemapWeights(IList<int> source, IList<int> dest, IList<double> weights)
        {
            Assert.Equal(source.Count, dest.Count);


            int edgeId = source[0];
            int offset = dest.IndexOf(edgeId);

            Assert.NotEqual(-1, offset);

            int cnt = source.Count;
            double[] results = new double[cnt];

            for (int i = 0; i < cnt; ++i)
            {
                int i1 = (i + offset) % cnt;
                results[i1] = weights[i];
            }

            return results;
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

        internal static void CalculateErrors(Func<float, float, double> model, IList<Vector2> samples, IList<double> values, out double maxError, out double stdError)
        {
            double sum2 = 0.0;
            double max = 0.0;


            int count = samples.Count;
            for (int i = 0; i < samples.Count; ++i)
            {
                var v1 = values[i];
                var v2 = model(samples[i].X, samples[i].Y);

                var delta = v1 - v2;
                sum2 += delta * delta;
                max = Math.Max(max, Math.Abs(delta));
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
