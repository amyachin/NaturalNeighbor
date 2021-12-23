using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace NaturalNeighbor.Internal
{
    internal static class TestUtils
    {

        public static NodeWeight[] ComputeReferenceNeighborWeights(Interpolator2d interpolator, double x, double y)
        {
            var pt = new Vector2((float) x, (float) y);

            SearchContext context = new SearchContext();
            var impl = interpolator.GetImplementation();
            SubDiv2D_Immutable immutable = impl.ToImmutable();
            var envelope = SubDiv2D_Immutable.CreateEnvelope(immutable, context, pt);

            var neighbors = new HashSet<NodeId>();
            var f = SubDiv2D_Immutable.SynthesizeFacet(immutable, context, pt, neighbors);

            var facets = impl.GetVoronoiFacets(neighbors).ToDictionary(it => it.Id);
            var intersectionHelper = new ConvexPolygonIntersectionHelper();
            List<Vector2> tmp = new List<Vector2>();

            double sum = 0.0;
            List<NodeWeight> weights = new List<NodeWeight>(envelope.Count);

            foreach (var e in envelope)
            {
                var n = impl.EdgeOrigin(e);
                tmp.Clear();
                double w = 0;

                if (!n.IsSentinel)
                {
                    intersectionHelper.Intersect(f.Vertices, facets[n].Vertices, tmp);
                    w = Utils.ComputePolygonArea2(tmp);
                }
                sum += w;
                weights.Add(new NodeWeight(n, w));
            }

            for (int i = 0; i < weights.Count; ++i)
            {
                var item = weights[i];
                weights[i] = new NodeWeight(item.NodeId, item.Weight / sum);
            }

            return weights.ToArray();
        }

        public static NodeWeight[] ComputeNeighborWeights(Interpolator2d interpolator, double x, double y)
        {
            var impl = interpolator.GetImplementation();
            var context = new SearchContext();
            var locType = impl.Locate(new Vector2((float) x, (float) y), context);

            NodeWeight[] result = null;
            if (locType == PointLocationType.Edge || locType == PointLocationType.Inside)
            {
                var envelope = impl.GetBowyerWatsonEnvelope(x, y, context.Edge);
                double[] weights = impl.GetBarycentricCoordinates(envelope, x, y);
                result = new NodeWeight[envelope.Count];
                for (int i = 0; i < envelope.Count; ++i)
                {
                    result[i] = new NodeWeight(impl.EdgeOrigin(envelope[i]), weights != null ? weights[i]: double.NaN);
                }
            }

            return result ?? new NodeWeight[0];
        }

    }
}
