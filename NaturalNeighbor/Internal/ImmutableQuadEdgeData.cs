using System;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NaturalNeighbor.Internal
{
    internal class ImmutableQuadEdgeData
    {


        public ImmutableQuadEdgeData(int edgeidx)
        {
            next = ImmutableArray.Create(
                edgeidx,
                edgeidx + 3,
                edgeidx + 2,
                edgeidx + 1);

            pts = Empty;
        }

        public ImmutableQuadEdgeData(ImmutableArray<int> next, ImmutableArray<int> pts) 
        {
            this.next = next;
            this.pts = pts;
        }

        static readonly ImmutableArray<int> Empty = ImmutableArray.Create(0, 0, 0, 0);


        public static readonly ImmutableQuadEdgeData Default = new ImmutableQuadEdgeData(Empty, Empty);


        public bool IsFree => next[0] <= 0;

        public readonly ImmutableArray<int> next;
        public readonly ImmutableArray<int> pts;
    }
}
