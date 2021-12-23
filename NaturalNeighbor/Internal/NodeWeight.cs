using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NaturalNeighbor.Internal
{
    internal struct NodeWeight
    {
        public NodeWeight(NodeId nodeId, double weight)
        {
            NodeId = nodeId;
            Weight = weight;
        }

        public NodeId NodeId { get; }

        public double Weight { get; }
    }
}
