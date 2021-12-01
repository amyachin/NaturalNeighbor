namespace NaturalNeighbor
{
    /// <summary>
    /// Delaunay graph node identifier
    /// </summary>
    public struct NodeId
    {

        /// <summary>
        /// Constructs new node id
        /// </summary>
        /// <param name="value">node index</param>
        public NodeId(int value)
        {
            _value = value;
        }

        /// <summary>
        /// Returns the underlying ordinal value 
        /// </summary>
        /// <param name="id"></param>
        public static explicit operator int(NodeId id)
        {
            return id._value;
        }

        /// <summary>
        /// String representation of node id
        /// </summary>
        /// <returns>string</returns>
        public override string ToString()
        {
            return $"NodeId={_value}";
        }

        readonly int _value;
    }

}