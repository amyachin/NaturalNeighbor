namespace NaturalNeighbor
{
    public struct NodeId
    {
        public NodeId(int value)
        {
            _value = value;
        }

        public static explicit operator int(NodeId id)
        {
            return id._value;
        }

        public override string ToString()
        {
            return $"NodeId={_value}";
        }

        readonly int _value;
    }

}