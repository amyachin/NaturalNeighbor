using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NaturalNeighbor.Internal
{
    internal class PriorityQueue<T>
    {
        public PriorityQueue(IComparer<T> comparer, int capacity)
        {
            _comparer = comparer;
            _items = new List<T>(capacity);
        }

        public PriorityQueue(IComparer<T> comparer)
        {
            _comparer = comparer;
            _items = new List<T>();
        }

        public PriorityQueue(IComparer<T> comparer, IEnumerable<T> items)
        {
            _comparer = comparer;
            _items = new List<T>(items);
            MakeHeap();
        }

        public void Enqueue(T value)
        {
            _items.Add(default(T));
            PushHeap(_items.Count - 1, 0, value);
        }

        public T Dequeue()
        {
            if (_items.Count == 0)
            {
                throw new InvalidOperationException("Cannot remove elements from an empty queue");
            }

            T result = _items[0];

            int last = _items.Count - 1;

            if (last > 0)
            {
                var value = _items[last];
                _items.RemoveAt(last);
                Heapify(0, value);
                return result;
            }

            _items.RemoveAt(last);
            return result;
        }

        public T Top()
        {
            return _items[0];
        }

        public void Clear()
        {
            _items.Clear();
        }

        public int Count => _items.Count;

        public bool IsEmpty => _items.Count == 0;

        private void MakeHeap()
        {
            if (_items.Count < 2)
            {
                return;
            }

            int len = _items.Count;
            int parent = (len - 2) / 2;
            
            while (true)
            {
                var value = _items[parent];
                Heapify(parent, value);

                if (parent == 0)
                {
                    break;
                }

                parent--;
            }
        }

        private void Heapify(int index, T value)
        {
            int topIndex = index;
            int child = index;

            int len = _items.Count;
            int lim = (_items.Count - 1) / 2;

            while (child < lim)
            {
                child = 2 * (child + 1);
                if (_comparer.Compare(_items[child], _items[child - 1]) > 0)
                {
                    child--;
                }

                _items[index] = _items[child];
                index = child;
            }

            if ((len & 1) == 0 && child == (len - 2) / 2)
            {
                child = 2 * (child + 1);
                _items[index] = _items[child - 1];
                index = child - 1;
            }

            PushHeap(index, topIndex, value);
        }

        private void PushHeap(int index, int topIndex, T value )
        {
            int parent = (index - 1) / 2;
            while (index > topIndex && _comparer.Compare(_items[parent], value) > 0)
            {
                _items[index] = _items[parent];
                index = parent;
                parent = (index - 1) / 2;
            }
            _items[index] = value;
        }

        private readonly IComparer<T> _comparer;

        private readonly List<T> _items;

    }
}
