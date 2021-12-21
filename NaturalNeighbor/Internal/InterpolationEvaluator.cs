using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace NaturalNeighbor.Internal
{
    abstract class InterpolationEvaluator
    {
        public InterpolationEvaluator(ISharedMethods sharedMethods, SearchContext context)
        {
            Context = context;
            SharedMethods = sharedMethods;
        }

        public abstract double Evaluate(double x, double y);

        protected SearchContext Context { get; }
        
        protected ISharedMethods SharedMethods { get; }

        public abstract InterpolationEvaluator Clone();

    }

    sealed class NearestNeighborEvaluator : InterpolationEvaluator
    {
        public NearestNeighborEvaluator(ISharedMethods sharedMethods, SearchContext context) : base(sharedMethods, context)
        {
        }

        public override double Evaluate(double x, double y)
        {
            return SharedMethods.LookupNearest(x, y, Context);
        }

        public override InterpolationEvaluator Clone()
        {
            return new NearestNeighborEvaluator(this.SharedMethods, new SearchContext());
        }
    }

    sealed class PiecewiseLinearEvaluator : InterpolationEvaluator
    {
        public PiecewiseLinearEvaluator(ISharedMethods sharedMethods, SearchContext context) : base(sharedMethods, context)
        {

        }

        public override double Evaluate(double x, double y)
        {
            return SharedMethods.LookupLinear(x, y, Context);
        }

        public override InterpolationEvaluator Clone()
        {
            return new PiecewiseLinearEvaluator(this.SharedMethods, new SearchContext());
        }
    }

    sealed class NaturalNeighborEvaluator : InterpolationEvaluator
    {
        public NaturalNeighborEvaluator(ISharedMethods sharedMethods, SearchContext context) : base(sharedMethods, context)
        {

        }

        public override double Evaluate(double x, double y)
        {
            double result =  SharedMethods.LookupNatural(x, y, Context);
            return result;
        }

        public override InterpolationEvaluator Clone()
        {
            return new NaturalNeighborEvaluator(SharedMethods, new SearchContext());
        }
    }

    interface ISharedMethods
    {
        double LookupNearest(double x, double y, SearchContext context);

        double LookupLinear(double x, double y, SearchContext context);

        double LookupNatural(double x, double y, SearchContext context);

    }

}
