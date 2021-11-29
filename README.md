#  NaturalNeighbor 
The library focues on spatial interpolation for scattered 2D data, such as GIS data or similar.   



## Classes

`Interpolator2d`

__Description :__

Produces an interpolating function that can be queried at arbitrary location within interpolation boundaries. The underlying triangulation is created once and reused for subsequent queries. Supported interpolation methods :  [*Nearest Neighbor*](https://en.wikipedia.org/wiki/Nearest-neighbor_interpolation), [*Linear*](https://en.wikipedia.org/wiki/Linear_interpolation), and [*Natural Neighbor*](https://en.wikipedia.org/wiki/Natural_neighbor_interpolation) 


__Usage :__
```csharp
using NaturalNeighbor;
...
var interp = Interpolator2d.Create(datapoints);
// interp.Method = InterpolationMethod.Nearest;
// interp.Method = InterpolationMethod.Natural (default)
// interp.Method = InterpolationMethod.Linear;

var z = interp.Lookup(1.3f, 2.1f);
```


`SubDivision2d`

__Description :__

Performs planar subdivision on a set of 2D points represented by collection of `Vector2`.  The class subdivides  a plane into triangles using Delaunay's algorithm, which corresponds to the dual graph of the Voronoi diagram. The subdivision can be used for fast location of points on the plane or building special graphs. 


__Usage :__


```csharp
var subdiv = new SubDivision2d(points);

var nodeId = subdiv.FindNearest(x, y, out var pt);

if (nodeId.HasValue)
{
    Console.WriteLine($"Nearest point: {pt}");
}


foreach(var triangle in subdiv.GetDelaunayTriangles()) 
{
    // Process triangles
    ...
}

foreach(var facet in subdiv.GetVoronoiFacets())
{
    // Process Voronoi facets
}
```

