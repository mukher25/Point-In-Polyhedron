# Point-in-Polyhedron Classification

C++ implementation of a point in polyhedron algorithm.

This project loads STL geometry, builds an acceleration structure, and classifies query points as:

- `0` -> Outside
- `1` -> Inside
- `2` -> Boundary

It also supports a basic interior volume hex meshing and VTK export for visualization.