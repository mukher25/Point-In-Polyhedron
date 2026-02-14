# Technical Solution Writeup

## 1. Problem Statement

The goal is to classify a query point relative to a triangulated surface from STL as one of:

- `Outside`
- `Inside`
- `Boundary`

The current implementation is done entirely in c++ with no external dependencies, only c++ standard template libraries. The core idea here is to use a ray tracing logic (odd-even hits) to detect if a point is inside or outside. To accelerate the formulation for large number of input triangle (~1M triangles), a custom lightweight octree structure with a voxel fill step is utilized. Additionally, for the purposes of a demonstration a very basic hex mesher with refinement near the booundary surfaces is also implemented. Although not explicitly stated in the problem description, an assumption is made that the input STL is watertight and manifold (the CLI will throw an error if it isn't).

## 2. Point In Polyhedron Algorithm
The algorithm of choice implemented here is ray casting. 
