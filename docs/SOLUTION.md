# Technical Solution Writeup

## 1. Problem Statement

The goal is to classify a query point relative to a closed triangulated surface from STL as one of:

- `Outside`
- `Inside`
- `Boundary` (on face/edge/vertex)

The implementation also includes octree-based preprocessing for fast repeated queries and a basic adaptive hex meshing routine for interior volume filling.