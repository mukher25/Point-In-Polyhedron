# Point-in-Polyhedron (C++)

Minimal C++17 tool for STL-based point classification.

Classification labels:
- `0` = `Outside`
- `1` = `Inside`
- `2` = `Boundary`

## Requirements
- CMake 3.15+
- C++17 compiler (`g++`, `clang++`, or MSVC)

## Build
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

## Quick Usage

Generate and classify random points:
```bash
./build/point_query --stl <mesh.stl> gen --n 100000 --results-out results.csv
```

Classify points from CSV (`x,y,z`):
```bash
./build/point_query --stl <mesh.stl> classify --in points.csv --out results.csv
```

Generate interior hex mesh (VTK output):
```bash
./build/point_query --stl <mesh.stl> mesh --mesh-res 16 --mesh-depth 6 --vtk-mesh mesh.vtk
```

Optional visualization outputs:
- `--vtk-stl <file.vtk>`
- `--vtk-boxes <file.vtk>`
- `--vtk-points <file.vtk>`
- `--vtk-mesh <file.vtk>`

## Tests
```bash
ctest --test-dir build --output-on-failure
```

## Technical Details
Algorithm selection, implementation details, complexity analysis, case studies, testing, and references are documented in:

- `docs/SOLUTION.md`
