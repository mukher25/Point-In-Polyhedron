#ifndef VTK_H
#define VTK_H

#include <vector>
#include <string>
#include <fstream>

#include "geometry.h"
#include "log.h"
#include "classifier.h"

namespace vtk
{
    using namespace geometry;
    struct PointStatus {
        geometry::pnt3d p;
        classifier::Location loc;
    };

    class vtkWriter{
    public:
        static void checkSTL(const std::string&filepath, const std::vector<triangle>& mesh){
            std::ofstream file(filepath);
            if (!file.is_open()) {
                log(LogLevel::ERROR, "Could not open VTK file for writing: " + filepath);
            }

            file << "# vtk DataFile Version 3.0\n";
            file << "STL Mesh Export\n";
            file << "ASCII\n";
            file << "DATASET POLYDATA\n";

            file << "POINTS " << mesh.size() * 3 << " double\n";
            for (const auto& tri : mesh) {
                for (size_t i = 0; i < 3; ++i) {
                    file << tri.verts[i].x << " " << tri.verts[i].y << " " << tri.verts[i].z << "\n";
                }
            }

            file << "\nPOLYGONS " << mesh.size() << " " << mesh.size() * 4 << "\n";
            for (size_t i = 0; i < mesh.size(); ++i) {
                size_t offset = i * 3;
                file << "3 " << offset << " " << offset + 1 << " " << offset + 2 << "\n";
            }

            file << "\nCELL_DATA " << mesh.size() << "\n";
            file << "NORMALS CellNormals double\n";
            for (const auto& tri : mesh) {
                file << tri.norm.x << " " << tri.norm.y << " " << tri.norm.z << "\n";
            }

            file.close();
            log(LogLevel::INFO, "VTK STL file written: " + filepath);
        }

        static void checkBoxes(const std::string&filepath, const std::vector<aabb>& boxes){
            std::ofstream file(filepath);
            if (!file.is_open()) {
                log(LogLevel::ERROR, "Could not open VTK file for writing: " + filepath);
            }

            file << "# vtk DataFile Version 3.0\n";
            file << "Octree Boxes\n";
            file << "ASCII\n";
            file << "DATASET UNSTRUCTURED_GRID\n";

            file << "POINTS " << boxes.size() * 8 << " double\n";
            for (const auto& b : boxes) {
                // The 8 corners of a generic AABB
                file << b.min.x << " " << b.min.y << " " << b.min.z << "\n"; // 0
                file << b.max.x << " " << b.min.y << " " << b.min.z << "\n"; // 1
                file << b.max.x << " " << b.max.y << " " << b.min.z << "\n"; // 2
                file << b.min.x << " " << b.max.y << " " << b.min.z << "\n"; // 3
                file << b.min.x << " " << b.min.y << " " << b.max.z << "\n"; // 4
                file << b.max.x << " " << b.min.y << " " << b.max.z << "\n"; // 5
                file << b.max.x << " " << b.max.y << " " << b.max.z << "\n"; // 6
                file << b.min.x << " " << b.max.y << " " << b.max.z << "\n"; // 7
            }

            file << "\nCELLS " << boxes.size() << " " << boxes.size() * 9 << "\n";
            for (size_t i = 0; i < boxes.size(); ++i) {
                size_t o = i * 8;
                // VTK Hexahedron node ordering matches the order we printed above
                file << "8 " << o << " " << o+1 << " " << o+2 << " " << o+3 << " " 
                             << o+4 << " " << o+5 << " " << o+6 << " " << o+7 << "\n";
            }

            file << "\nCELL_TYPES " << boxes.size() << "\n";
            for (size_t i = 0; i < boxes.size(); ++i) file << "12\n"; // 12 = VTK_HEXAHEDRON

            file.close();
            log(LogLevel::INFO, "VTK Octree file written: " + filepath);
        }

        static void checkPoints(const std::string& filepath, const std::vector<PointStatus>& points){
            std::ofstream file(filepath);
            if (!file.is_open()) {
                log(LogLevel::ERROR, "Could not open VTK file for writing: " + filepath);
            }

            file << "# vtk DataFile Version 3.0\n";
            file << "Test Points\n";
            file << "ASCII\n";
            file << "DATASET POLYDATA\n";

            file << "POINTS " << points.size() << " double\n";
            for (const auto& pt : points) {
                file << pt.p.x << " " << pt.p.y << " " << pt.p.z << "\n";
            }

            file << "\nVERTICES " << points.size() << " " << points.size() * 2 << "\n";
            for (size_t i = 0; i < points.size(); ++i) {
                file << "1 " << i << "\n";
            }

            file << "\nPOINT_DATA " << points.size() << "\n";
            file << "SCALARS Status int 1\n";
            file << "LOOKUP_TABLE default\n";

            for (const auto& pt : points) {
                file << static_cast<int>(pt.loc) << "\n";
            }

            file.close();
            log(LogLevel::INFO, "VTK Points file written: " + filepath);
        }

        static void checkHex(const std::string&filepath, const std::vector<hex>& mesh){
            std::ofstream file(filepath);
            if (!file.is_open()) {
                log(LogLevel::ERROR, "Could not open VTK file for writing: " + filepath);
            }

            file << "# vtk DataFile Version 3.0\n";
            file << "Hex Interior Mesh\n";
            file << "ASCII\n";
            file << "DATASET UNSTRUCTURED_GRID\n";

            file << "POINTS " << mesh.size() * 8 << " double\n";
            for (const auto& h : mesh) {
                for (const auto& v : h.verts) {
                    file << v.x << " " << v.y << " " << v.z << "\n";
                }
            }

            file << "\nCELLS " << mesh.size() << " " << mesh.size() * 9 << "\n";
            for (size_t i = 0; i < mesh.size(); ++i) {
                size_t base = i * 8;
                file << "8 " << base << " " << base+1 << " " << base+2 << " " << base+3 << " "
                             << base+4 << " " << base+5 << " " << base+6 << " " << base+7 << "\n";
            }

            file << "\nCELL_TYPES " << mesh.size() << "\n";
            for (size_t i = 0; i < mesh.size(); ++i) {
                file << "12\n";
            }

            file.close();
            log(LogLevel::INFO, "VTK Hex Mesh with " + std::to_string(mesh.size()) + " hex elements written to " + filepath);
        }
    };
}

#endif