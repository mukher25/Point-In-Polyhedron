#ifndef STL_PARSER_H
#define STL_PARSER_H

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cstdint>
#include <map>
#include <tuple>

#include "geometry.h"
#include "log.h"

namespace stl
{
    using geometry::triangle;

    class stlReader{

    public:
        
        static std::vector<triangle> load(const std::string& filepath){
            std::ifstream file(filepath, std::ios::binary);
            if (!file.is_open())
                log(LogLevel::ERROR, "Unable to open STL file: " + filepath);

            if (_isASCII(file))
                return _parseASCII(file);
            else
                return _parseBinary(file);
        }

        // validation to ensure assumptions are met
        struct pntCompare{
            bool operator()(const geometry::pnt3d& a, const geometry::pnt3d& b) const {
                if (std::abs(a.x - b.x) > constants::WELD_TOL) return a.x < b.x;
                if (std::abs(a.y - b.y) > constants::WELD_TOL) return a.y < b.y;
                if (std::abs(a.z - b.z) > constants::WELD_TOL) return a.z < b.z;   
                return false;      
            }
        };


        static bool validateMesh(const std::vector<triangle>& mesh){
            // probably can do like a hash map or something, will see later if slow
            std::map<geometry::pnt3d, int, pntCompare> uniqueVerts;
            int next_id = 0;

            struct TriInds {int v[3];};
            std::vector<TriInds> indxMesh;  indxMesh.reserve(mesh.size());

            for (const auto& tri : mesh){
                TriInds idx;
                for (size_t i=0; i<3; ++i){
                    auto it = uniqueVerts.find(tri.verts[i]);
                    if (it == uniqueVerts.end()){
                        uniqueVerts[tri.verts[i]] = next_id;
                        idx.v[i] = next_id;
                        next_id++;
                    }
                    else{
                        idx.v[i] = it->second;
                    }
                }
                indxMesh.push_back(idx);
            }

            std::map<std::pair<int, int>, int> edge_counts;
            for (const auto& t : indxMesh) {
                int edges[3][2] = {
                    {t.v[0], t.v[1]},
                    {t.v[1], t.v[2]},
                    {t.v[2], t.v[0]}
                };

                for (auto& e : edges) {
                    int v1 = std::min(e[0], e[1]);
                    int v2 = std::max(e[0], e[1]);
                    edge_counts[{v1, v2}]++;
                }
            }

            int open_edges = 0;
            int non_manifold_edges = 0;

            for (const auto& pair : edge_counts) {
                int count = pair.second;
                if (count == 1) open_edges++;
                else if (count > 2) non_manifold_edges++;
            }

            if (open_edges > 0 || non_manifold_edges > 0) {
                log(LogLevel::WARNING, "Mesh Validation Failed: " + 
                    std::to_string(open_edges) + " open edges (holes), " + 
                    std::to_string(non_manifold_edges) + " non-manifold edges.");
                return false;
            }
            log(LogLevel::INFO, "Mesh Validation Passed: Watertight and Manifold.");
            return true;
        };
    private:
        
        // rough check to see the type of file
        static bool _isASCII(std::ifstream& file) {
            char header[6] = {0};
            file.read(header, 5);
            if (!file.good()) {
                file.clear();
                file.seekg(0, std::ios::beg);
                return false;
            }

            const bool starts_with_solid = (std::string(header) == "solid");
            if (!starts_with_solid) {
                file.clear();
                file.seekg(0, std::ios::beg);
                return false;
            }

            // Binary STL may also start with "solid". Detect it using expected file size.
            file.clear();
            file.seekg(80, std::ios::beg);
            uint32_t tri_count = 0;
            file.read(reinterpret_cast<char*>(&tri_count), sizeof(tri_count));
            if (!file.good()) {
                file.clear();
                file.seekg(0, std::ios::beg);
                return true;
            }

            file.clear();
            file.seekg(0, std::ios::end);
            const std::streamoff size = file.tellg();
            const std::streamoff expected_binary =
                static_cast<std::streamoff>(84) + static_cast<std::streamoff>(50) * tri_count;

            file.clear();
            file.seekg(0, std::ios::beg); // reset for parser
            return size != expected_binary;
        }

        static std::vector<triangle> _parseASCII(std::ifstream& file){

            std::vector<triangle> mesh;
            std::string line, word;

            triangle tri;
            size_t vid = 0;

            while (std::getline(file, line)){
                std::stringstream ss(line);
                ss >> word;

                if (word == "vertex"){
                    if (vid < 3){
                        ss >> tri.verts[vid].x >> tri.verts[vid].y >> tri.verts[vid].z;
                        vid++;
                    }
                }
                else if (word == "endfacet"){
                    if (vid == 3){
                        tri.compute();
                        mesh.push_back(tri);
                    }
                    vid = 0;
                }
            }
            return mesh;
        }
        static std::vector<triangle> _parseBinary(std::ifstream& file){

            // ensure no padding - memory alignment
            #pragma pack(push, 1)
            struct binTri{
                float n[3], v1[3], v2[3], v3[3];
                uint16_t attr;
            };
            #pragma pack(pop)

            file.seekg(80, std::ios::beg); // header
            uint32_t count;
            file.read(reinterpret_cast<char*>(&count), 4);

            std::vector<triangle> mesh;
            mesh.reserve(count);

            for (uint32_t i=0; i<count; ++i){
                binTri b_tri;
                if(!file.read(reinterpret_cast<char*>(&b_tri), 50))
                    log(LogLevel::ERROR, "Corrupted binary STL file.");

                triangle tri;
                tri.verts[0] = {(double)b_tri.v1[0], (double)b_tri.v1[1], (double)b_tri.v1[2]};
                tri.verts[1] = {(double)b_tri.v2[0], (double)b_tri.v2[1], (double)b_tri.v2[2]};
                tri.verts[2] = {(double)b_tri.v3[0], (double)b_tri.v3[1], (double)b_tri.v3[2]};

                tri.compute();
                mesh.push_back(tri);
            }
            return mesh;
        }
    };
}
#endif
