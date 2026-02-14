#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "stl_parser.h"
#include "test.h"

namespace testutil 
{

void writeAsciiTriangle(const fs::path& file_path) {
    
    std::ofstream out(file_path);
    expect(out.is_open(), "Unable to create ASCII STL fixture.");

    out << "solid unit_triangle\n";
    out << "  facet normal 0 0 1\n";
    out << "    outer loop\n";
    out << "      vertex 0 0 0\n";
    out << "      vertex 1 0 0\n";
    out << "      vertex 0 1 0\n";
    out << "    endloop\n";
    out << "  endfacet\n";
    out << "endsolid unit_triangle\n";
}

void writeBinaryTriangle(const fs::path& file_path) {
#pragma pack(push, 1)
    struct BinTri {
        float n[3];
        float v1[3];
        float v2[3];
        float v3[3];
        uint16_t attr;
    };
#pragma pack(pop)

    std::ofstream out(file_path, std::ios::binary);
    expect(out.is_open(), "Unable to create binary STL fixture.");

    char header[80] = {};
    const std::string hdr = "binary_triangle_fixture";
    std::copy(hdr.begin(), hdr.end(), header);
    out.write(header, sizeof(header));

    const uint32_t count = 1;
    out.write(reinterpret_cast<const char*>(&count), sizeof(count));

    BinTri tri{};
    tri.n[2] = 1.0f;
    tri.v1[0] = 0.0f;
    tri.v1[1] = 0.0f;
    tri.v1[2] = 0.0f;
    tri.v2[0] = 1.0f;
    tri.v2[1] = 0.0f;
    tri.v2[2] = 0.0f;
    tri.v3[0] = 0.0f;
    tri.v3[1] = 1.0f;
    tri.v3[2] = 0.0f;
    tri.attr = 0;

    out.write(reinterpret_cast<const char*>(&tri), sizeof(tri));
}

void checkSingleTriangle(const std::vector<geometry::triangle>& mesh, const std::string& fixture_name) {
    expect(mesh.size() == 1, fixture_name + ": expected 1 triangle.");
    const geometry::triangle& t = mesh[0];

    // stl reader itself
    expect(near(t.verts[0].x, 0.0) && near(t.verts[0].y, 0.0) && near(t.verts[0].z, 0.0), fixture_name + ": unexpected vertex 0.");
    expect(near(t.verts[1].x, 1.0) && near(t.verts[1].y, 0.0) && near(t.verts[1].z, 0.0), fixture_name + ": unexpected vertex 1.");
    expect(near(t.verts[2].x, 0.0) && near(t.verts[2].y, 1.0) && near(t.verts[2].z, 0.0), fixture_name + ": unexpected vertex 2.");

    // compute in tri
    expect(near(t.norm.x, 0.0) && near(t.norm.y, 0.0) && near(t.norm.z, 1.0), fixture_name + ": unexpected computed normal.");
    expect(near(t.bnds.min.x, 0.0) && near(t.bnds.min.y, 0.0) && near(t.bnds.min.z, 0.0), fixture_name + ": unexpected bounding-box minimum.");
    expect(near(t.bnds.max.x, 1.0) && near(t.bnds.max.y, 1.0) && near(t.bnds.max.z, 0.0), fixture_name + ": unexpected bounding-box maximum.");
}

} // namespace

int main() {
    const testutil::fs::path temp_dir = testutil::makeTempDir("stl_parser_test");
    
    const testutil::fs::path ascii_file = temp_dir / "triangle_ascii.stl";
    testutil::writeAsciiTriangle(ascii_file);
    const auto ascii_mesh = stl::stlReader::load(ascii_file.string());
    testutil::checkSingleTriangle(ascii_mesh, "ASCII fixture");

    const testutil::fs::path binary_file = temp_dir / "triangle_binary.stl";
    testutil::writeBinaryTriangle(binary_file);
    const auto binary_mesh = stl::stlReader::load(binary_file.string());
    testutil::checkSingleTriangle(binary_mesh, "Binary fixture");

    testutil::cleanupDir(temp_dir);
    std::cout << "[PASS] STL parser unit tests passed.\n";
    return 0;
}
