#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "classifier.h"
#include "test.h"
#include "vtk.h"

namespace testutil 
{
    geometry::triangle makeTri(const geometry::pnt3d& v0, const geometry::pnt3d& v1, const geometry::pnt3d& v2) {
        geometry::triangle t;
        t.verts[0] = v0;
        t.verts[1] = v1;
        t.verts[2] = v2;
        t.compute();
        return t;
    }

    // 1x1x1 cube
    std::vector<geometry::triangle> makeCube(){
        using geometry::pnt3d;
        const pnt3d v000{0,0,0}, v100{1,0,0}, v110{1,1,0}, v010{0,1,0};
        const pnt3d v001{0,0,1}, v101{1,0,1}, v111{1,1,1}, v011{0,1,1};

        std::vector<geometry::triangle> tris;
        tris.reserve(12);

        // two tris per face
        tris.push_back(makeTri(v000, v100, v110));
        tris.push_back(makeTri(v000, v110, v010));

        tris.push_back(makeTri(v001, v111, v101));
        tris.push_back(makeTri(v001, v011, v111));

        tris.push_back(makeTri(v000, v011, v001));
        tris.push_back(makeTri(v000, v010, v011));

        tris.push_back(makeTri(v100, v101, v111));
        tris.push_back(makeTri(v100, v111, v110));

        tris.push_back(makeTri(v000, v001, v101));
        tris.push_back(makeTri(v000, v101, v100));

        tris.push_back(makeTri(v010, v110, v111));
        tris.push_back(makeTri(v010, v111, v011));

        return tris;
    }

    geometry::aabb computeBnds(const std::vector<geometry::triangle>& tris, double margin)
    {
        geometry::aabb bb = tris[0].bnds;
        for (const auto& t : tris){
            bb.min.x = std::min(bb.min.x, t.bnds.min.x);
            bb.min.y = std::min(bb.min.y, t.bnds.min.y);
            bb.min.z = std::min(bb.min.z, t.bnds.min.z);

            bb.max.x = std::max(bb.max.x, t.bnds.max.x);
            bb.max.y = std::max(bb.max.y, t.bnds.max.y);
            bb.max.z = std::max(bb.max.z, t.bnds.max.z);
        }

        const geometry::pnt3d dims = bb.max - bb.min;
        const double max_dim = std::max({dims.x, dims.y, dims.z});
        const double m = max_dim*margin;

        bb.min = bb.min + geometry::pnt3d{-m,-m,-m};
        bb.max = bb.max + geometry::pnt3d{m,m,m};

        return bb;
    }

    void expectLoc(classifier::pntClassifier& clsfr, const geometry::pnt3d& p, classifier::Location expected, const std::string& msg){
        const classifier::Location got = clsfr.classify(p);
        expect(got==expected, msg);
    }
}

int main(){

    const auto tris = testutil::makeCube();
    const geometry::aabb root_bnds = testutil::computeBnds(tris, 0.1);

    // octree
    octree::octreeNode root(root_bnds, 1, 5);
    for (size_t i=0; i<tris.size();++i) root.tri_idx.push_back(i);
    root.subDivide(tris, 0);

    // classifier
    classifier::pntClassifier clsfr(tris, root);


    std::vector<vtk::PointStatus> test_pts;
    test_pts.reserve(11);

    const double d = 1e-6;
    test_pts.push_back({{0.5, 0.5, 0.5}, classifier::Location::Inside});   // inside
    test_pts.push_back({{2.0, 0.5, 0.5}, classifier::Location::Outside});  // outside
    test_pts.push_back({{0.0, 0.5, 0.5}, classifier::Location::Boundary}); // face bc
    test_pts.push_back({{0.0, 0.0, 0.5}, classifier::Location::Boundary}); // edge bc
    test_pts.push_back({{0.0, 0.0, 0.0}, classifier::Location::Boundary}); // vertex bc

    test_pts.push_back({{+d, 0.5, 0.5}, classifier::Location::Inside});
    test_pts.push_back({{-d, 0.5, 0.5}, classifier::Location::Outside});
    test_pts.push_back({{+d, +d, 0.5}, classifier::Location::Inside});
    test_pts.push_back({{-d, -d, 0.5}, classifier::Location::Outside});
    test_pts.push_back({{+d, +d, +d}, classifier::Location::Inside});
    test_pts.push_back({{-d, -d, -d}, classifier::Location::Outside});

    // const testutil::fs::path temp_dir = testutil::makeTempDir("classifier_test");
    // const testutil::fs::path stl_file = temp_dir / "stl.vtk";
    // const testutil::fs::path octree_file = temp_dir / "octree.vtk";
    // const testutil::fs::path pts_file = temp_dir / "pts.vtk";

    // vtk::vtkWriter::checkSTL(stl_file, tris);

    // std::vector<geometry::aabb> all_boxes;
    // root.get_boxes(all_boxes);
    // vtk::vtkWriter::checkBoxes(octree_file, all_boxes);

    // vtk::vtkWriter::checkPoints(pts_file, test_pts);

    // checks
    testutil::expectLoc(clsfr, test_pts[0].p, classifier::Location::Inside, "inside point failed");
    testutil::expectLoc(clsfr, test_pts[1].p, classifier::Location::Outside, "outside point failed");
    testutil::expectLoc(clsfr, test_pts[2].p, classifier::Location::Boundary, "face boundary failed");
    testutil::expectLoc(clsfr, test_pts[3].p, classifier::Location::Boundary, "edge boundary failed");
    testutil::expectLoc(clsfr, test_pts[4].p, classifier::Location::Boundary, "vertex boundary failed");

    // slight perturb and check
    testutil::expectLoc(clsfr, test_pts[5].p, classifier::Location::Inside,  "face +delta should be inside");
    testutil::expectLoc(clsfr, test_pts[6].p, classifier::Location::Outside, "face -delta should be outside");
    testutil::expectLoc(clsfr, test_pts[7].p, classifier::Location::Inside,  "edge +delta should be inside");
    testutil::expectLoc(clsfr, test_pts[8].p, classifier::Location::Outside, "edge -delta should be outside");
    testutil::expectLoc(clsfr, test_pts[9].p, classifier::Location::Inside,  "vertex +delta should be inside");
    testutil::expectLoc(clsfr, test_pts[10].p, classifier::Location::Outside, "vertex -delta should be outside");

    // testutil::cleanupDir(temp_dir);
    std::cout << "[PASS] Classifier unit tests passed.\n";
    return 0;
}