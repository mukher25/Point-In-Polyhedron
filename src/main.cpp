#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <random>
#include <fstream>
#include <sstream>
#include <functional>
#include <map>
#include <chrono>

#include "stl_parser.h"
#include "geometry.h"
#include "octree.h"
#include "constants.h"
#include "log.h"
#include "vtk.h"
#include "ray.h"
#include "classifier.h"
#include "cmdline.h"
#include "mesher.h"

// helper to write csv
void writeCsv(const std::string& filepath, const std::vector<vtk::PointStatus>& results, bool pts_only){
    std::ofstream file(filepath);
    if (!file.is_open()) {
        log(LogLevel::ERROR, "Could not open output CSV: " + filepath);
        return;
    }

    if (pts_only){
        file << "x,y,z\n";
        for (const auto& r : results)
            file << r.p.x << "," << r.p.y << "," << r.p.z << "\n";
    }
    else{
        file << "x,y,z,location\n";
        for (const auto& r : results)
            file << r.p.x << "," << r.p.y << "," << r.p.z << "," << static_cast<int>(r.loc) << "\n";
    }

    log(LogLevel::INFO, "Wrote results to " + filepath);
}

// helper to read csv
std::vector<geometry::pnt3d> readCsv(const std::string& filepath){
    std::vector<geometry::pnt3d> points;
    std::ifstream file(filepath);
    if (!file.is_open()) {
        log(LogLevel::ERROR, "Could not open input CSV: " + filepath);
        return points;
    }

    std::string line;
    int line_num = 0;
    while (std::getline(file, line)) {
        line_num++;
        if (line.empty()) continue;

        std::string parse_line = line;
        std::replace(parse_line.begin(), parse_line.end(), ',', ' ');

        std::stringstream ss(parse_line);
        double x, y, z;

        if (ss >> x >> y >> z) {
            points.push_back({x, y, z});
        } else {
            if (isdigit(line[0]) || line[0] == '-') {
                log(LogLevel::WARNING, "Skipping malformed line " + std::to_string(line_num));
            }
        }
    }
    log(LogLevel::INFO, "Read " + std::to_string(points.size()) + " points from " + filepath);
    return points;
}

// workers
void runGen(const cmdline::opts& opts, classifier::pntClassifier& classifier, const geometry::aabb& bounds){
    log(LogLevel::INFO, "Mode: Generation (" + std::to_string(opts.n) + " points)");

    std::mt19937 gen(static_cast<std::mt19937::result_type>(opts.seed));
    std::uniform_real_distribution<> disX(bounds.min.x, bounds.max.x);
    std::uniform_real_distribution<> disY(bounds.min.y, bounds.max.y);
    std::uniform_real_distribution<> disZ(bounds.min.z, bounds.max.z);

    std::vector<vtk::PointStatus> results;
    results.reserve(opts.n);

    auto t_start = std::chrono::high_resolution_clock::now();
    for(size_t i=0; i<opts.n; ++i) {
        geometry::pnt3d p = {disX(gen), disY(gen), disZ(gen)};
        results.push_back({p, classifier.classify(p)});
    }
    auto t_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> t_elapsed = t_end - t_start;
    log(LogLevel::INFO, "Queried " + std::to_string(opts.n) + " points in " + std::to_string(t_elapsed.count()) + " seconds.");
    if (opts.n > 0) {
        double avg_micro = ((t_elapsed.count()) / double(opts.n));
        log(LogLevel::INFO, "Average time per point: " + std::to_string(avg_micro) + " seconds.");
    }

    writeCsv(opts.results_out, results, false);

    if (!opts.points_out.empty() && opts.points_out != opts.results_out)writeCsv(opts.points_out, results, true);
    if (!opts.vtk_points.empty()) vtk::vtkWriter::checkPoints(opts.vtk_points, results);
}

void runClassify(const cmdline::opts& opts, classifier::pntClassifier& classifier){
    log(LogLevel::INFO, "Mode: Classify (" + opts.in_csv + ")");

    std::vector<geometry::pnt3d> inputs = readCsv(opts.in_csv);
    if (inputs.empty()) log(LogLevel::ERROR, "No input query points read!");

    std::vector<vtk::PointStatus> results;
    results.reserve(inputs.size());

    auto t_start = std::chrono::high_resolution_clock::now();

    for (const auto& p : inputs)
        results.push_back({p, classifier.classify(p)});

    auto t_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> t_elapsed = t_end - t_start;

    log(LogLevel::INFO, "Queried " + std::to_string(inputs.size()) + " points in " + std::to_string(t_elapsed.count()) + " seconds.");
    if (!inputs.empty()) {
        double avg_micro = ((t_elapsed.count()) / double(inputs.size()));
        log(LogLevel::INFO, "Average time per point: " + std::to_string(avg_micro) + " seconds.");
    }

    writeCsv(opts.out_csv, results, false);
    if (!opts.vtk_points.empty()) vtk::vtkWriter::checkPoints(opts.vtk_points, results);
}

void runMesh(const cmdline::opts& opts, const std::vector<geometry::triangle>& tris, classifier::pntClassifier& classifier, const octree::octreeNode& root, const geometry::aabb& bounds){
    log(LogLevel::INFO, "Mode: Adaptive Meshing");
    
    if (!opts.vtk_mesh.empty()) {
        auto t_start = std::chrono::high_resolution_clock::now();
        std::vector<geometry::hex> vol = mesher::MeshGenerator::mesh(classifier,root, bounds, tris, opts.mesh_res, opts.mesh_depth);
        auto t_end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> t_elapsed = t_end - t_start;
        log(LogLevel::INFO, "Meshing generated in " + std::to_string(t_elapsed.count()) + " seconds.");
        vtk::vtkWriter::checkHex(opts.vtk_mesh, vol);
    }
    else {
        log(LogLevel::WARNING, "No output specified. Use --vtk-mesh <file.vtk>");
    }
}

int main(int argc, char* argv[]){

    // get command line arguments
    auto opt_val = cmdline::parse(argc, argv);
    if (!opt_val)
        return 0;
    cmdline::opts& opts = *opt_val;

    // load mesh
    log(LogLevel::INFO, "Loading Mesh: " + opts.stl_path);
    auto t_start_stl = std::chrono::high_resolution_clock::now();
    std::vector<geometry::triangle> tris = stl::stlReader::load(opts.stl_path);
    auto t_end_stl = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> t_stl = t_end_stl - t_start_stl;
    log(LogLevel::INFO, std::to_string(tris.size()) + " STL traingles read in " + std::to_string(t_stl.count()) + " seconds.");
    if (tris.empty())
        log(LogLevel::ERROR, "Mesh is empty!");

    // write this vtk if needed
    if (!opts.vtk_stl.empty()) vtk::vtkWriter::checkSTL(opts.vtk_stl, tris);

    // validation of assumptions
    stl::stlReader::validateMesh(tris);

    // boundding box
    geometry::aabb root_bounds = tris[0].bnds;
    for (const auto& t : tris){
        root_bounds.min.x = std::min(root_bounds.min.x, t.bnds.min.x);
        root_bounds.min.y = std::min(root_bounds.min.y, t.bnds.min.y);
        root_bounds.min.z = std::min(root_bounds.min.z, t.bnds.min.z);

        root_bounds.max.x = std::max(root_bounds.max.x, t.bnds.max.x);
        root_bounds.max.y = std::max(root_bounds.max.y, t.bnds.max.y);
        root_bounds.max.z = std::max(root_bounds.max.z, t.bnds.max.z);
    }

    // padding
    geometry::pnt3d dims = root_bounds.max - root_bounds.min;
    double max_dim = std::max({dims.x, dims.y, dims.z});
    double margin_size = max_dim*opts.margin;

    root_bounds.min = root_bounds.min + geometry::pnt3d{-margin_size, -margin_size, -margin_size};
    root_bounds.max = root_bounds.max + geometry::pnt3d{margin_size, margin_size, margin_size};

    // octree
    log(LogLevel::INFO, "Building Octree...");
    auto t_start_octree = std::chrono::high_resolution_clock::now();
    octree::octreeNode root(root_bounds, opts.min_tri, opts.max_depth);
    for (size_t i = 0; i < tris.size(); ++i)
        root.tri_idx.push_back(i);
    root.subDivide(tris, 0);
    auto t_end_octree = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> t_octree = t_end_octree - t_start_octree;
    log(LogLevel::INFO, "Octree build complete in " + std::to_string(t_octree.count()) + " seconds.");

    if (!opts.vtk_boxes.empty()){
        std::vector<geometry::aabb> all_boxes;
        root.get_boxes(all_boxes);
        vtk::vtkWriter::checkBoxes(opts.vtk_boxes, all_boxes);
    }

    // classifier
    log(LogLevel::INFO, "Initializing Classifier (Voxel Fill)...");
    auto t_start_cls = std::chrono::high_resolution_clock::now();
    classifier::pntClassifier clsfr(tris, root);
    auto t_end_cls = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> t_cls = t_end_cls - t_start_cls;
    log(LogLevel::INFO, "Classifier initialized in " + std::to_string(t_cls.count()) + " seconds.");

    // results for points
    std::map<cmdline::Mode, std::function<void()>> workflows;

    workflows[cmdline::Mode::Gen]      = [&]() { runGen(opts, clsfr, root_bounds); };
    workflows[cmdline::Mode::Classify] = [&]() { runClassify(opts, clsfr); };
    workflows[cmdline::Mode::Mesh]     = [&]() { runMesh(opts, tris, clsfr, root, root_bounds); };

    if (workflows.count(opts.mode))
        workflows[opts.mode]();
    else
        log(LogLevel::ERROR, "Unknown execution mode selected.");

    return 0;

}
