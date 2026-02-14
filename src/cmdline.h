#ifndef CMDLINE_H
#define CMDLINE_H

#include <string>
#include <vector>
#include <optional>
#include <sstream>
#include <iostream>
#include "log.h"

namespace cmdline
{
    enum class Mode {Gen, Classify, Mesh, Unknown};

    struct opts{
        std::string stl_path;
        double margin = 0.1;
        int min_tri = 20;
        int max_depth = 10;
        Mode mode{Mode::Unknown};

        // generation
        std::size_t n = 1000;
        std::string points_out;
        std::string results_out = "results.csv";
        int seed = 50;

        // classification only
        std::string in_csv;
        std::string out_csv = "results.csv";

        // meshing
        int mesh_res = 10;
        int mesh_depth = 10;

        // vtk options
        std::string vtk_stl;
        std::string vtk_boxes;
        std::string vtk_points;
        std::string vtk_mesh;
    };

    inline std::string usage(const std::string& exe){
        std::ostringstream os;
        os  << "Usage:\n"
            << " Global:\n"
            << "    " << exe << " --stl <file.stl> [COMMAND] [OPTIONS]\n"
            << "    --margin <val>              Bounding box margin ratio (default: 0.1)\n"
            << "    --mintri <val>              Minimum number of triangles per tree for voxel generation (default: 20)\n"
            << "    --maxdepth <val>            Maximum number of splits for voxel generation (default: 10)\n\n"
            << " Commands:\n"
            << "    gen                         Generate random points in bounding box and classify them\n"
            << "      --n <val>                 Number of points to generate (default: 1000)\n"
            << "      --points-out <file.csv>   File to save generated points into\n"
            << "      --results-out <file.csv>  File to save classification results into\n"
            << "      --seed <val>              Random number seed (default: 50)\n\n"
            << "    classify                    Classify points from specified input file\n"
            << "      --in <file.csv>           File to read query points from\n"
            << "      --out <file.csv>          File to save classification results into\n\n"
            << "    mesh                        Create 3D hex mesh for inside\n"
            << "      --mesh-res <val>          Initial number of grid points per axis (default: 10)\n"
            << "      --mesh-depth <val>        Maximum depth of mesh refinement (default: 10)\n"
            << " VTK options:\n"
            << "    --vtk-stl <file.vtk>        File to write read in stl into\n"
            << "    --vtk-boxes <file.vtk>      File to write octree structure into\n"
            << "    --vtk-points <file.vtk>     File to color coded points into\n"
            << "    --vtk-mesh <file.vtk>       File to write mesh into\n";
        return os.str();
    }

    inline std::optional<opts> parse(int argc, char* argv[]){
        if (argc < 2) {
            std::cout << usage(argv[0]);
            return std::nullopt;
        }

        opts options;

        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];

            if (arg == "-h" || arg == "--help") {
                std::cout << usage(argv[0]);
                return std::nullopt;
            }
            else if (arg == "--stl") {
                if (i + 1 < argc) options.stl_path = argv[++i];
                else { log(LogLevel::ERROR, "--stl requires a filename."); return std::nullopt; }
            }
            else if (arg == "--margin") {
                if (i + 1 < argc) options.margin = std::stod(argv[++i]);
            }
            else if (arg == "--mintri") {
                if (i + 1 < argc) options.min_tri = std::stoi(argv[++i]);
            }
            else if (arg == "--maxdepth") {
                if (i + 1 < argc) options.max_depth = std::stoi(argv[++i]);
            }

            // Modes
            else if (arg == "gen") {
                options.mode = Mode::Gen;
            }
            else if (arg == "classify") {
                options.mode = Mode::Classify;
            }
            else if (arg == "mesh") {
                options.mode = Mode::Mesh;
            }

            // VTK
            else if (arg == "--vtk-stl") {
                if (i + 1 < argc) options.vtk_stl = argv[++i];
            }
            else if (arg == "--vtk-boxes") {
                if (i + 1 < argc) options.vtk_boxes = argv[++i];
            }
            else if (arg == "--vtk-points") {
                if (i + 1 < argc) options.vtk_points = argv[++i];
            }
            else if (arg == "--vtk-mesh") {
                if (i + 1 < argc) options.vtk_mesh = argv[++i];
            }

            // Gen / Classify / Mesh Options
            else if (arg == "--n") {
                if (i + 1 < argc) options.n = std::stoul(argv[++i]);
            }
            else if (arg == "--points-out") {
                if (i + 1 < argc) options.points_out = argv[++i];
            }
            else if (arg == "--results-out" || arg == "--out") {
                if (i + 1 < argc) options.results_out = argv[++i];
                options.out_csv = options.results_out;
            }
            else if (arg == "--seed") {
                if (i + 1 < argc) options.seed = std::stoi(argv[++i]);
            }
            else if (arg == "--in") {
                if (i + 1 < argc) options.in_csv = argv[++i];
            }
            else if (arg == "--mesh-res") {
                if (i + 1 < argc) options.mesh_res = std::stoi(argv[++i]);
            }
            else if (arg == "--mesh-depth") {
                if (i + 1 < argc) options.mesh_depth = std::stoi(argv[++i]);
            }
            else {
                log(LogLevel::WARNING, "Unknown argument: " + arg);
            }
        }

        // checks
        if (options.stl_path.empty()) {
            log(LogLevel::ERROR, "No STL file specified. Use --stl <file>.");
            return std::nullopt;
        }

        if (options.mode == Mode::Unknown) {
            log(LogLevel::ERROR, "No valid mode specified (gen, classify, mesh).");
            std::cout << usage(argv[0]);
            return std::nullopt;
        }

        if (options.mode == Mode::Classify && options.in_csv.empty()) {
            log(LogLevel::ERROR, "'classify' mode requires --in <csv_file>.");
            return std::nullopt;
        }

        if (options.mode == Mode::Mesh && options.mesh_res <= 0) {
            log(LogLevel::ERROR, "'mesh' mode requires --mesh-res > 0.");
            return std::nullopt;
        }

        if (options.mode == Mode::Mesh && options.mesh_depth < 0) {
            log(LogLevel::ERROR, "'mesh' mode requires --mesh-depth >= 0.");
            return std::nullopt;
        }

        return options;
    }
}
#endif
