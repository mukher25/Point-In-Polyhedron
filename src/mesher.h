#ifndef MESHER_H
#define MESHER_H

#include <algorithm>
#include <string>
#include <vector>

#include "geometry.h"
#include "classifier.h"
#include "octree.h"
#include "vtk.h"
#include "log.h"
#include "constants.h"

namespace mesher
{
    using namespace geometry;

    class MeshGenerator{
    public:
        static std::vector<hex> mesh(classifier::pntClassifier& clsfr, const octree::octreeNode& root, const aabb& bounds, const std::vector<triangle>& tris, int initial_res, int max_depth){
            
            log(LogLevel::INFO, "Starting Mesh Generation...");

            std::vector<hex> result;

            if (initial_res <= 0)
                log(LogLevel::ERROR, "Invalid --mesh-res value. It must be > 0.");
            if (max_depth < 0)
                log(LogLevel::ERROR, "Invalid --mesh-depth value. It must be >= 0.");

            const std::size_t total_coarse_cells =
                static_cast<std::size_t>(initial_res) *
                static_cast<std::size_t>(initial_res) *
                static_cast<std::size_t>(initial_res);
            const std::size_t progress_step = std::max<std::size_t>(1, total_coarse_cells / 20);
            std::size_t processed_cells = 0;
            std::vector<unsigned char> level_reported(static_cast<std::size_t>(max_depth) + 1, 0);

            log(
                LogLevel::INFO,
                "Mesher coarse grid: " + std::to_string(initial_res) + "^3 (" +
                std::to_string(total_coarse_cells) + " cells), max depth " +
                std::to_string(max_depth)
            );
            
            // initial coarse grid generation
            double x_step = (bounds.max.x - bounds.min.x) / initial_res;
            double y_step = (bounds.max.y - bounds.min.y) / initial_res;
            double z_step = (bounds.max.z - bounds.min.z) / initial_res;

            for (int i=0; i<initial_res; ++i){
                for (int j=0; j<initial_res; ++j){
                    for (int k=0; k<initial_res; ++k){
                        aabb cell;
                        cell.min = {bounds.min.x + i*x_step, bounds.min.y + j*y_step, bounds.min.z + k*z_step};
                        cell.max = {bounds.min.x + (i+1)*x_step, bounds.min.y + (j+1)*y_step, bounds.min.z + (k+1)*z_step};

                        // recursive refinement
                        _refine(cell, 0, max_depth, tris, &root, clsfr, result, level_reported);
                        processed_cells++;

                        if ((processed_cells % progress_step) == 0 || processed_cells == total_coarse_cells){
                            const double pct = (100.0 * static_cast<double>(processed_cells)) / static_cast<double>(total_coarse_cells);
                            log(
                                LogLevel::INFO,
                                "Mesher progress: " + std::to_string(processed_cells) + "/" +
                                std::to_string(total_coarse_cells) + " coarse cells (" +
                                std::to_string(pct) + "%)"
                            );
                        }
                    }
                }
            }

            return result;
        }

    private:
        static hex _createHex(const aabb& cell){
            hex h;
            h.verts[0] = {cell.min.x, cell.min.y, cell.min.z};
            h.verts[1] = {cell.max.x, cell.min.y, cell.min.z};
            h.verts[2] = {cell.max.x, cell.max.y, cell.min.z};
            h.verts[3] = {cell.min.x, cell.max.y, cell.min.z};
            h.verts[4] = {cell.min.x, cell.min.y, cell.max.z};
            h.verts[5] = {cell.max.x, cell.min.y, cell.max.z};
            h.verts[6] = {cell.max.x, cell.max.y, cell.max.z};
            h.verts[7] = {cell.min.x, cell.max.y, cell.max.z};
            return h;
        }

        static aabb _createChild(const aabb& parent, const pnt3d& p, int i){
            aabb child;
            child.min.x = (i&1) ? p.x : parent.min.x; 
            child.max.x = (i&1) ? parent.max.x : p.x;

            child.min.y = (i&2) ? p.y : parent.min.y; 
            child.max.y = (i&2) ? parent.max.y : p.y;

            child.min.z = (i&4) ? p.z : parent.min.z; 
            child.max.z = (i&4) ? parent.max.z : p.z;
            return child;
        }

        static void _collectIntersectingTriIds(const octree::octreeNode* node, const aabb& box, std::vector<size_t>& tri_ids){
            if (!node || !node->box.intersects(box)) return;

            if (node->is_leaf){
                for (size_t idx : node->tri_idx){
                    tri_ids.push_back(idx);
                }
                return;
            }

            for (const auto& child : node->children){
                if (child) _collectIntersectingTriIds(child.get(), box, tri_ids);
            }
        }

        static bool _isFlat(const std::vector<size_t>& tri_ids, const std::vector<triangle>& tris){
            if (tri_ids.empty()) return true;

            pnt3d ref_norm = {0.,0.,0.};
            bool found_ref = false;
            for (size_t idx : tri_ids){
                if (tris[idx].norm.length2() > constants::SMALL){
                    ref_norm = tris[idx].norm;
                    found_ref = true;
                    break;
                }
            }
            if (!found_ref) return true;

            for (size_t idx : tri_ids){
                if (tris[idx].norm.length2() <= constants::SMALL) continue;
                double dot = std::abs(ref_norm & tris[idx].norm);
                if (dot < constants::FLAT_THRESHOLD) return false;
            }
            return true;
        }

        static void _refine(const aabb& box, int current_depth, int max_depth, const std::vector<triangle>& tris, const octree::octreeNode* root, classifier::pntClassifier& clsfr, std::vector<hex>& result, std::vector<unsigned char>& level_reported){
            if (current_depth >= 0 && current_depth <= max_depth) {
                const std::size_t d = static_cast<std::size_t>(current_depth);
                if (!level_reported[d]) {
                    level_reported[d] = 1;
                    log(
                        LogLevel::INFO,
                        "Mesher refinement level reached: " +
                        std::to_string(current_depth) + "/" + std::to_string(max_depth)
                    );
                }
            }

            bool intersects = (root) ? root->intersect_box(box) : false;
            
            if (!intersects){
                // no need to split, if inside make cell
                if (clsfr.classify((box.min + box.max)*0.5) == classifier::Location::Inside)
                    result.push_back(_createHex(box));
                return;
            }

            // on boundary, recursive split
            std::vector<size_t> tri_ids;
            tri_ids.reserve(64);
            _collectIntersectingTriIds(root, box, tri_ids);
            std::sort(tri_ids.begin(), tri_ids.end());
            tri_ids.erase(std::unique(tri_ids.begin(), tri_ids.end()), tri_ids.end());

            bool force_stop = false;
            // if (current_depth > 2 && _isFlat(tri_ids, tris)) force_stop = true;

            if (current_depth >= max_depth || force_stop){
                pnt3d center = (box.min + box.max)*0.5;
                if (clsfr.classify(center) == classifier::Location::Inside)
                    result.push_back(_createHex(box));
                return;
            }

            pnt3d mid = (box.min + box.max)*0.5;
            for(int i=0; i<8; ++i){
                aabb childBox = _createChild(box, mid, i);

                // Keep the same octree subtree root and prune via box intersection checks.
                _refine(childBox, current_depth + 1, max_depth, tris, root, clsfr, result, level_reported);
            }

        }
    };
}

#endif
