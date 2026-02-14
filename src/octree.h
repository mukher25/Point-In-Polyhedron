#ifndef OCTREE_H
#define OCTREE_H

#include <vector>
#include <memory>
#include <cmath>

#include "geometry.h"
#include "constants.h"
#include "ray.h"

namespace octree
{
    using namespace geometry;

    // filter status - detailed ray casting only to be done on BOUNDARY nodes
    enum class nodeStatus {UNKNOWN, BOUNDARY, EXTERNAL, INTERNAL};

    class octreeNode{
    public:
        aabb box;
        std::vector<size_t> tri_idx; // id to orignal tris
        std::unique_ptr<octreeNode> children[8]; // automatically does memory management
        bool is_leaf = true;
        nodeStatus status = nodeStatus::UNKNOWN;
        int min_tri;
        int max_depth;

        octreeNode(const aabb& b, int minTri, int maxDepth) : box(b), min_tri(minTri), max_depth(maxDepth) {}

        // to check if triangle intersects a bb
        bool intersects(const triangle& tri, const aabb& bb){
            
            // quick check on bb level
            if (!tri.bnds.intersects(bb)) return false;

            // plane intersection check
            pnt3d c = (bb.max + bb.min)*0.5; // center
            pnt3d e = (bb.max - bb.min)*0.5; // half radius calc
            double r = e.x * std::abs(tri.norm.x) + e.y * std::abs(tri.norm.y) + e.z * std::abs(tri.norm.z);
            double s = tri.norm & (c - tri.verts[0]); // center to tri plane distance
            return std::abs(s) <= r;

        }

        bool intersect_box(const aabb& qbox) const{
            // quick return if not even intersecting
            if (!box.intersects(qbox)) return false;

            // if leaf then check if tris
            if (is_leaf) return !tri_idx.empty();

            //recurse
            for (const auto& child : children)
                if (child && child->intersect_box(qbox)) return true;
            
            return false;
        }

        void subDivide(const std::vector<triangle>& tris, int depth){

            if (tri_idx.size() <= size_t(min_tri) || depth >= max_depth) return; // done

            pnt3d mid = (box.max + box.min)*0.5;
            is_leaf = false;

            // 8 children
            for (int i=0; i<8; ++i){

                aabb childBox;

                // bitwise masking for box defintions
                childBox.min.x = (i&1) ? mid.x : box.min.x;
                childBox.max.x = (i&1) ? box.max.x : mid.x;
                childBox.min.y = (i&2) ? mid.y : box.min.y;
                childBox.max.y = (i&2) ? box.max.y : mid.y;
                childBox.min.z = (i&4) ? mid.z : box.min.z;
                childBox.max.z = (i&4) ? box.max.z : mid.z;

                auto child = std::make_unique<octreeNode>(childBox, min_tri, max_depth);

                for (size_t id:tri_idx)
                    if (intersects(tris[id], childBox))
                        child->tri_idx.push_back(id);

                if (!child->tri_idx.empty()){
                    child->subDivide(tris, depth+1);
                    children[i] = std::move(child);
                }

            }
            
            // they are transferred to children at this point
            tri_idx.clear();
            tri_idx.shrink_to_fit();

        }

        // traversal
        void getIntersectedLeafs(const ray::Ray& ray, std::vector<const octreeNode*>& hits) const {
            if (!ray.intersect(box)) return;

            if (is_leaf) {
                if (!tri_idx.empty()) hits.push_back(this); // only want to hold boundaries
                return;
            }

            for (const auto& child : children) {
                if (child) child->getIntersectedLeafs(ray, hits);
            }
        }

        // visualization
        void get_boxes(std::vector<aabb>& box_list) const{
            box_list.push_back(box);
            if (!is_leaf)
                for (const auto& child : children)
                    if (child) child->get_boxes(box_list);
        }

    };
}

#endif