#ifndef CLASSIFIER_H
#define CLASSIFIER_H

#include <vector>
#include <cmath>
#include <unordered_set>

#include "geometry.h"
#include "octree.h"
#include "ray.h"
#include "log.h"

namespace classifier
{
    enum class Location{Outside = 0, Inside = 1, Boundary = 2}; 

    class pntClassifier{
    public:
        
        pntClassifier(const std::vector<geometry::triangle>& m, octree::octreeNode& r) : _mesh(m), _root(r){
            _voxelFill(_root);
        }

        // main one
        // bool isInside(const geometry::pnt3d& p){
        //     const octree::octreeNode* node = _findNode(&_root, p);

        //     // this is where voxel fill helps
        //     if  (node->status == octree::nodeStatus::INTERNAL) return true;
        //     if  (node->status == octree::nodeStatus::EXTERNAL) return false;

        //     // refined stuff
        //     return _castRay(p);
        // }

        Location classify(const geometry::pnt3d& p){
            
            if (!_root.box.contains(p)) return Location::Outside;
            if (_isOnBoundary(p)) return Location::Boundary;

            const octree::octreeNode* node = _findNode(&_root, p);

            // this is where voxel fill helps
            if  (node->status == octree::nodeStatus::INTERNAL) return Location::Inside;
            if  (node->status == octree::nodeStatus::EXTERNAL) return Location::Outside;

            // refined stuff
            return _castRay(p) ? Location::Inside : Location::Outside;
        }

    private:
        const std::vector<geometry::triangle>& _mesh;
        octree::octreeNode& _root; // non const for status update

        // fill status on the aabbs - upfront overhead for later speedup
        void _voxelFill(octree::octreeNode& node){
            if (!node.is_leaf){
                for (auto& child : node.children){
                    if (child) _voxelFill(*child);
                }
                return;
            }

            if (!node.tri_idx.empty())
                node.status = octree::nodeStatus::BOUNDARY;
            else{
                // this is either fully in or fully out
                // ray casting to identify
                geometry::pnt3d center = (node.box.max + node.box.min)*0.5;
                if (_castRay(center))
                    node.status = octree::nodeStatus::INTERNAL;
                else
                    node.status = octree::nodeStatus::EXTERNAL;
            }
        }

        bool _castRay(const geometry::pnt3d& p){

            int attempts = 0; // try 3, ideally first one works, otherwise print warning and continue
            while (attempts < 3){

                geometry::pnt3d dir;
                switch (attempts)
                {
                case 0:
                    dir = {1.0, 0.61803398875, 0.41421356237}; // non-axis-aligned
                    break;
                case 1:
                    dir = {0.37139067635, 1.0, 0.70710678118}; // non-axis-aligned
                    break;
                case 2:
                    dir = {0.57735026919, 0.26794919243, 1.0}; // non-axis-aligned
                    break;
                default:
                    log(LogLevel::ERROR, "Ray casting failed in 3 attempts. Stopping.");
                    break;
                }

                ray::Ray r(p, dir);

                std::vector<const octree::octreeNode*> candidates;
                _root.getIntersectedLeafs(r, candidates);

                int hits = 0;
                bool edge_graze = false;
                std::unordered_set<size_t> tested_tris;

                for (const auto* node : candidates) {
                    for (size_t idx : node->tri_idx){
                        if (tested_tris.find(idx) != tested_tris.end()) continue;
                        tested_tris.insert(idx);
                        double t = r.intersect(_mesh[idx]);
                        if (t==-2){
                            edge_graze = true;
                            break;
                        }
                        if (t > 0.)
                            hits++;
                    }
                    if (edge_graze) break;
                }
                if (edge_graze) {
                    attempts++;
                    continue; 
                }
                if (hits == 0){
                    attempts++;
                    continue;
                }
                return (hits % 2) != 0;
            }

            return false;
        }

        const octree::octreeNode* _findNode(const octree::octreeNode* node, const geometry::pnt3d& p){
            if (node->is_leaf) return node;

            // bitwise index finder
            geometry::pnt3d mid = (node->box.min + node->box.max) * 0.5;
            int idx = 0;
            if (p.x >= mid.x) idx |= 1;
            if (p.y >= mid.y) idx |= 2;
            if (p.z >= mid.z) idx |= 4;

            if (node->children[idx])
                return _findNode(node->children[idx].get(), p);

            return node;
        }

        bool _isOnBoundary(const geometry::pnt3d& p) const{
            const geometry::pnt3d dims = _root.box.max - _root.box.min;
            const double diag = dims.length();
            const double eps_dist = std::max(1e-12, 1e-9 * diag);
            const double eps_bary = 1e-10;  

            geometry::aabb probe{
                {p.x - eps_dist, p.y - eps_dist, p.z - eps_dist},
                {p.x + eps_dist, p.y + eps_dist, p.z + eps_dist}
            };

            std::vector<size_t> tri_ids;
            tri_ids.reserve(64);
            _collectProbeTris(&_root, probe, tri_ids);

            if (tri_ids.empty()) return false;

            std::sort(tri_ids.begin(), tri_ids.end());
            tri_ids.erase(std::unique(tri_ids.begin(), tri_ids.end()), tri_ids.end());

            for (size_t id : tri_ids) {
            if (_ptOnTri(p, _mesh[id], eps_dist, eps_bary))
                return true;
            }
            return false;
            
        }

        void _collectProbeTris(const octree::octreeNode* node, const geometry::aabb& probe, std::vector<size_t>& out) const{
            if (!node || !node->box.intersects(probe)) return;

            if (node->is_leaf) {
                out.insert(out.end(), node->tri_idx.begin(), node->tri_idx.end());
                return;
            }

            for (const auto& child : node->children)
                if (child) _collectProbeTris(child.get(), probe, out);
        }

        bool _ptOnTri(const geometry::pnt3d& p, const geometry::triangle& tri, double eps_dist, double eps_bary) const{
            if (tri.norm.length2() <= constants::SMALL) return false; // degenerate tri

            if (p.x < tri.bnds.min.x - eps_dist || p.x > tri.bnds.max.x + eps_dist ||
                p.y < tri.bnds.min.y - eps_dist || p.y > tri.bnds.max.y + eps_dist ||
                p.z < tri.bnds.min.z - eps_dist || p.z > tri.bnds.max.z + eps_dist) {
                return false;
            }

            const geometry::pnt3d v0 = tri.verts[0];
            const geometry::pnt3d v1 = tri.verts[1];
            const geometry::pnt3d v2 = tri.verts[2];

            const double signed_dist = (p - v0) & tri.norm;
            if (std::abs(signed_dist) > eps_dist) return false;  

            const geometry::pnt3d e0 = v1 - v0;
            const geometry::pnt3d e1 = v2 - v0;
            const geometry::pnt3d vp = p - v0;

            const double d00 = e0 & e0;
            const double d01 = e0 & e1;
            const double d11 = e1 & e1;
            const double d20 = vp & e0;
            const double d21 = vp & e1;

            const double denom = d00 * d11 - d01 * d01;
            if (std::abs(denom) <= constants::SMALL) return false;

            const double v = (d11 * d20 - d01 * d21) / denom;
            const double w = (d00 * d21 - d01 * d20) / denom;
            const double u = 1.0 - v - w;

            return (u >= -eps_bary && v >= -eps_bary && w >= -eps_bary &&
                    u <= 1.0 + eps_bary && v <= 1.0 + eps_bary && w <= 1.0 + eps_bary);
        }
    };

}

#endif
