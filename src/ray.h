#ifndef RAY_H
#define RAY_H

#include <cmath>
#include <limits>
#include <algorithm>

#include "geometry.h"
#include "constants.h"

namespace ray
{
    using namespace geometry;

    class Ray{
    public:
        pnt3d origin;
        pnt3d dir;
        pnt3d inv_dir; // cachje for quick calc

        Ray(const pnt3d& o, const pnt3d& d): origin(o), dir(d.normalize()){
            inv_dir.x = (std::abs(dir.x) < constants::SMALL) ? constants::BIG : 1.0 / dir.x;
            inv_dir.y = (std::abs(dir.y) < constants::SMALL) ? constants::BIG : 1.0 / dir.y;
            inv_dir.z = (std::abs(dir.z) < constants::SMALL) ? constants::BIG : 1.0 / dir.z;
        }

        // tri interesection
        // moller trumbore
        double intersect(const triangle& tri) const{
            pnt3d e1 = tri.verts[1] - tri.verts[0];
            pnt3d e2 = tri.verts[2] - tri.verts[0];

            pnt3d crs = dir ^ e2;
            double dot = e1 & crs;

            // parallel?
            if (dot > -constants::EPSILON && dot < constants::EPSILON) return -1.;

            double f = 1./dot;
            pnt3d s = origin - tri.verts[0];
            double u = f * (s & crs);

            // outside?
            if (u < 0. || u > 1.) return -1.;

            pnt3d q = s ^ e1;
            double v = f * (dir & q);

            // outside?
            if (v < 0. || (u + v) > 1.) return -1.;

            // ambuguity
            if (u < constants::EPSILON || v < constants::EPSILON || (1. - (u+v)) < constants::EPSILON) return -2.;

            double t = f*(e2&q);

            // positive side
            if (t > constants::EPSILON) return t;

            return -1.;
        }

        // aabb intersection
        // slabs
        bool intersect(const aabb& box) const{

            // x check
            double tx1 = (box.min.x - origin.x) * inv_dir.x;
            double tx2 = (box.max.x - origin.x) * inv_dir.x;

            double tmin = std::min(tx1, tx2);
            double tmax = std::max(tx1, tx2);

            // y checl
            double ty1 = (box.min.y - origin.y) * inv_dir.y;
            double ty2 = (box.max.y - origin.y) * inv_dir.y;

            tmin = std::max(tmin, std::min(ty1, ty2));
            tmax = std::min(tmax, std::max(ty1, ty2));

            // z check
            double tz1 = (box.min.z - origin.z) * inv_dir.z;
            double tz2 = (box.max.z - origin.z) * inv_dir.z;

            tmin = std::max(tmin, std::min(tz1, tz2));
            tmax = std::min(tmax, std::max(tz1, tz2));

            return tmax >= tmin && tmax >= 0.0;
        }

    };
}

#endif
