#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <cmath>
#include <array>
#include <algorithm>

#include "constants.h"
#include "log.h"

namespace geometry
{
    // 3d point data structure
    struct pnt3d{
        double x, y, z;

        // operators
        pnt3d operator-(const pnt3d& v) const {return {x-v.x, y-v.y, z-v.z};}
        pnt3d operator+(const pnt3d& v) const {return {x+v.x, y+v.y, z+v.z};}
        pnt3d operator*(const double s) const {return {x*s, y*s, z*s};}
        double operator&(const pnt3d& v) const {return (x*v.x + y*v.y + z*v.z);} // dot product
        pnt3d operator^(const pnt3d& v) const {return {y * v.z - z * v.y,z * v.x - x * v.z,x * v.y - y * v.x};} // cross product
        
        // other required stuff
        double length2() const { return (x*x + y*y + z*z);}
        double length() const {return std::sqrt(length2());}
        pnt3d normalize() const {
            double len2 = length2(); 
            if (length2() < constants::SMALL)
                return {0., 0., 0.};
            double len = std::sqrt(len2);
            return {x/len, y/len, z/len};
        }
    };


    // axis aligned bounding box data structure
    struct aabb{
        pnt3d min, max;

        bool contains(const pnt3d& p) const{
            return (p.x >= min.x && p.x <= max.x &&
                    p.y >= min.y && p.y <= max.y &&
                    p.z >= min.z && p.z <= max.z);
        }

        bool intersects(const aabb& other) const{
            return (min.x <= other.max.x && max.x >= other.min.x &&
                    min.y <= other.max.y && max.y >= other.min.y && 
                    min.z <= other.max.z && max.z >= other.min.z);
        }
    };

    // data structure for traingles from stl
    struct triangle{
        std::array<pnt3d, 3> verts;
        pnt3d norm;
        aabb bnds;

        // evaluate and store bounding box and normal, choosing to ignore the stl normal for better control
        void compute(){

            bnds.min = {
                std::min({verts[0].x, verts[1].x, verts[2].x}), 
                std::min({verts[0].y, verts[1].y, verts[2].y}), 
                std::min({verts[0].z, verts[1].z, verts[2].z})
            };

            bnds.max = {
                std::max({verts[0].x, verts[1].x, verts[2].x}), 
                std::max({verts[0].y, verts[1].y, verts[2].y}), 
                std::max({verts[0].z, verts[1].z, verts[2].z})
            };

            pnt3d e1 = verts[1] - verts[0];
            pnt3d e2 = verts[2] - verts[0];
            pnt3d crs = e1^e2;
            double len = crs.length();

            // degenerate tri
            if (len <= constants::SMALL){
                pnt3d cent = verts[0] + verts[1] + verts[2];
                log(LogLevel::WARNING, "Degenerate Triangle found at: " + std::to_string(cent.x) + ", " + std::to_string(cent.y) + ", " + std::to_string(cent.z));
                norm = {0.,0.,0.};
            }
            else{
                norm = crs.normalize();
            }
        }
    };

    // data structure for hex cell - very basic, first bottom face ccw, then top face ccw
    struct hex{
        std::array<pnt3d, 8> verts;

        pnt3d center() const{
            pnt3d c = {0.,0.,0.};
            for (const auto& p : verts) c = c + p;
            return c*0.125;
        }
    };
}
#endif